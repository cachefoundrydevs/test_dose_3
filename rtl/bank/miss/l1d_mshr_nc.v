// ============================================================================
// l1d_mshr_nc.v  -  the NON-CACHEABLE entries of one bank
// ============================================================================
// IN SIMPLE WORDS
//   Some accesses must NOT use the cache: device registers, I/O (the CPU marks them cpu_request_cacheable = 0). Such a
//   request never looks at the cache arrays and never changes them. Stage 2 hands it to this unit, which performs it as
//   ONE single-beat AXI access and answers the CPU through an "MSHR response (2)":
//        load : AXI read (one beat, access size)               -> R beat  -> load data, bytes from bit 0
//        store: AXI write address + data (one beat, strobes = the bytes written) -> B response -> store acknowledge
//
//   The AXI address is the CPU address UNCHANGED (e.g. 0x29FF8) and AxSIZE is the access size (0..3 = 1/2/4/8 bytes).
//   The byte lane inside the beat comes from the low address bits (read: where to pick the data; write: the strobes).
//
// THE QUEUE  (question from an earlier session: "what is the FIFO in l1d_mshr_nc?")
//   Up to MSHR_NC_ENTRIES (2) requests wait in a small FIFO (l1d_fifo u_q). That FIFO IS the set of non-cacheable
//   entries: a request occupies one place from allocation until its answer is taken. The requests are done STRICTLY
//   ONE AT A TIME AND IN ORDER by the head state machine, so device accesses are never merged or reordered.
//   When the queue is full, can_allocate = 0 and Stage 2 makes the request's seat wait (seat state WNC) until the
//   queue pops (entry_freed_pulse).
//   The AXI id of such an access is { nc = 1, bank, 0 }.
//
// THE HEAD STATE MACHINE   (named states H_*)
//   H_IDLE   nothing in progress                                  --queue not empty--> H_ISSUE
//   H_ISSUE  the AXI read, or write address + data, is offered     --axi_*_req_ready--> H_WAIT
//   H_WAIT   waiting for the R beat (load) or the B response (store) --> H_RESP
//   H_RESP   the answer is ready for Stage 3 (response_valid)      --response_taken--> H_IDLE (queue pops)
//   It is a Moore machine (outputs depend on the state).
//
// LIMITS
//   A younger NC request that is already in the pipeline when the queue pops can take the freed place before an older
//   seat in WNC is granted again, so NC accesses may enter the queue out of arrival order. (Observation from code
//   reading, not simulated; for the user to decide whether the LSU's NC ordering makes it matter.)
//
// PORTS (new name, old name, meaning)
//   allocate_valid                       (was alloc)                    queue a non-cacheable request
//   allocate_seat                        (was a_seat)                   LSU seat of the request
//   allocate_is_write                    (was a_rd_wr)                  the request is a store
//   allocate_access_size                 (was a_size)                   request size
//   allocate_master_id                   (was a_mid)                    request master id
//   allocate_rob_index                   (was a_idx)                    request ROB index
//   allocate_port                        (new in A2)                    request port of the request (queued, sent back in the response)
//   allocate_addr                        (was a_addr)                   exact byte address
//   allocate_store_data                  (was a_wdata)                  store data
//   can_allocate                         (was can_alloc)                the queue has a free place
//   axi_read_req_valid                   (was rd_valid)                 the head wants to send an AXI read
//   axi_read_req_ready                   (was rd_ready)                 the read channel takes it
//   axi_read_req_addr                    (was rd_addr)                  byte address
//   axi_read_req_size                    (was rd_size)                  AxSIZE
//   axi_read_beat_valid                  (was r_valid)                  the read beat arrived
//   axi_read_beat_data                   (was r_data)                   beat data
//   axi_read_beat_error                  (was r_err)                    error response
//   axi_write_req_valid                  (was wr_valid)                 the head wants to send an AXI write
//   axi_write_req_ready                  (was wr_ready)                 the write channel takes it
//   axi_write_req_addr                   (was wr_addr)                  byte address
//   axi_write_req_size                   (was wr_size)                  AxSIZE
//   axi_write_req_data                   (was wr_data)                  beat data (store data on its byte lanes)
//   axi_write_req_strobe                 (was wr_strb)                  byte strobes
//   axi_write_resp_valid                 (was b_valid)                  the write response arrived
//   axi_write_resp_error                 (was b_err)                    error response
//   response_valid                       (was resp_valid)               the head has a response for Stage 3
//   response_packet                      (was resp_pkt)                 the response packet
//   response_seat                        (was resp_seat)                LSU seat of the response
//   response_taken                       (was resp_ack)                 Stage 3 took the response
//   queue_empty                          (was empty)                    no non-cacheable request is queued
//   entry_freed_pulse                    (was freed)                    one-cycle pulse: a queue place became free
//
// VERILOG WORDS USED IN THIS FILE
//   {a, b, c}               glue the fields into one packet (queue entry: seat, rd_wr, size, master id, index, address, data)
//   head[Q_ADDR +: ADDR_W]  the address field of the queue entry at the head
//   wdata_ext << {lane, 3'b000}   shift the store data to its byte lane (lane x 8 bits)
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

module l1d_mshr_nc #(`L1D_PARAMS) (
    clk, rst,
    allocate_valid, allocate_seat, allocate_is_write, allocate_access_size, allocate_master_id, allocate_rob_index, allocate_port, allocate_addr, allocate_store_data, can_allocate,
    axi_read_req_valid, axi_read_req_ready, axi_read_req_addr, axi_read_req_size,
    axi_read_beat_valid, axi_read_beat_data, axi_read_beat_error,
    axi_write_req_valid, axi_write_req_ready, axi_write_req_addr, axi_write_req_size, axi_write_req_data, axi_write_req_strobe,
    axi_write_resp_valid, axi_write_resp_error,
    response_valid, response_packet, response_seat, response_taken,
    queue_empty, entry_freed_pulse
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // queue depth (at least 2 because l1d_fifo needs DEPTH >= 2)
    localparam QDEPTH = (MSHR_NC_ENTRIES < 2) ? 2 : MSHR_NC_ENTRIES;
    // queued entry: | port | seat | rd_wr | size | master_id | idx | addr | wdata |
    // layout of one queue entry (highest field first): | port | seat | rd_wr | size | master_id | idx | addr | wdata |   (port added in A2)
    localparam Q_WDATA = 0;
    localparam Q_ADDR  = Q_WDATA + WRITE_DATA_WIDTH;
    localparam Q_IDX   = Q_ADDR  + ADDR_W;
    localparam Q_MID   = Q_IDX   + IDX_BITS;
    localparam Q_SIZE  = Q_MID   + MASTER_ID_BITS;
    localparam Q_RDWR  = Q_SIZE  + MEM_SIZE_BITS;
    localparam Q_SEAT  = Q_RDWR  + 1;
    localparam Q_PORT  = Q_SEAT  + SEAT_W;               // first bit of the request port field
    localparam Q_W     = Q_PORT  + PORT_W;               // whole entry width (PORT_W = 2 bits added for the 4 request ports)

    input  wire                        clk;               // clock: all registers change on its rising edge
    input  wire                        rst;               // synchronous reset, active high (control state goes to its idle value)
    // allocation (Stage 2)
    input  wire                        allocate_valid;    // queue a non-cacheable request
    input  wire [SEAT_W-1:0]           allocate_seat;     // LSU seat of the request
    input  wire                        allocate_is_write; // the request is a store
    input  wire [MEM_SIZE_BITS-1:0]    allocate_access_size;  // request size
    input  wire [MASTER_ID_BITS-1:0]   allocate_master_id;  // request master id
    input  wire [IDX_BITS-1:0]         allocate_rob_index;  // request ROB index
    input  wire [PORT_W-1:0]           allocate_port;     // request port of the request, PORT_W = 2 bits (4 ports)
    input  wire [ADDR_W-1:0]           allocate_addr;     // exact byte address
    input  wire [WRITE_DATA_WIDTH-1:0] allocate_store_data;  // store data
    output wire                        can_allocate;      // the queue has a free place
    // AXI read (single beat)
    output wire                        axi_read_req_valid;  // the head wants to send an AXI read
    input  wire                        axi_read_req_ready;  // the read channel takes it
    output wire [ADDR_W-1:0]           axi_read_req_addr; // byte address
    output wire [2:0]                  axi_read_req_size; // AxSIZE
    input  wire                        axi_read_beat_valid;  // the read beat arrived
    input  wire [AXI_DATA_W-1:0]       axi_read_beat_data;  // beat data
    input  wire                        axi_read_beat_error;  // error response
    // AXI write (single beat)
    output wire                        axi_write_req_valid;  // the head wants to send an AXI write
    input  wire                        axi_write_req_ready;  // the write channel takes it
    output wire [ADDR_W-1:0]           axi_write_req_addr;  // byte address
    output wire [2:0]                  axi_write_req_size;  // AxSIZE
    output wire [AXI_DATA_W-1:0]       axi_write_req_data;  // beat data (store data on its byte lanes)
    output wire [AXI_STRB_W-1:0]       axi_write_req_strobe;  // byte strobes
    input  wire                        axi_write_resp_valid;  // the write response arrived
    input  wire                        axi_write_resp_error;  // error response
    // response (2) to Stage 3
    output wire                        response_valid;    // the head has a response for Stage 3
    output wire [RESP_W-1:0]           response_packet;   // the response packet
    output wire [SEAT_W-1:0]           response_seat;     // LSU seat of the response
    input  wire                        response_taken;    // Stage 3 took the response
    // status
    output wire                        queue_empty;       // no non-cacheable request is queued
    output wire                        entry_freed_pulse; // one-cycle pulse: a queue place became free

    // ---------------- queue ----------------
    // ---- the queue of waiting non-cacheable requests ----
    wire [Q_W-1:0] head;
    wire           q_empty, q_full;
    wire           q_pop;

    // push = allocation by Stage 2, pop = when the head's answer is taken; the head is visible on `head` without a read delay
    l1d_fifo #(.WIDTH(Q_W), .DEPTH(QDEPTH)) u_q (
        .clk        (clk),
        .rst        (rst),
        .push       (allocate_valid),
        .push_data  ({allocate_port, allocate_seat, allocate_is_write, allocate_access_size, allocate_master_id, allocate_rob_index, allocate_addr, allocate_store_data}),
        .pop        (q_pop),
        .front_data (head),
        .empty      (q_empty),
        .full       (q_full),
        .used_slots (),
        .free_slots ()
    );
    // MSHR_NC_ENTRIES = 1 still uses a 2-deep FIFO, so limit the count here
    reg [1:0] n_used;                                     // number of used places, counted separately because MSHR_NC_ENTRIES = 1 still uses a 2-deep FIFO
    assign can_allocate = (MSHR_NC_ENTRIES < 2) ? (n_used == 2'd0) : !q_full;

    // the fields of the HEAD entry (h_ = head): address, store data, size, direction, and the byte lane inside the AXI beat
    // TIMING OPT (N-2, session 41): the head entry is copied to a register in EVERY cycle (head_q), and the request outputs, the state
    // machine and the response read the copy, not the FIFO's front mux. The copy equals the head in every state except H_IDLE: the head
    // is the same entry from the cycle before H_ISSUE (H_IDLE with the queue not empty) until it is popped in H_RESP, and only a pop
    // changes the front of a non-empty FIFO. In H_IDLE the request and response outputs are not valid, so nobody reads them.
    // The byte lane and the access size are also kept as one-hot / mask registers (lane_oh_q, szk_q), so the write strobe, the shifted
    // write data and the load data are AND-OR selects (no adder, compare or shifter behind the head).
    localparam WBYTES = WRITE_DATA_WIDTH / 8;               // bytes of the CPU data, WBYTES = 8
    reg  [Q_W-1:0]              head_q;                     // copy of the head entry, Q_W = 141 bits (139 + PORT_W = 2)
    reg  [AXI_STRB_W-1:0]       lane_oh_q;                  // one-hot byte lane of the head inside the beat, AXI_STRB_W = 16 bits
    reg  [WBYTES-1:0]           szk_q;                      // szk_q[k] = byte k is inside the access size of the head, WBYTES = 8 bits
    wire [AXI_OFF_W-1:0]        f_lane = head[Q_ADDR +: AXI_OFF_W];                  // byte lane of the FIFO head, AXI_OFF_W = 4 bits
    wire [MEM_SIZE_BITS-1:0]    f_size = head[Q_SIZE +: MEM_SIZE_BITS];              // access size of the FIFO head, MEM_SIZE_BITS = 2 bits
    integer ql;
    always @(posedge clk) begin
        head_q <= head;
        for (ql = 0; ql < AXI_STRB_W; ql = ql + 1)
            lane_oh_q[ql] <= (f_lane == ql);
        for (ql = 0; ql < WBYTES; ql = ql + 1)
            szk_q[ql] <= (ql < (1 << f_size));
    end
    wire [ADDR_W-1:0]           h_addr  = head_q[Q_ADDR +: ADDR_W];
    wire [WRITE_DATA_WIDTH-1:0] h_wdata = head_q[Q_WDATA +: WRITE_DATA_WIDTH];
    wire [MEM_SIZE_BITS-1:0]    h_size  = head_q[Q_SIZE +: MEM_SIZE_BITS];
    wire                        h_rd_wr = head_q[Q_RDWR];

    // ---------------- head state machine ----------------
    localparam [1:0] H_IDLE  = 2'd0;   // nothing in progress
    localparam [1:0] H_ISSUE = 2'd1;   // AR, or AW + W, offered
    localparam [1:0] H_WAIT  = 2'd2;   // waiting for R or B
    localparam [1:0] H_RESP  = 2'd3;   // answer ready for Stage 3

    reg [1:0]            hs;                              // head state (H_*)
    reg [AXI_DATA_W-1:0] rbeat;                           // the data beat of a load, kept for the response
    reg                  herr;                            // the R or B response of the head had an error

    // ---- the head state machine and the used-places counter (clocked) ----
    always @(posedge clk) begin
        if (rst) begin
            hs     <= H_IDLE;
            herr   <= 1'b0;
            n_used <= 2'd0;
        end else begin
            case ({allocate_valid, q_pop})
                2'b10:   n_used <= n_used + 1'b1;
                2'b01:   n_used <= n_used - 1'b1;
                default: n_used <= n_used;
            endcase
            case (hs)
                H_IDLE:  if (!q_empty) hs <= H_ISSUE;
                H_ISSUE: if ((h_rd_wr == OP_READ) ? axi_read_req_ready : axi_write_req_ready) hs <= H_WAIT;
                H_WAIT: begin
                    if (h_rd_wr == OP_READ && axi_read_beat_valid) begin
                        rbeat <= axi_read_beat_data;
                        herr  <= axi_read_beat_error;
                        hs    <= H_RESP;
                    end else if (h_rd_wr == OP_WRITE && axi_write_resp_valid) begin
                        herr  <= axi_write_resp_error;
                        hs    <= H_RESP;
                    end
                end
                H_RESP:  if (response_taken) hs <= H_IDLE;
                default: hs <= H_IDLE;
            endcase
        end
    end

    // the queue pops (and an entry is freed) when the head's answer is taken by Stage 3
    assign q_pop = (hs == H_RESP) && response_taken;
    assign entry_freed_pulse = q_pop;
    assign queue_empty = q_empty;

    // ---------------- AXI requests ----------------
    // ---- AXI requests of the head: strobes = the bytes of the access on their lane ----
    // write strobe: byte i is written when i = lane + k for a byte k inside the access (the old (i >= lane) && (i < lane + nbytes));
    // write data: byte k of the CPU data goes to byte lane + k of the beat (the old wdata << {lane, 3'b000}); both cut at the beat end.
    integer i, k;
    reg [AXI_STRB_W-1:0] strb;                              // AXI_STRB_W = 16 bits
    reg [AXI_DATA_W-1:0] wdata_sh;                          // AXI_DATA_W = 128 bits
    always @* begin
        for (i = 0; i < AXI_STRB_W; i = i + 1) begin
            strb[i]          = 1'b0;
            wdata_sh[i*8 +: 8] = 8'h00;
            for (k = 0; k < WBYTES; k = k + 1)
                if (i - k >= 0) begin
                    strb[i]            = strb[i] | (lane_oh_q[i-k] & szk_q[k]);
                    wdata_sh[i*8 +: 8] = wdata_sh[i*8 +: 8] | ({8{lane_oh_q[i-k]}} & h_wdata[k*8 +: 8]);
                end
        end
    end

    // the head offers an AXI read (load) or write (store) while in H_ISSUE
    assign axi_read_req_valid = (hs == H_ISSUE) && (h_rd_wr == OP_READ);
    // exact LSU address, LSU access size (1 / 2 / 4 / 8 bytes -> AxSIZE 0..3);
    // data and strobe sit on the byte lanes of that address
    wire [2:0] h_axsize = h_size;                           // zero-extended

    assign axi_read_req_addr  = h_addr;
    assign axi_read_req_size  = h_axsize;
    assign axi_write_req_valid = (hs == H_ISSUE) && (h_rd_wr == OP_WRITE);
    assign axi_write_req_addr  = h_addr;
    assign axi_write_req_size  = h_axsize;
    assign axi_write_req_data  = wdata_sh;
    assign axi_write_req_strobe  = strb;

    // ---------------- response ----------------
    // ---- response: pick the loaded bytes out of the data beat (l1d_resp_gen on ONE beat instead of a line) ----
    // load data (the function of l1d_resp_gen on the beat): byte k = beat byte lane + k when k is inside the access and lane + k is inside
    // the beat, else 0; as an AND-OR with the one-hot lane and the size mask
    reg [WRITE_DATA_WIDTH-1:0] load_data;                   // WRITE_DATA_WIDTH = 64 bits
    integer lk, lj;
    always @* begin
        for (lk = 0; lk < WBYTES; lk = lk + 1) begin
            load_data[lk*8 +: 8] = 8'h00;
            for (lj = 0; lj < AXI_STRB_W; lj = lj + 1)
                if (lj + lk < AXI_STRB_W)
                    load_data[lk*8 +: 8] = load_data[lk*8 +: 8] | ({8{szk_q[lk] & lane_oh_q[lj]}} & rbeat[(lj+lk)*8 +: 8]);
        end
    end

// ---- simulation-only trace: queue entries, head state changes, the AXI access, R / B, protocol checks and STUCK reports ----
`ifdef L1D_DEBUG
    // ======================================================================
    // Debug trace (simulation only): queue entries, head state changes, the
    // AXI access, R / B, protocol checks and STUCK reports.
    // ======================================================================
    function [8*5-1:0] dbg_hname;
        input [1:0] s;
        begin
            case (s)
                H_IDLE:  dbg_hname = "IDLE";
                H_ISSUE: dbg_hname = "ISSUE";
                H_WAIT:  dbg_hname = "WAIT";
                default: dbg_hname = "RESP";
            endcase
        end
    endfunction

    reg [1:0] dbg_ps;
    integer   dbg_age;
    initial begin dbg_ps = H_IDLE; dbg_age = 0; end

    always @(posedge clk) begin
        if (rst) begin
            dbg_ps  = H_IDLE;
            dbg_age = 0;
        end else begin
            if (allocate_valid)
                $display("[%0t] L1D NC   %m queued idx=%0d %s addr=%h size=%0d data=%h (can_alloc=%b)",
                         $time, allocate_rob_index, (allocate_is_write ? "ST" : "LD"), allocate_addr, allocate_access_size, allocate_store_data, can_allocate);
            if (hs != dbg_ps) begin
                $display("[%0t] L1D NC   %m head: %0s -> %0s  %s addr=%h (AXI addr %h, size %0d, strobe %h)",
                         $time, dbg_hname(dbg_ps), dbg_hname(hs), (h_rd_wr ? "ST" : "LD"),
                         h_addr, h_addr, h_axsize, strb);
                dbg_ps  = hs;
                dbg_age = 0;
            end else if (hs != H_IDLE) begin
                dbg_age = dbg_age + 1;
                if (dbg_age % `L1D_DBG_WD == 0)
                    $display("[%0t] L1D NC   %m STUCK head in %0s for %0d cycles  %s addr=%h  (ISSUE: AXI master busy; WAIT: no %s from memory; RESP: no response-FIFO place)",
                             $time, dbg_hname(hs), dbg_age, (h_rd_wr ? "ST" : "LD"), h_addr,
                             (h_rd_wr ? "B" : "R beat"));
            end
            if (axi_read_beat_valid) begin
                if (hs != H_WAIT || h_rd_wr != OP_READ)
                    $display("[%0t] L1D NC   %m ERROR NC R beat while the head is in %0s (wrong RID from memory?)",
                             $time, dbg_hname(hs));
                else
                    $display("[%0t] L1D NC   %m R err=%b data=%h", $time, axi_read_beat_error, axi_read_beat_data);
            end
            if (axi_write_resp_valid) begin
                if (hs != H_WAIT || h_rd_wr != OP_WRITE)
                    $display("[%0t] L1D NC   %m ERROR NC B while the head is in %0s (wrong BID from memory?)",
                             $time, dbg_hname(hs));
                else
                    $display("[%0t] L1D NC   %m B err=%b", $time, axi_write_resp_error);
            end
        end
    end
`endif

    // ---- the answer for Stage 3: seat, packet (port, direction, size, master id, index, data) ----
    assign response_valid = (hs == H_RESP);
    assign response_seat  = head_q[Q_SEAT +: SEAT_W];
    assign response_packet   = {head_q[Q_PORT +: PORT_W], h_rd_wr, h_size, head_q[Q_MID +: MASTER_ID_BITS], head_q[Q_IDX +: IDX_BITS],
                         (h_rd_wr == OP_WRITE) ? {WRITE_DATA_WIDTH{1'b0}} : load_data};

endmodule

`default_nettype wire
