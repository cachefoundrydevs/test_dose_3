// ============================================================================
// l1d_bank_s3.v  -  bank stage S3: the AMO line merge, the response arbiter (pipeline / MSHR / NC), the memory channels
// ============================================================================
// IN SIMPLE WORDS
//   Three small jobs at the exit of the bank:
//   1. S3 of an AMO hit: the AMO unit's new value (computed in S2) is put at its byte position into the old line, and the
//      merged line goes to the data SRAM write port (l1d_bank_r) in this cycle.
//   2. The response arbiter: in each cycle at most one answer leaves the bank into its port's FIFO in l1d_resp_merge. The
//      pipeline's own answer (decided in S2, "s3_load") goes first; otherwise an MSHR answer, then a non-cacheable answer,
//      but only when every port's FIFO has a place beyond the ones promised to R and S1 (miss_ok). The seat of the answer
//      that leaves is freed (done_*). NOTE: this push happens in the S2 cycle of the pipeline's answer (no S3 register for
//      answers); the name "s3_load" is kept from cache_v2.
//   3. The memory channels: refills of the MSHR go out before non-cacheable reads; victim-cache writebacks before
//      non-cacheable writes.
//
//   s3_* registers (AMO) ---> merge ---> s3_merged (to the data SRAM write port)
//   s3_load / s2_pkt (S2), MSHR answer, NC answer ---> one push ---> lsu_response_* , done_*
//   MSHR read, NC read ---> axi_read_req_*;   VC writeback, NC write ---> axi_write_req_*
//
// STAGE / CYCLE
//   No registers. The AMO merge works on the S3 registers (cycle after S2); the response arbiter and the memory channels
//   are combinational in the cycle of their inputs.
//
// INSTANTIATED BY
//   l1d_bank, instance u_s3 (one per bank).
//
// PARAMETERS
//   `L1D_PARAMS   all cache parameters (l1d_defines.vh); used here: LINE_BYTES (64), WRITE_DATA_WIDTH (64),
//                 AXI_DATA_W (128) and the widths of l1d_fields.vh
//   BANK_ID       default 0: number of this bank (debug messages only)
//
// PORTS (name, direction, width at the defaults, meaning)
//   clk, rst                           in   1      clock and reset (debug counters only: this module has no registers)
//   s3_old, s3_be, s3_off, s3_new      in   512/64/6/64  the S3 registers of an AMO hit (from l1d_bank_s2)
//   s3_merged                          out  512    the old line with the AMO result in place
//   s3_load, s2_pkt, s2_seat           in   1/74/3 the pipeline answers now: its packet and seat
//   m_resp_valid/_pkt/_seat            in   1/74/3 an MSHR answer waits;   m_resp_ack out 1: it is taken
//   n_resp_valid/_pkt/_seat            in   1/74/3 a non-cacheable answer waits;   n_resp_ack out 1: it is taken
//   miss_ok                            in   1      an MSHR / NC answer may be pushed (credit, from l1d_bank_s0)
//   lsu_response_fifo_free_N           in   3      free places of port N's FIFO (debug messages only)
//   lsu_response_valid/_packet         out  1/74   the push into this bank's response FIFO
//   done_valid, done_seat              out  1/3    a response left: free this seat
//   m_rd_* / n_rd_* (in), m_rd_ready / n_rd_ready (out)   the MSHR and NC read requests
//   vc_wr_* / n_wr_* (in), vc_wr_ready / n_wr_ready (out) the victim-cache and NC write requests
//   axi_read_req_* , axi_write_req_*   out / in    the memory channels of the bank (to l1d_l2_port)
//
// WORKED EXAMPLE (the pipeline answers a hit while an MSHR answer waits)
//   s3_load = 1, m_resp_valid = 1, miss_ok = 1 -> lsu_response_packet = s2_pkt, m_take = 0 (the MSHR answer waits one
//   more cycle), done_seat = s2_seat. Next cycle, with no pipeline answer: m_take = 1, packet = m_resp_pkt.
//
// VERILOG WORDS USED IN THIS FILE
//   {a, b}               glue a and b side by side;  {N{1'b0}} = N zero bits
//   x << {off, 3'b000}   shift x left by off bytes (off * 8 bits)
// ============================================================================
`ifndef L1D_BANK_S3_V
`define L1D_BANK_S3_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_bank_s3 #(
    `L1D_PARAMS,                                       // all cache parameters (l1d_defines.vh)
    parameter BANK_ID = 0                              // default 0: number of this bank (debug messages only)
) (
    clk, rst,
    s3_old, s3_be, s3_off, s3_new, s3_merged,
    s3_load, s2_pkt, s2_seat,
    m_resp_valid, m_resp_pkt, m_resp_seat, m_resp_ack,
    n_resp_valid, n_resp_pkt, n_resp_seat, n_resp_ack,
    miss_ok, lsu_response_fifo_free_0, lsu_response_fifo_free_1, lsu_response_fifo_free_2, lsu_response_fifo_free_3,
    lsu_response_valid, lsu_response_packet, done_valid, done_seat,
    m_rd_valid, m_rd_ready, m_rd_line, m_rd_sub, m_rd_unique,
    n_rd_valid, n_rd_ready, n_rd_addr, n_rd_size,
    vc_wr_valid, vc_wr_ready, vc_wr_line, vc_wr_sub, vc_wr_data,
    n_wr_valid, n_wr_ready, n_wr_addr, n_wr_size, n_wr_data, n_wr_strb,
    axi_read_req_valid, axi_read_req_ready, axi_read_req_addr, axi_read_req_burst_len, axi_read_req_size,
    axi_read_req_sub_id, axi_read_req_non_cacheable, axi_read_req_unique,
    axi_write_req_valid, axi_write_req_ready, axi_write_req_addr, axi_write_req_burst_len, axi_write_req_size,
    axi_write_req_sub_id, axi_write_req_non_cacheable, axi_write_req_data, axi_write_req_strobe
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // ---- local constants ----
    localparam [7:0]          LEN_C  = AXI_LEN;                  // AXI burst length of a whole line (beats minus one), 3 at 128-bit beats
    localparam [2:0]          SIZE_C = AXI_SIZE;                 // AXI size of a full beat, 4 (16 bytes) at 128-bit beats
    localparam [OFFSET_W-1:0] OFF0   = {OFFSET_W{1'b0}};         // zero line offset appended to a line address, OFFSET_W = 6 bits

    // ======================================================================
    // Ports
    // ======================================================================
    input  wire                        clk;                      // clock (debug counters only)
    input  wire                        rst;                      // synchronous reset, active high (debug counters only)
    // ---- S3 of an AMO hit (registers of l1d_bank_s2) ----
    input  wire [LINE_W-1:0]           s3_old;                   // the old line, LINE_W = 512 bits
    input  wire [LINE_BYTES-1:0]       s3_be;                    // bytes of the AMO word, LINE_BYTES = 64 bits
    input  wire [OFFSET_W-1:0]         s3_off;                   // byte offset of the AMO word, 6 bits
    input  wire [WRITE_DATA_WIDTH-1:0] s3_new;                   // the AMO unit's result, aligned to bit 0, 64 bits
    output reg  [LINE_W-1:0]           s3_merged;                // the old line with the result in place, 512 bits
    // ---- answers that want to leave the bank ----
    input  wire                        s3_load;                  // the pipeline answers now (decided in S2)
    input  wire [RESP_W-1:0]           s2_pkt;                   // its response packet, RESP_W = 74 bits
    input  wire [SEAT_W-1:0]           s2_seat;                  // its seat, SEAT_W = 3 bits
    input  wire                        m_resp_valid;             // an MSHR answer waits
    input  wire [RESP_W-1:0]           m_resp_pkt;               // its packet, 74 bits
    input  wire [SEAT_W-1:0]           m_resp_seat;              // its seat, 3 bits
    output wire                        m_resp_ack;               // the MSHR answer is taken this cycle
    input  wire                        n_resp_valid;             // a non-cacheable answer waits
    input  wire [RESP_W-1:0]           n_resp_pkt;               // its packet, 74 bits
    input  wire [SEAT_W-1:0]           n_resp_seat;              // its seat, 3 bits
    output wire                        n_resp_ack;               // the NC answer is taken this cycle
    input  wire                        miss_ok;                  // an MSHR / NC answer may be pushed (credit)
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_0; // free places of port 0 FIFO (debug messages only), 3 bits
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_1; // ... port 1, 3 bits
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_2; // ... port 2, 3 bits
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_3; // ... port 3, 3 bits
    wire [4*RCNT_W-1:0] resp_free_all = {lsu_response_fifo_free_3, lsu_response_fifo_free_2, lsu_response_fifo_free_1, lsu_response_fifo_free_0};  // debug print only, ports 3..0
    // ---- the push into this bank's response FIFO ----
    output wire                        lsu_response_valid;       // a response leaves the bank this cycle
    output wire [RESP_W-1:0]           lsu_response_packet;      // its packet, 74 bits
    output reg                         done_valid;               // a response left: its seat is freed
    output reg  [SEAT_W-1:0]           done_seat;                // that seat, 3 bits
    // ---- read requests of the MSHR and the NC queue ----
    input  wire                        m_rd_valid;               // the MSHR offers a line read
    output wire                        m_rd_ready;               // ... it is taken
    input  wire [LINE_ADDR_W-1:0]      m_rd_line;                // ... line address, LINE_ADDR_W = 58 bits
    input  wire [MSHR_ID_W-1:0]        m_rd_sub;                 // ... MSHR entry (part of the AXI id), 3 bits
    input  wire                        m_rd_unique;              // ... the line is wanted unique
    input  wire                        n_rd_valid;               // the NC queue offers a read
    output wire                        n_rd_ready;               // ... it is taken
    input  wire [ADDR_W-1:0]           n_rd_addr;                // ... byte address, ADDR_W = 64 bits
    input  wire [2:0]                  n_rd_size;                // ... AXI size
    // ---- write requests of the victim cache and the NC queue ----
    input  wire                        vc_wr_valid;              // the victim cache offers a line writeback
    output wire                        vc_wr_ready;              // ... it is taken
    input  wire [LINE_ADDR_W-1:0]      vc_wr_line;               // ... line address, 58 bits
    input  wire [VC_ID_W-1:0]          vc_wr_sub;                // ... slot number (part of the AXI id), VC_ID_W = 3 bits
    input  wire [LINE_W-1:0]           vc_wr_data;               // ... the line, 512 bits
    input  wire                        n_wr_valid;               // the NC queue offers a write
    output wire                        n_wr_ready;               // ... it is taken
    input  wire [ADDR_W-1:0]           n_wr_addr;                // ... byte address, 64 bits
    input  wire [2:0]                  n_wr_size;                // ... AXI size
    input  wire [AXI_DATA_W-1:0]       n_wr_data;                // ... one beat, AXI_DATA_W = 128 bits
    input  wire [AXI_STRB_W-1:0]       n_wr_strb;                // ... its byte strobes, AXI_STRB_W = 16 bits
    // ---- memory channels of the bank (to l1d_l2_port) ----
    output wire                        axi_read_req_valid;       // the bank offers a read request
    input  wire                        axi_read_req_ready;       // the L2 port took it
    output wire [AXI_ADDR_W-1:0]       axi_read_req_addr;        // read address, AXI_ADDR_W = 64 bits
    output wire [7:0]                  axi_read_req_burst_len;   // beats minus one
    output wire [2:0]                  axi_read_req_size;        // AXI size
    output wire [AXI_SUBID_W-1:0]      axi_read_req_sub_id;      // MSHR entry (0 for non-cacheable), AXI_SUBID_W = 3 bits
    output wire                        axi_read_req_non_cacheable;  // 1 = non-cacheable access
    output wire                        axi_read_req_unique;      // the line is wanted unique
    output wire                        axi_write_req_valid;      // the bank offers a write request
    input  wire                        axi_write_req_ready;      // the L2 port took it
    output wire [AXI_ADDR_W-1:0]       axi_write_req_addr;       // write address, 64 bits
    output wire [7:0]                  axi_write_req_burst_len;  // beats minus one
    output wire [2:0]                  axi_write_req_size;       // AXI size
    output wire [AXI_SUBID_W-1:0]      axi_write_req_sub_id;     // victim-cache slot (0 for non-cacheable), 3 bits
    output wire                        axi_write_req_non_cacheable; // 1 = non-cacheable access
    output wire [LINE_W-1:0]           axi_write_req_data;       // whole line (beat k at [k*AXI_DATA_W +: AXI_DATA_W]), 512 bits
    output wire [AXI_STRB_W-1:0]       axi_write_req_strobe;     // strobes of a single-beat (non-cacheable) write, 16 bits

    // ======================================================================
    // S3 of an AMO hit: the new value merged into the old line
    // ======================================================================
    // S3: the new value at its byte position, merged byte by byte into the old line
    wire [LINE_W-1:0] s3_din = {{(LINE_W-WRITE_DATA_WIDTH){1'b0}}, s3_new} << {s3_off, 3'b000};
    integer ab;
    always @* begin
        for (ab = 0; ab < LINE_BYTES; ab = ab + 1)
            s3_merged[ab*8 +: 8] = s3_be[ab] ? s3_din[ab*8 +: 8] : s3_old[ab*8 +: 8];
    end

    // ======================================================================
    // Response arbiter (the pipeline answers in S2, then MSHR, then NC)
    // ======================================================================
    // ---- The pipeline's own answer (s2_push, decided in S2) goes first; otherwise an MSHR response, then an NC response, when a place
    //      in the response FIFO is free beyond the ones promised to R / S1 (miss_ok). The push goes into this bank's FIFO in
    //      l1d_resp_merge in this cycle; when that FIFO is empty the merger takes it straight into its output register. ----
    wire s2_push = s3_load;                               // (s3_load keeps its cache_v2 name: "this request answers now")
    wire m_take = !s2_push && m_resp_valid && miss_ok;
    wire n_take = !s2_push && !m_resp_valid && n_resp_valid && miss_ok;
    wire miss_push = m_take || n_take;                    // an MSHR or NC answer is pushed this cycle
    assign m_resp_ack = m_take;
    assign n_resp_ack = n_take;

    // the response push of this bank: the S2 answer, or an MSHR / NC response
    assign lsu_response_valid  = s2_push || miss_push;
    assign lsu_response_packet = s2_push ? s2_pkt : (m_take ? m_resp_pkt : n_resp_pkt);

    // the seat to free is that of the response that leaves: S2's, or the MSHR's, or the NC's
    always @* begin
        done_valid = lsu_response_valid;
        done_seat  = s2_push ? s2_seat : (m_take ? m_resp_seat : n_resp_seat);
    end

    // ======================================================================
    // Memory channels: MSHR / NC reads, victim-cache / NC writes
    // ======================================================================
    // ---- MEMORY CHANNELS. READ: cacheable line refills (MSHR) first, then the NC access. ----
    wire [ADDR_W-1:0] m_rd_addr = {m_rd_line, OFF0};
    wire [ADDR_W-1:0] rd_addr_w = m_rd_valid ? m_rd_addr : n_rd_addr;
    assign axi_read_req_valid   = m_rd_valid || n_rd_valid;
    assign m_rd_ready = axi_read_req_ready && m_rd_valid;
    assign n_rd_ready = axi_read_req_ready && !m_rd_valid;
    assign axi_read_req_addr    = rd_addr_w;                        // zero-extended / truncated to AXI_ADDR_W
    assign axi_read_req_burst_len     = m_rd_valid ? LEN_C : 8'd0;
    assign axi_read_req_size    = m_rd_valid ? SIZE_C : n_rd_size;
    assign axi_read_req_sub_id     = m_rd_valid ? m_rd_sub : {AXI_SUBID_W{1'b0}};
    assign axi_read_req_non_cacheable      = !m_rd_valid;
    assign axi_read_req_unique  = m_rd_valid && m_rd_unique;

    // write: victim-cache writebacks first, then the NC store
    // ---- WRITE: victim-cache writebacks first, then the NC store. A line is written as a burst (LEN_C), an NC store as one beat. ----
    wire [ADDR_W-1:0] vc_wr_addr = {vc_wr_line, OFF0};
    wire [ADDR_W-1:0] wr_addr_w  = vc_wr_valid ? vc_wr_addr : n_wr_addr;
    wire [LINE_W-1:0] n_wr_line  = n_wr_data;              // NC beat in the low bits
    assign axi_write_req_valid    = vc_wr_valid || n_wr_valid;
    assign vc_wr_ready = axi_write_req_ready && vc_wr_valid;
    assign n_wr_ready  = axi_write_req_ready && !vc_wr_valid;
    assign axi_write_req_addr     = wr_addr_w;
    assign axi_write_req_burst_len      = vc_wr_valid ? LEN_C : 8'd0;
    assign axi_write_req_size     = vc_wr_valid ? SIZE_C : n_wr_size;
    assign axi_write_req_sub_id      = vc_wr_valid ? vc_wr_sub : {AXI_SUBID_W{1'b0}};
    assign axi_write_req_non_cacheable       = !vc_wr_valid;
    assign axi_write_req_data     = vc_wr_valid ? vc_wr_data : n_wr_line;
    assign axi_write_req_strobe     = vc_wr_valid ? {AXI_STRB_W{1'b1}} : n_wr_strb;

// ---- simulation-only trace (compiled only when L1D_DEBUG is defined): responses that leave and answers that cannot leave ----
`ifdef L1D_DEBUG
    integer dbg_mrw, dbg_nrw;                             // cycles an MSHR / NC answer has waited
    initial begin dbg_mrw = 0; dbg_nrw = 0; end

    always @(posedge clk) begin
        if (!rst) begin
            if (lsu_response_valid)
                $display("[%0t] L1D B%0d S3 RESP from %s  seat=%0d idx=%0d %s data=%h (resp_free=%0d)",
                         $time, BANK_ID, s2_push ? "pipeline" : (m_take ? "MSHR" : "NC"),
                         done_seat, lsu_response_packet[RESP_IDX_LSB +: IDX_BITS], (lsu_response_packet[RESP_RDWR_BIT] ? "ST-ack" : "LD"),
                         lsu_response_packet[RESP_DATA_LSB +: WRITE_DATA_WIDTH], resp_free_all);
            if (m_resp_valid && !m_take) begin
                dbg_mrw = dbg_mrw + 1;
                if (dbg_mrw % `L1D_DBG_WD == 0)
                    $display("[%0t] L1D B%0d S3 STUCK MSHR response waits %0d cycles: s3_busy=%b miss_ok=%b resp_free=%0d",
                             $time, BANK_ID, dbg_mrw, s2_push, miss_ok, resp_free_all);
            end else dbg_mrw = 0;
            if (n_resp_valid && !n_take) begin
                dbg_nrw = dbg_nrw + 1;
                if (dbg_nrw % `L1D_DBG_WD == 0)
                    $display("[%0t] L1D B%0d S3 STUCK NC response waits %0d cycles: s3_busy=%b mshr_resp=%b miss_ok=%b resp_free=%0d",
                             $time, BANK_ID, dbg_nrw, s2_push, m_resp_valid, miss_ok, resp_free_all);
            end else dbg_nrw = 0;
        end
    end
`endif

endmodule

`default_nettype wire
`endif // L1D_BANK_S3_V
