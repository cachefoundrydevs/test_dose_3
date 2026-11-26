// ==========================================================================
// l1d_pf_sms_front.v - SMS front end: pick one record per cycle, queue it
// ==========================================================================
// IN SIMPLE WORDS : The cache sends up to 4 training records and up to 4
//   eviction events per cycle. The learning core takes one event per
//   cycle, so this block picks ONE training record and ONE eviction
//   event each cycle (the first valid port, starting from a rotating
//   port so that no port is always last), shortens the PC to a hash,
//   and puts them into two small lossy FIFOs. Records that lose the
//   pick are dropped: a missed record only leaves a hole in a pattern.
// STAGE / CYCLE   : S-1 (before the core). FIFO input is registered.
// INSTANTIATED BY : l1d_pf_sms, instance u_front, 1 copy.
//
// PARAMETERS:
//   name              default unit     meaning
//   N_PORTS           4       ports    request ports (training records)
//   N_BANKS           4       banks    eviction ports
//   PC_W              32      bits     PC bits from the cache (assumed)
//   LINE_ADDR_W       50      bits     physical line address
//   PC_HASH_W         12      bits     PC hash kept in the tables
//   TRAIN_FIFO_DEPTH  4       records  training FIFO depth (power of 2)
//   EVT_FIFO_DEPTH    4       records  eviction FIFO depth (power of 2)
//
// PORTS (default widths):
//   clk, rst               in  1        clock, synchronous reset (high)
//   pf_enable              in  1        1 = on; low drops all records
//   tr_valid               in  4        training record per port
//   tr_no_train            in  4        AMO / LR / SC / NC: ignore
//   tr_pc                  in  4x32     PC of each access
//   tr_line_addr           in  4x50     line address of each access
//   ev_valid               in  4        eviction event per bank
//   ev_line_addr           in  4x50     evicted line address per bank
//   train_pop, evt_pop     in  1 each   core takes the FIFO head
//   train_head_valid       out 1        training record waiting
//   train_head_pc_hash     out 12       its PC hash
//   train_head_line_addr   out 50       its line address
//   evt_head_valid         out 1        eviction event waiting
//   evt_head_line_addr     out 50       its line address
//
// WORKED EXAMPLE : ports 1 and 3 valid in cycle 7, rotation = 2:
//   order seen is 2,3,0,1 -> port 3 is taken, port 1 is dropped.
//
// ASSUMPTIONS (V-9.2): A1 FIFO depths are powers of two. A2 N_PORTS and
//   N_BANKS >= 1. A3 the cache already marks unusable accesses with
//   tr_no_train (AMO, LR/SC, non-cacheable, wrong path).
// DEVIATION: README keeps all records in a FIFO and lets two lanes read
//   it. Here the pick is made before a one-lane core (TRAIN_LANES 1,
//   open item SM-13). Drops are the same kind (lossy).
// COMBINATIONAL OUTPUTS (V-3.11): FIFO heads (2-level mux).
// ==========================================================================
`ifndef L1D_PF_SMS_FRONT_V
`define L1D_PF_SMS_FRONT_V
`default_nettype none

module l1d_pf_sms_front #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter N_PORTS          = 4,  // default 4: training ports
    parameter N_BANKS          = 4,  // default 4: eviction ports
    parameter PC_W             = 32, // default 32 bits: PC from cache
    parameter LINE_ADDR_W      = 50, // default 50 bits: line address
    parameter PC_HASH_W        = 12, // default 12 bits: PC hash
    parameter TRAIN_FIFO_DEPTH = 4,  // default 4 records
    parameter EVT_FIFO_DEPTH   = 4   // default 4 records
) (
    // ---- clock and reset ----
    input  wire                           clk,                  // clock
    input  wire                           rst,                  // sync reset
    // ---- control ----
    input  wire                           pf_enable,            // 1 = on
    // ---- training records in (one per request port) ----
    input  wire [N_PORTS-1:0]             tr_valid,             // record
    input  wire [N_PORTS-1:0]             tr_no_train,          // ignore
    input  wire [N_PORTS*PC_W-1:0]        tr_pc,                // 4 x 32
    input  wire [N_PORTS*LINE_ADDR_W-1:0] tr_line_addr,         // 4 x 50
    // ---- eviction events in (one per bank) ----
    input  wire [N_BANKS-1:0]             ev_valid,             // event
    input  wire [N_BANKS*LINE_ADDR_W-1:0] ev_line_addr,         // 4 x 50
    // ---- core takes a record ----
    input  wire                           train_pop,            // pop head
    input  wire                           evt_pop,              // pop head
    // ---- records out (to the core) ----
    output wire                           train_head_valid,     // waiting
    output wire [PC_HASH_W-1:0]           train_head_pc_hash,   // 12 bits
    output wire [LINE_ADDR_W-1:0]         train_head_line_addr, // 50 bits
    output wire                           evt_head_valid,       // waiting
    output wire [LINE_ADDR_W-1:0]         evt_head_line_addr    // 50 bits
);

    // ---- local constants (value at the default) ----
    localparam TRAIN_W = PC_HASH_W + LINE_ADDR_W;   // 62 bits record
    localparam PORT_W  = $clog2(N_PORTS);           // 2 bits rotation
    localparam BANK_W  = $clog2(N_BANKS);           // 2 bits rotation
    localparam [PORT_W-1:0] PORT_LAST = N_PORTS - 1; // 3
    localparam [BANK_W-1:0] BANK_LAST = N_BANKS - 1; // 3

    reg  [PORT_W-1:0]        train_rot_q;     // 2: first port tried
    reg  [BANK_W-1:0]        evt_rot_q;       // 2: first bank tried
    reg  [N_PORTS-1:0]       train_ok;        // 4: record is usable
    reg                      train_found;     // 1: a port was chosen
    reg  [PC_W-1:0]          train_sel_pc;    // 32: its PC
    reg  [LINE_ADDR_W-1:0]   train_sel_line;  // 50: its line address
    reg                      evt_found;       // 1: a bank was chosen
    reg  [LINE_ADDR_W-1:0]   evt_sel_line;    // 50: its line address
    wire [PC_HASH_W-1:0]     train_sel_hash;  // 12: hash of its PC
    wire [TRAIN_W-1:0]       train_push_word; // 62: {hash, line}
    wire [TRAIN_W-1:0]       train_head_word; // 62: {hash, line}

    // ---- hash of the PC (XOR fold, 32 -> 12 bits) ----
    function [PC_HASH_W-1:0] fold_pc;
        input [PC_W-1:0] pc_value;
        integer bit_idx;
        reg [PC_HASH_W-1:0] folded;
        begin
            folded = {PC_HASH_W{1'b0}};
            for (bit_idx = 0; bit_idx < PC_W; bit_idx = bit_idx + 1) begin
                folded[bit_idx % PC_HASH_W] = folded[bit_idx % PC_HASH_W]
                                              ^ pc_value[bit_idx];
            end // for bit_idx
            fold_pc = folded;
        end
    endfunction

    // ---- pick one training record: first usable port from the rotation
    always @* begin : train_pick_proc
        integer rot_idx;  // rotated port number
        integer scan_idx; // walks the ports
        train_ok       = tr_valid & ~tr_no_train & {N_PORTS{pf_enable}};
        train_found    = 1'b0;
        train_sel_pc   = {PC_W{1'b0}};
        train_sel_line = {LINE_ADDR_W{1'b0}};
        for (scan_idx = 0; scan_idx < N_PORTS; scan_idx = scan_idx + 1) begin
            rot_idx = (scan_idx + train_rot_q) % N_PORTS;
            if (train_ok[rot_idx] && !train_found) begin
                train_found    = 1'b1;
                train_sel_pc   = tr_pc[rot_idx*PC_W +: PC_W];
                train_sel_line = tr_line_addr[rot_idx*LINE_ADDR_W +: LINE_ADDR_W];
            end
        end // for scan_idx
    end // always train_pick_proc

    // ---- pick one eviction event: first valid bank from the rotation
    always @* begin : evt_pick_proc
        integer rot_idx;  // rotated bank number
        integer scan_idx; // walks the banks
        evt_found    = 1'b0;
        evt_sel_line = {LINE_ADDR_W{1'b0}};
        for (scan_idx = 0; scan_idx < N_BANKS; scan_idx = scan_idx + 1) begin
            rot_idx = (scan_idx + evt_rot_q) % N_BANKS;
            if (ev_valid[rot_idx] && pf_enable && !evt_found) begin
                evt_found    = 1'b1;
                evt_sel_line = ev_line_addr[rot_idx*LINE_ADDR_W +: LINE_ADDR_W];
            end
        end // for scan_idx
    end // always evt_pick_proc

    // ---- rotation counters: one step per cycle, wrap at the port count
    always @(posedge clk) begin : rotation_proc
        if (rst) begin
            train_rot_q <= {PORT_W{1'b0}};
            evt_rot_q   <= {BANK_W{1'b0}};
        end
        else begin
            train_rot_q <= (train_rot_q == PORT_LAST) ? {PORT_W{1'b0}}
                                                      : train_rot_q + 1'b1;
            evt_rot_q   <= (evt_rot_q == BANK_LAST) ? {BANK_W{1'b0}}
                                                    : evt_rot_q + 1'b1;
        end // else (rst)
    end // always rotation_proc

    assign train_sel_hash  = fold_pc(train_sel_pc);
    assign train_push_word = {train_sel_hash, train_sel_line};

    // ---- the two FIFOs ----
    l1d_pf_sms_fifo #(
        .WIDTH (TRAIN_W),
        .DEPTH (TRAIN_FIFO_DEPTH)
    ) u_train_fifo (
        .clk        (clk),
        .rst        (rst),
        .push_valid (train_found),
        .push_data  (train_push_word),
        .pop_en     (train_pop),
        .head_valid (train_head_valid),
        .head_data  (train_head_word)
    );

    assign train_head_pc_hash   = train_head_word[TRAIN_W-1:LINE_ADDR_W];
    assign train_head_line_addr = train_head_word[LINE_ADDR_W-1:0];

    l1d_pf_sms_fifo #(
        .WIDTH (LINE_ADDR_W),
        .DEPTH (EVT_FIFO_DEPTH)
    ) u_evict_fifo (
        .clk        (clk),
        .rst        (rst),
        .push_valid (evt_found),
        .push_data  (evt_sel_line),
        .pop_en     (evt_pop),
        .head_valid (evt_head_valid),
        .head_data  (evt_head_line_addr)
    );

endmodule

`default_nettype wire
`endif
