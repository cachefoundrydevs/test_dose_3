// ============================================================================
// l1d_snoop_ctrl.v  -  snoop control unit (new in v2; ONE for the whole cache)
// ============================================================================
// IN SIMPLE WORDS
//   SNOOP = another core (or the L2) asks this cache about a line: "do you have line X? (read: tell me / give me the
//   dirty data; invalidate: throw your copy away)". This block is the reception desk for those questions:
//
//     snoop request in  ->  look at the bank bits of the address  ->  hand it to that bank's snoop buffer
//     the banks answer  ->  one small FIFO per bank  ->  round robin over the non-empty FIFOs  ->  ONE output register
//                       ->  snoop_response_* pins
//
// REQUEST SIDE
//   A snoop is taken in a cycle with snoop_request_valid = 1 AND snoop_request_ready = 1. snoop_request_ready is the
//   addressed bank's "snoop buffer not full", so a full bank holds the port (head-of-line blocking, accepted in v2).
//
// RESPONSE SIDE (one answer per snoop)
//   Answers of one bank leave in the order the snoops came; answers of different banks may leave in any order, so every
//   answer carries its line address (snoop_response_addr). snoop_response_dirty = 1: snoop_response_data is the line and the
//   receiver must write it to memory (the cache has given up the dirty copy). snoop_response_present = 1: the cache held
//   the line (in the arrays or the victim cache) when the snoop was served.
//   A bank pushes into its FIFO only when its Stage 0 reserved a place (the snoop credit, bank_snoop_response_fifo_free), so
//   a FIFO never overflows. The output register holds its answer until snoop_response_ready = 1.
//
// HAS_SNOOP = 0 ties the request port off (snoop_request_ready = 0, nothing reaches the banks).
//
// FOUND BY READING (for the user to decide): SNP_RESP_DEPTH = 1 is broken (the FIFO depth is forced to 2 but the free-count width
// comes from the parameter); the read multiplexer uses the binary sel_idx (TIMING OPT SC-2: now one-hot AND-OR); snoop_response_ready drives
// about 580 enables. TIMING OPT (session 45): SC-1 POP_LAST on the answer FIFOs, SC-2 one-hot answer select; same pins in every cycle.
//
// PORTS (new name, old name, meaning)
//   snoop_request_valid                  (was snp_valid)                a snoop request from outside
//   snoop_request_ready                  (was snp_ready)                the addressed bank can take it
//   snoop_request_addr                   (was snp_addr)                 any address inside the line
//   snoop_request_invalidate             (was snp_inv)                  1 = invalidate (another core writes), 0 = read
//   bank_snoop_valid                     (was bk_snp_valid)             one bit per bank: the snoop is for this bank
//   bank_snoop_line_addr                 (was bk_snp_line)              line address of the snoop
//   bank_snoop_invalidate                (was bk_snp_inv)               1 = invalidate, 0 = read
//   bank_snoop_ready                     (was bk_snp_ready)             one bit per bank: its snoop buffer has room
//   bank_snoop_response_valid            (was bk_snpr_valid)            one bit per bank: it pushes a snoop answer
//   bank_snoop_response_packet           (was bk_snpr_pkt)              the banks' answer packets side by side
//   bank_snoop_response_fifo_free        (was bk_snpr_free)             free places in each bank's answer FIFO
//   snoop_response_valid                 (was snpr_valid)               the output register holds a snoop answer
//   snoop_response_ready                 (was snpr_ready)               the receiver takes the answer
//   snoop_response_addr                  (was snpr_addr)                line address of the answered snoop
//   snoop_response_present               (was snpr_present)             the cache held the line
//   snoop_response_dirty                 (was snpr_dirty)               snoop_response_data is a dirty line: write it to memory
//   snoop_response_data                  (was snpr_data)                the line
//
// VERILOG WORDS USED IN THIS FILE
//   output reg snoop_response_valid   a flip-flop: the output register's valid bit
//   bank_snoop_response_packet[b*SNPR_W +: SNPR_W]   the answer packet of bank b
//   en = (HAS_SNOOP != 0)             a constant 1 or 0 decided when the design is built
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

module l1d_snoop_ctrl #(`L1D_PARAMS) (
    clk, rst,
    snoop_request_valid, snoop_request_ready, snoop_request_addr, snoop_request_invalidate,
    bank_snoop_valid, bank_snoop_line_addr, bank_snoop_invalidate, bank_snoop_ready,
    bank_snoop_response_valid, bank_snoop_response_packet, bank_snoop_response_fifo_free,
    snoop_response_valid, snoop_response_ready, snoop_response_addr, snoop_response_present, snoop_response_dirty, snoop_response_data
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // NB = number of banks;  OFF0 = zero line offset (the answer address is {line address, 6 zero bits})
    localparam NB = NUM_BANKS;
    localparam [OFFSET_W-1:0] OFF0 = {OFFSET_W{1'b0}};

    input  wire                     clk;                  // clock: all registers change on its rising edge
    input  wire                     rst;                  // synchronous reset, active high (control state goes to its idle value)
    // snoop request port
    input  wire                     snoop_request_valid;  // a snoop request from outside
    output wire                     snoop_request_ready;  // the addressed bank can take it
    input  wire [ADDR_W-1:0]        snoop_request_addr;   // any address inside the line
    input  wire                     snoop_request_invalidate;  // 1 = invalidate (another core writes), 0 = read
    // to the banks' snoop buffers
    output wire [NB-1:0]            bank_snoop_valid;     // one bit per bank: the snoop is for this bank
    output wire [LINE_ADDR_W-1:0]   bank_snoop_line_addr; // line address of the snoop
    output wire                     bank_snoop_invalidate;  // 1 = invalidate, 0 = read
    input  wire [NB-1:0]            bank_snoop_ready;     // one bit per bank: its snoop buffer has room
    // from the banks
    input  wire [NB-1:0]            bank_snoop_response_valid;  // one bit per bank: it pushes a snoop answer
    input  wire [NB*SNPR_W-1:0]     bank_snoop_response_packet;  // the banks' answer packets side by side
    output wire [NB*SNPR_CNT_W-1:0] bank_snoop_response_fifo_free;  // free places in each bank's answer FIFO
    // snoop response port
    output reg                      snoop_response_valid; // the output register holds a snoop answer
    input  wire                     snoop_response_ready; // the receiver takes the answer
    output wire [ADDR_W-1:0]        snoop_response_addr;  // line address of the answered snoop
    output wire                     snoop_response_present;  // the cache held the line
    output wire                     snoop_response_dirty; // snoop_response_data is a dirty line: write it to memory
    output wire [LINE_W-1:0]        snoop_response_data;  // the line

    // ---------------- request routing ----------------
    // ---- request routing: the bank number is the bank field of the snoop address ----
    wire [BANK_W-1:0] r_bank = snoop_request_addr[BANK_LSB +: BANK_W];
    wire              en     = (HAS_SNOOP != 0);

    // only the addressed bank sees snoop_valid; the line address and the kind go to all banks unchanged
    genvar b;
    generate
        for (b = 0; b < NB; b = b + 1) begin : g_route
            assign bank_snoop_valid[b] = en && snoop_request_valid && (r_bank == b);
        end
    endgenerate
    // the request is "ready" when the addressed bank's snoop buffer has a free entry (and snoops are enabled)
    assign bank_snoop_line_addr = snoop_request_addr[OFFSET_W +: LINE_ADDR_W];
    assign bank_snoop_invalidate  = snoop_request_invalidate;
    assign snoop_request_ready   = en && bank_snoop_ready[r_bank];

    // ---------------- response merge ----------------
    // ---- response merge: one FIFO per bank (at least 2 places), then a round-robin choice into one output register ----
    localparam RD = (SNP_RESP_DEPTH < 2) ? 2 : SNP_RESP_DEPTH;

    // f_empty = FIFO is empty, f_dout = its oldest answer, f_pop = take it; load = the output register can take a new answer now
    wire [NB-1:0]        f_empty;
    wire [NB*SNPR_W-1:0] f_dout;
    wire [NB-1:0]        f_pop;
    wire                 load = !snoop_response_valid || snoop_response_ready;

    wire [NB-1:0]     sel_gnt;
    wire [BANK_W-1:0] sel_idx;
    wire              sel_any;

    // round robin over the non-empty FIFOs; the turn moves on when the output register really loads
    l1d_arb_rr #(.N(NB), .IDX_W(BANK_W)) u_arb (
        .clk          (clk),
        .rst          (rst),
        .request_bits (~f_empty),
        .grant_taken  (load),
        .grant_onehot (sel_gnt),
        .grant_index  (sel_idx),
        .any_request  (sel_any)
    );

    // one answer FIFO per bank: pushed by the bank, popped when the arbiter chose it and the output register loads;
    // its free count goes back to the bank as its credit
    generate
        for (b = 0; b < NB; b = b + 1) begin : g_fifo
            // TIMING OPT (SC-1, session 45): POP_LAST = 1 as in l1d_resp_merge (RM-1): the pop comes late (FIFO not empty -> arbiter grant ->
            // load && grant -> pop), so the level update keeps the pop out of its arithmetic. Same FIFO behaviour in every cycle.
            l1d_fifo #(.WIDTH(SNPR_W), .DEPTH(RD), .CNT_W(SNPR_CNT_W), .POP_LAST(1)) u_fifo (
                .clk        (clk),
                .rst        (rst),
                .push       (bank_snoop_response_valid[b]),
                .push_data  (bank_snoop_response_packet[b*SNPR_W +: SNPR_W]),
                .pop        (f_pop[b]),
                .front_data (f_dout[b*SNPR_W +: SNPR_W]),
                .empty      (f_empty[b]),
                .full       (),
                .used_slots (),
                .free_slots (bank_snoop_response_fifo_free[b*SNPR_CNT_W +: SNPR_CNT_W])
            );
            assign f_pop[b] = load && sel_gnt[b];
        end
    endgenerate

    // ---- the output register (valid bit + answer packet): loads the chosen FIFO's oldest answer; holds it until the receiver reads it ----
    // TIMING OPT (SC-2, session 45): the chosen FIFO's answer by AND-OR with the ONE-HOT grant sel_gnt instead of the binary index
    // (f_dout[sel_idx*SNPR_W +: SNPR_W] is a shift by a 2-bit number that fans out to every bit). The register loads only with sel_any,
    // and then sel_gnt is one-hot at sel_idx, so the loaded value is the same in every cycle.
    reg [SNPR_W-1:0] sel_pkt;                             // the answer of the granted FIFO, SNPR_W = 572 bits
    integer k;
    always @* begin
        sel_pkt = {SNPR_W{1'b0}};
        for (k = 0; k < NB; k = k + 1)
            sel_pkt = sel_pkt | ({SNPR_W{sel_gnt[k]}} & f_dout[k*SNPR_W +: SNPR_W]);
    end
    reg [SNPR_W-1:0] out_pkt;
    always @(posedge clk) begin
        if (rst) begin
            snoop_response_valid <= 1'b0;
            out_pkt    <= {SNPR_W{1'b0}};
        end else if (load) begin
            snoop_response_valid <= sel_any;
            if (sel_any)
                out_pkt <= sel_pkt;
        end
    end

    // unpack the answer packet: line address, present, dirty, line data
    assign snoop_response_addr    = {out_pkt[SNPR_LINE_LSB +: LINE_ADDR_W], OFF0};
    assign snoop_response_present = out_pkt[SNPR_PRES_BIT];
    assign snoop_response_dirty   = out_pkt[SNPR_DIRTY_BIT];
    assign snoop_response_data    = out_pkt[SNPR_DATA_LSB +: LINE_W];

// ---- simulation-only trace: snoops taken and answers given ----
`ifdef L1D_DEBUG
    // Debug trace (simulation only): snoops taken and answers given.
    always @(posedge clk) begin
        if (!rst) begin
            if (snoop_request_valid && snoop_request_ready)
                $display("[%0t] L1D SNPC snoop %s addr=%h -> bank %0d", $time,
                         snoop_request_invalidate ? "INV " : "READ", snoop_request_addr, r_bank);
            if (snoop_response_valid && snoop_response_ready)
                $display("[%0t] L1D SNPC answer addr=%h present=%b dirty=%b", $time,
                         snoop_response_addr, snoop_response_present, snoop_response_dirty);
        end
    end
`endif

endmodule

`default_nettype wire
