// ============================================================================
// l1d_resp_merge.v  -  response merger (four banks -> one LSU response port)
// ============================================================================
// IN SIMPLE WORDS
//   Each bank can finish one request per cycle, but the CPU has ONE response port. So every bank
//   puts its response into its own little FIFO (a queue). A round-robin arbiter (l1d_arb_rr) picks
//   one non-empty FIFO and moves its oldest response into a single OUTPUT REGISTER, which drives the
//   lsu_response_* pins of l1d_top. The register holds the response until the CPU says
//   lsu_read_enable = 1 ("I took it"). Only then can the next response be loaded.
//
//   (v1/v2 only: with one LSU port per bank this block disappears.)
//
// THE CREDIT RULE (why a bank never overflows its FIFO)
//   bank_response_fifo_free tells each bank how many places its FIFO still has. A bank grants a new
//   CPU request in its Stage 0 only when a place for the response is reserved, so a bank never
//   pushes into a full FIFO. (The check is in l1d_bank, signals miss_ok / lsu_credit.)
//
// TIMING (cache_v2_bank_pipe)
//   A response pushed into an EMPTY FIFO can be on the pins ONE cycle after the push (bypass: it goes straight from the
//   bank into the output register). Otherwise it waits in its FIFO for its round-robin turn, as before.
//   Example: banks 1 and 3 push in the same cycle t into empty FIFOs. At t+1 one of them is on the pins, at t+2 the other.
//
// PORTS (new name, old name, meaning)
//   bank_response_valid                  (was bank_resp_valid)          one bit per bank: the bank pushes a response this cycle
//   bank_response_packet                 (was bank_resp)                the banks' response packets side by side
//   bank_response_fifo_free              (was bank_free)                free places in each bank's response FIFO (the bank's credit)
//   bank_response_fifo_free_next         (cache_v6)                     the same after this cycle's push / pop (l1d_bank_s0 credit_q)
//   merged_response_valid                (was out_valid)                the output register holds a response for the LSU
//   merged_response_packet               (was out_resp)                 the response packet shown to the LSU
//   lsu_read_enable                      (was read_enable)              the LSU takes the current response this cycle
//
// OLD NAMES OF THE OUTPUT PINS
//   merged_response_valid / merged_response_packet become, in l1d_top, the pins that used to be called
//   final_r_cache_hit_from_scheduler, final_r_hit_rd_wr_from_scheduler, final_r_mem_size_from_scheduler,
//   final_r_master_id_from_scheduler, final_r_idx_from_scheduler and final_r_data_from_scheduler
//   (now lsu_response_valid, ..._is_write_ack, ..._size, ..._master_id, ..._rob_index, ..._data).
//
// THE VPU PORT (cache_v5 C-20)
//   The merger of the VPU port is the same block with EXTRA_W = LINE_W: each stored answer is {line data, response packet},
//   so its FIFOs and its output register are 586 bits wide; the scalar ports keep 74 bits (EXTRA_W = 0).
//
// VERILOG WORDS USED IN THIS FILE
//   bank_response_packet[b*RESP_W +: RESP_W]   the response of bank b: the RESP_W bits starting at bit b*RESP_W
//   output reg merged_response_valid           a register (flip-flop): it keeps its value until the next load
// TIMING OPT: RM-1 POP_LAST FIFOs (session 42); RM-2 one-hot response select (session 45). Same pins in every cycle.
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

module l1d_resp_merge #(
    `L1D_PARAMS,                                     // all cache parameters (l1d_defines.vh)
    parameter EXTRA_W = 0                            // default 0: bits added on top of each response packet (C-20: LINE_W = 512 for the VPU port)
) (
    clk, rst,
    bank_response_valid, bank_response_packet, bank_response_fifo_free, bank_response_fifo_free_next,
    merged_response_valid, merged_response_packet, lsu_read_enable
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    localparam PKT_W = RESP_W + EXTRA_W;             // stored packet width: RESP_W = 74 bits (scalar port), 74 + 512 = 586 bits (VPU port)

    input  wire                       clk;                // clock: all registers change on its rising edge
    input  wire                       rst;                // synchronous reset, active high (control state goes to its idle value)
    input  wire [NUM_BANKS-1:0]       bank_response_valid;  // one bit per bank: the bank pushes a response this cycle
    input  wire [NUM_BANKS*PKT_W-1:0] bank_response_packet;  // the banks' response packets side by side, 4*74 = 296 bits (VPU: 4*586 = 2344)
    output wire [NUM_BANKS*RCNT_W-1:0] bank_response_fifo_free;  // free places in each bank's response FIFO (the bank's credit), 4*3 = 12 bits
    output wire [NUM_BANKS*RCNT_W-1:0] bank_response_fifo_free_next;  // cache_v6: the same after this cycle's push / pop (the credit of the next issue), 12 bits
    output reg                        merged_response_valid;  // the output register holds a response for the LSU
    output reg  [PKT_W-1:0]           merged_response_packet;  // the response packet shown to the LSU, 74 bits (VPU: 586)
    input  wire                       lsu_read_enable;    // the LSU takes the current response this cycle

    // signals of the per-bank FIFOs: f_empty = FIFO is empty, f_dout = its oldest response, f_pop = take it out
    wire [NUM_BANKS-1:0]        f_empty;
    wire [NUM_BANKS*PKT_W-1:0] f_dout;
    wire [NUM_BANKS-1:0]        f_pop;
    wire [NUM_BANKS-1:0]        head_v;               // cache_v2_bank_pipe: the bank has an answer at its head (stored, or pushed now into an empty FIFO)
    wire [NUM_BANKS*PKT_W-1:0] head_d;               // that answer
    wire [NUM_BANKS-1:0]        byp;                  // the answer pushed now is taken directly (FIFO empty, granted, register loads)

    // The output register can take a new response when it is empty or when the
    // LSU is reading the current one this cycle.
    // load = the output register can take a new response now: it is empty, or the CPU is reading the current one this cycle
    wire load = !merged_response_valid || lsu_read_enable;

    // the arbiter's answer: sel_gnt = one-hot choice, sel_idx = its number, sel_any = at least one FIFO has a response
    wire [NUM_BANKS-1:0] sel_gnt;
    wire [BANK_W-1:0]    sel_idx;
    wire                 sel_any;

    // round-robin over the FIFOs that are not empty; the turn moves on only when the output register really loads (advance = load)
    l1d_arb_rr #(.N(NUM_BANKS), .IDX_W(BANK_W)) u_arb (
        .clk          (clk),
        .rst          (rst),
        .request_bits (head_v),
        .grant_taken  (load),
        .grant_onehot (sel_gnt),
        .grant_index  (sel_idx),
        .any_request  (sel_any)
    );

    // one response FIFO per bank; a FIFO is popped when the arbiter chose it and the output register loads
    genvar b;
    generate
        for (b = 0; b < NUM_BANKS; b = b + 1) begin : g_fifo
            // TIMING OPT (RM-1, session 42): POP_LAST = 1, because the pop of these FIFOs comes late (FIFO not empty -> arbiter grant ->
            // output register loads -> pop); the level update then keeps the pop out of its arithmetic. Same FIFO behaviour in every cycle.
            l1d_fifo #(.WIDTH(PKT_W), .DEPTH(RESP_FIFO_DEPTH), .CNT_W(RCNT_W), .POP_LAST(1)) u_fifo (
                .clk        (clk),
                .rst        (rst),
                .push       (bank_response_valid[b] && !byp[b]),   // a bypassed answer is not stored
                .push_data  (bank_response_packet[b*PKT_W +: PKT_W]),
                .pop        (f_pop[b]),
                .front_data (f_dout[b*PKT_W +: PKT_W]),
                .empty      (f_empty[b]),
                .full       (),
                .used_slots (),
                .free_slots (bank_response_fifo_free[b*RCNT_W +: RCNT_W]),
                .free_slots_next (bank_response_fifo_free_next[b*RCNT_W +: RCNT_W])
            );
            assign f_pop[b] = load && sel_gnt[b] && !f_empty[b];
            // cache_v2_bank_pipe: BYPASS. An answer pushed into an EMPTY FIFO is visible at the head in the same cycle (head_v / head_d);
            // if the arbiter takes it, it goes straight into the output register and is not stored. Per-bank order is kept
            // (only an empty FIFO is bypassed) and the round-robin stays fair. Load / store hit: 4 cycles at the pins.
            assign head_v[b] = !f_empty[b] || bank_response_valid[b];
            assign head_d[b*PKT_W +: PKT_W] = f_empty[b] ? bank_response_packet[b*PKT_W +: PKT_W] : f_dout[b*PKT_W +: PKT_W];
            assign byp[b]    = f_empty[b] && load && sel_gnt[b];
        end
    endgenerate

    // ---- the output register: loads the chosen FIFO's oldest response when `load` is 1 (rising edge) ----
    // TIMING OPT (RM-2, session 45): the granted FIFO's response by AND-OR with the one-hot grant instead of the binary index (a shift
    // by sel_idx fans out to every bit). The register loads only with sel_any, and then sel_gnt is one-hot at sel_idx: same value.
    reg [PKT_W-1:0] sel_resp;                            // the response of the granted FIFO, PKT_W = 74 bits (scalar), 586 bits (VPU)
    integer          k;
    always @* begin
        sel_resp = {PKT_W{1'b0}};
        for (k = 0; k < NUM_BANKS; k = k + 1)
            sel_resp = sel_resp | ({PKT_W{sel_gnt[k]}} & head_d[k*PKT_W +: PKT_W]);
    end
    always @(posedge clk) begin
        if (rst) begin
            merged_response_valid <= 1'b0;
            merged_response_packet  <= {PKT_W{1'b0}};
        end else if (load) begin
            merged_response_valid <= sel_any;
            if (sel_any)
                merged_response_packet <= sel_resp;
        end
    end

// ---- simulation-only trace: every response moved to the output register, and a STUCK message when the CPU does not read ----
`ifdef L1D_DEBUG
    // Debug trace (simulation only): responses moved into the output
    // register, and a STUCK report when the LSU does not read.
    integer dbg_hold;
    initial dbg_hold = 0;
    always @(posedge clk) begin
        if (!rst) begin
            if (load && sel_any)
                $display("[%0t] L1D MERGE bank %0d -> output  idx=%0d %s (visible next cycle)",
                         $time, sel_idx, head_d[sel_idx*PKT_W + RESP_IDX_LSB +: IDX_BITS],
                         head_d[sel_idx*PKT_W + RESP_RDWR_BIT] ? "ST-ack" : "LD");
            if (merged_response_valid && !lsu_read_enable) begin
                dbg_hold = dbg_hold + 1;
                if (dbg_hold % `L1D_DBG_WD == 0)
                    $display("[%0t] L1D MERGE STUCK output idx=%0d held %0d cycles: read_enable_from_lsu=%b",
                             $time, merged_response_packet[RESP_IDX_LSB +: IDX_BITS], dbg_hold, lsu_read_enable);
            end else dbg_hold = 0;
        end
    end
`endif

endmodule

`default_nettype wire
