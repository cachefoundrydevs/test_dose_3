// ============================================================================
// l1d_resp_merge.v  -  response merger (four banks -> one LSU response port)
// ============================================================================
// IN SIMPLE WORDS
//   Each bank can finish one request per cycle, but the CPU has ONE response port. So every bank
//   puts its response into its own little FIFO (a queue). A round-robin arbiter (written flat inside this module since RM-3, same order as l1d_arb_rr) picks
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
// TIMING OPT: RM-1 POP_LAST FIFOs (session 42); RM-2 one-hot response select (session 45); RM-3 (session 80, GR-1 rework): flat round-robin
//   grant inside this module (no l1d_arb_rr, no use_masked mux), push / pop / bypass / data select from `sel_win` (no head_v of the bank in
//   the cone), the empty-FIFO mux folded into the AND-OR, `free_next` picked by push / pop from free - 1 / free / free + 1. Same pins in every cycle.
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

    // signals of the per-bank FIFOs: f_empty = FIFO is empty, f_full = FIFO is full, f_dout = its oldest response, f_pop = take it out
    wire [NUM_BANKS-1:0]        f_empty;
    wire [NUM_BANKS-1:0]        f_full;                // NUM_BANKS = 4 bits
    wire [NUM_BANKS*PKT_W-1:0] f_dout;
    wire [NUM_BANKS-1:0]        f_pop;
    wire [NUM_BANKS-1:0]        f_push;               // RM-3: the push the FIFO sees (an answer that is not taken directly)
    wire [NUM_BANKS-1:0]        head_v;               // cache_v2_bank_pipe: the bank has an answer at its head (stored, or pushed now into an empty FIFO)
    wire [NUM_BANKS*PKT_W-1:0] head_d;               // that answer (RM-3: used by the debug trace only; the output register takes the flat AND-OR below)
    wire [NUM_BANKS-1:0]        byp;                  // the answer pushed now is taken directly (FIFO empty, granted, register loads)

    // The output register can take a new response when it is empty or when the
    // LSU is reading the current one this cycle.
    // load = the output register can take a new response now: it is empty, or the CPU is reading the current one this cycle
    wire load = !merged_response_valid || lsu_read_enable;

    // the arbiter's answer: sel_gnt = one-hot choice, sel_idx = its number, sel_any = at least one FIFO has a response
    wire [NUM_BANKS-1:0] sel_gnt;
    wire [NUM_BANKS-1:0] sel_win;                         // RM-3: bank b would win if it asks (nobody who asks beats it); needs no head_v[b]
    reg  [BANK_W-1:0]    sel_idx;
    wire                 sel_any = |head_v;

    // ---- RM-3 (GR-1 rework): the round-robin arbiter written flat, inside this module ----
    // Old: l1d_arb_rr = masked requests -> OR4 (use_masked) -> mux of two lowest-set-bit chains; every consumer waited for that.
    // New: `turn_above` is the registered thermometer "bank j is after the one served last" (bit 0 is always 0, so it has no flop).
    //      Bank i beats bank j when        i < j :  turn_above[i] | ~turn_above[j]      (the lower bank wins, unless only j is after `last`)
    //                                      i > j :  turn_above[i] & ~turn_above[j]      (the higher bank wins only when only i is after `last`)
    //      Both are flop-only terms (early). Bank j is granted when it asks and no asking bank beats it: one AND-OR, no mux, no use_masked.
    //      This is exactly the order last+1 .. N-1, 0 .. last of l1d_arb_rr (checked over every request pattern and mask, N = 2 to 6).
    reg  [NUM_BANKS-1:1] turn_above_q;                    // bank j (j >= 1) is after the bank served last; 0 after reset; NUM_BANKS-1 = 3 flops
    wire [NUM_BANKS-1:0] turn_above = {turn_above_q, 1'b0};  // NUM_BANKS = 4 bits, bit 0 constant 0

    genvar b;
    genvar rival;
    generate
        for (b = 0; b < NUM_BANKS; b = b + 1) begin : g_grant
            wire [NUM_BANKS-1:0] beaten_by;               // bit i = bank i asks and beats bank b, NUM_BANKS = 4 bits
            for (rival = 0; rival < NUM_BANKS; rival = rival + 1) begin : g_rival
                if (rival == b) begin : g_self
                    assign beaten_by[rival] = 1'b0;
                end else if (rival < b) begin : g_lower
                    assign beaten_by[rival] = head_v[rival] & (turn_above[rival] | ~turn_above[b]);
                end else begin : g_higher
                    assign beaten_by[rival] = head_v[rival] & turn_above[rival] & ~turn_above[b];
                end
            end
            assign sel_win[b] = ~|beaten_by;
            assign sel_gnt[b] = head_v[b] & sel_win[b];
        end
    endgenerate

    // the number of the granted bank (debug trace only; the datapath uses the one-hot sel_gnt)
    integer kb;
    always @* begin
        sel_idx = {BANK_W{1'b0}};
        for (kb = 0; kb < NUM_BANKS; kb = kb + 1)
            if (sel_gnt[kb]) sel_idx = sel_idx | kb[BANK_W-1:0];
    end

    // the turn moves on when the output register really loads an answer (advance = load, as l1d_arb_rr): the banks after the winner
    reg [NUM_BANKS-1:1] turn_above_next;                  // NUM_BANKS-1 = 3 bits: bit j = some granted bank is below j
    integer kt;
    always @* begin
        turn_above_next = {(NUM_BANKS-1){1'b0}};
        for (kt = 1; kt < NUM_BANKS; kt = kt + 1)
            turn_above_next[kt] = |(sel_gnt & ((1 << kt) - 1));
    end
    always @(posedge clk) begin
        if (rst)
            turn_above_q <= {(NUM_BANKS-1){1'b0}};
        else if (load && sel_any)
            turn_above_q <= turn_above_next;
    end

    // one response FIFO per bank; a FIFO is popped when the arbiter chose it and the output register loads
    generate
        for (b = 0; b < NUM_BANKS; b = b + 1) begin : g_fifo
            // TIMING OPT (RM-1, session 42): POP_LAST = 1, because the pop of these FIFOs comes late (FIFO not empty -> arbiter grant ->
            // output register loads -> pop); the level update then keeps the pop out of its arithmetic. Same FIFO behaviour in every cycle.
            // RM-3: free_slots_next is made here (see g_credit) from values that are ready early, so the FIFO's own output is left open.
            l1d_fifo #(.WIDTH(PKT_W), .DEPTH(RESP_FIFO_DEPTH), .CNT_W(RCNT_W), .POP_LAST(1)) u_fifo (
                .clk        (clk),
                .rst        (rst),
                .push       (f_push[b]),
                .push_data  (bank_response_packet[b*PKT_W +: PKT_W]),
                .pop        (f_pop[b]),
                .front_data (f_dout[b*PKT_W +: PKT_W]),
                .empty      (f_empty[b]),
                .full       (f_full[b]),
                .used_slots (),
                .free_slots (bank_response_fifo_free[b*RCNT_W +: RCNT_W]),
                .free_slots_next ()
            );
            // RM-3: head_v / byp / f_pop / f_push use sel_win (no head_v[b] in the cone): when the FIFO is not empty head_v[b] is 1 anyway,
            // and a push means bank_response_valid[b] = 1 (so head_v[b] = 1). Same values as the old sel_gnt based terms.
            assign f_pop[b]  = load && !f_empty[b] && sel_win[b];
            // cache_v2_bank_pipe: BYPASS. An answer pushed into an EMPTY FIFO is visible at the head in the same cycle (head_v / head_d);
            // if the arbiter takes it, it goes straight into the output register and is not stored. Per-bank order is kept
            // (only an empty FIFO is bypassed) and the round-robin stays fair. Load / store hit: 4 cycles at the pins.
            assign head_v[b] = !f_empty[b] || bank_response_valid[b];
            assign head_d[b*PKT_W +: PKT_W] = f_empty[b] ? bank_response_packet[b*PKT_W +: PKT_W] : f_dout[b*PKT_W +: PKT_W];
            assign byp[b]    = f_empty[b] && load && sel_win[b];       // only meaningful with bank_response_valid[b] = 1
            assign f_push[b] = bank_response_valid[b] && !byp[b];      // a bypassed answer is not stored
        end
    endgenerate

    // ---- RM-3: the credit after this cycle (bank_response_fifo_free_next), picked by the late push / pop from early values ----
    // The FIFO takes a push when it is not full and a pop only when it is not empty (f_pop already has that). Free places after the cycle:
    // push only -> free - 1, pop only -> free + 1, else free. free is registered state, free - 1 and free + 1 are ready before push / pop.
    genvar c;
    generate
        for (c = 0; c < NUM_BANKS; c = c + 1) begin : g_credit
            wire [RCNT_W-1:0] free_now  = bank_response_fifo_free[c*RCNT_W +: RCNT_W];  // RCNT_W = 3 bits
            wire [RCNT_W-1:0] free_less = free_now - {{(RCNT_W-1){1'b0}}, 1'b1};      // one more answer stored
            wire [RCNT_W-1:0] free_more = free_now + {{(RCNT_W-1){1'b0}}, 1'b1};      // one answer taken out
            wire              store_one = f_push[c] && !f_full[c];                    // the FIFO really stores an answer
            wire              take_one  = f_pop[c];                                  // the FIFO really gives one out
            assign bank_response_fifo_free_next[c*RCNT_W +: RCNT_W] =
                  ({RCNT_W{ store_one && !take_one}} & free_less)
                | ({RCNT_W{!store_one &&  take_one}} & free_more)
                | ({RCNT_W{ store_one ==  take_one}} & free_now);
        end
    endgenerate

    // ---- the output register: loads the chosen FIFO's oldest response when `load` is 1 (rising edge) ----
    // TIMING OPT (RM-2, session 45): the granted FIFO's response by AND-OR with the one-hot grant instead of the binary index (a shift
    // by sel_idx fans out to every bit). The register loads only with sel_any, and then sel_gnt is one-hot at sel_idx: same value.
    // RM-3: the "FIFO empty -> take the pushed answer, else the stored one" mux is folded into the AND-OR: two selects per bank,
    // pick_new = granted and the FIFO is empty (the answer comes straight from the bank), pick_old = granted and the FIFO has it stored.
    reg [PKT_W-1:0] sel_resp;                            // the response of the granted FIFO, PKT_W = 74 bits (scalar), 586 bits (VPU)
    integer          k;
    reg              pick_new;                           // bank k: take the answer pushed this cycle (FIFO empty)
    reg              pick_old;                           // bank k: take the oldest stored answer
    always @* begin
        sel_resp = {PKT_W{1'b0}};
        for (k = 0; k < NUM_BANKS; k = k + 1) begin
            pick_new = bank_response_valid[k] && f_empty[k] && sel_win[k];
            pick_old = !f_empty[k] && sel_win[k];
            sel_resp = sel_resp | ({PKT_W{pick_new}} & bank_response_packet[k*PKT_W +: PKT_W])
                                | ({PKT_W{pick_old}} & f_dout[k*PKT_W +: PKT_W]);
        end
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
