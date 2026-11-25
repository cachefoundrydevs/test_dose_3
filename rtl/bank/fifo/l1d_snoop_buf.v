// ============================================================================
// l1d_snoop_buf.v  -  snoop buffer: waiting room of the snoops of one bank (new in v2)
// ============================================================================
// IN SIMPLE WORDS
//   A SNOOP is a question from another core (or the L2): "do you have line X? give it to me / throw it
//   away". (new_snoop_invalidate = 1: the other core WRITES the line, so drop yours; 0: the other core
//   READS it, so keep a shared copy.) The snoop controller (l1d_snoop_ctrl) sends the snoop to the bank that
//   owns the address, and it lands here, in one of SNP_BUF_DEPTH (2) entries.
//   Every cycle this block offers Priority Select ONE snoop (the oldest one that may go); a snoop has the
//   TOP priority in Stage 0 (design rule R-8: snoops never wait behind CPU work).
//
// ORDER (important, found by a test)
//   - Snoops to the SAME line are served in arrival order.
//   - A snoop that cannot be served yet (its line is AMO-locked, or it waits for an install or a writeback) does
//     NOT hold back snoops to OTHER lines: the oldest entry that may go is offered. A strict FIFO deadlocked in
//     the tiny configuration: a lock-held snoop blocked the snoop on another line whose R data a waiting CPU
//     request needed, and that CPU request blocked the AMO write that releases the lock.
//   - Only ONE snoop of the bank is in the pipeline at a time (no entry is offered while one is INFL), so one
//     free place in the snoop-answer FIFO is enough credit (see l1d_bank).
//
// THE LIFE OF A SNOOP ENTRY   (codes SH_* in l1d_encodings.vh)
//
//   READY --(offer_granted: Priority Select took it)--> INFL
//   INFL  --(answered in Stage 2: s2_action_code = SACT_DONE)------------> entry freed
//   INFL  --(its line is being installed by an MSHR entry: SACT_WMSHR)---> WMSHR
//   INFL  --(its line is in a victim slot being written back: SACT_WVC)--> WVC
//   WMSHR / WVC --(that MSHR entry / victim slot was freed)--> READY       (the snoop is retried)
//   A wake-up in the same cycle as the decision to wait is not lost: the entry goes straight back to READY
//   (same function wait_state as in l1d_lsu_req_buf).
//
// WHEN IS A SNOOP HELD BACK?   (cand[] below)
//   - amo_lock_snoop_hold = 1 and the snoop is for amo_lock_line: the AMO lock keeps the line private
//   - amo_read_in_pipeline = 1: an AMO read is in Stage 1 or 2 and has not set its lock yet
//   - an OLDER snoop to the same line is still in the buffer
//   Every wait is bounded (design rule S-1): an install takes a few cycles, a writeback ends with its AXI B
//   response, an AMO lock ends with the AMO write or the lock time-out.
//
// PORTS (new name, old name, meaning)
//   new_snoop_valid                      (was in_valid)                 a new snoop arrives from the snoop controller
//   new_snoop_line_addr                  (was in_line)                  line address of the new snoop
//   new_snoop_invalidate                 (was in_inv)                   1 = invalidate (another core writes), 0 = read
//   new_snoop_ready                      (was in_ready)                 a free entry exists
//   amo_lock_snoop_hold                  (was lock_snp)                 the AMO lock holds snoops to amo_lock_line
//   amo_lock_line                        (was lock_line)                line address the AMO lock is on
//   amo_read_in_pipeline                 (was amor_busy)                an AMO read is in Stage 1 or 2 (its lock is not set yet)
//   amo_lock_set_valid                   (new, Tier 2)                  the AMO lock is set at the next clock edge (bank: s2_amo_read_completed)
//   amo_lock_set_line                    (new, Tier 2)                  the line it is set on (bank: s2_line_addr); amo_lock_line becomes this value
//   offer_valid                          (was sel_valid)                a snoop is offered to Priority Select
//   offer_entry_id                       (was sel_id)                   entry number of the offered snoop
//   offer_line_addr                      (was sel_line)                 line address of the offered snoop
//   offer_invalidate                     (was sel_inv)                  1 = invalidate, 0 = read
//   offer_granted                        (was grant)                    Priority Select granted the offered snoop
//   s2_action_valid                      (was act_valid)                Stage 2 decided the snoop must wait or is finished
//   s2_action_entry_id                   (was act_sid)                  entry number of that snoop
//   s2_action_code                       (was act_code)                 SACT_DONE / SACT_WMSHR / SACT_WVC
//   s2_action_wait_id                    (was act_id)                   MSHR entry or victim-cache slot waited for
//   mshr_entry_freed                     (was mshr_freed)               one bit per MSHR entry: became free (wake-up)
//   victim_slot_freed                    (was vc_freed)                 one bit per victim-cache slot: became free (wake-up)
//   buffer_empty                         (was empty)                    no entry is in use
//
// VERILOG WORDS USED IN THIS FILE
//   reg [1:0] hs [0:N-1]   an array of N registers of 2 bits: hs[i] is the state of entry i
//   !(|infl)               NOT (any bit of infl is 1) = "no entry is in the pipeline"
//   function wait_state    a formula: given the Stage 2 wait code it returns the entry state to enter
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

module l1d_snoop_buf #(`L1D_PARAMS) (
    clk, rst,
    new_snoop_valid, new_snoop_line_addr, new_snoop_invalidate, new_snoop_ready,
    amo_lock_snoop_hold, amo_lock_line, amo_read_in_pipeline, amo_lock_set_valid, amo_lock_set_line,
    offer_valid, offer_entry_id, offer_line_addr, offer_invalidate, offer_granted,
    s2_action_valid, s2_action_entry_id, s2_action_code, s2_action_wait_id,
    mshr_entry_freed, victim_slot_freed,
    buffer_empty
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // N = number of entries; at least 2 even if SNP_BUF_DEPTH is set to 1
    localparam N = (SNP_BUF_DEPTH < 2) ? 2 : SNP_BUF_DEPTH;

    input  wire                    clk;                   // clock: all registers change on its rising edge
    input  wire                    rst;                   // synchronous reset, active high (control state goes to its idle value)
    // new snoop from l1d_snoop_ctrl
    input  wire                    new_snoop_valid;       // a new snoop arrives from the snoop controller
    input  wire [LINE_ADDR_W-1:0]  new_snoop_line_addr;   // line address of the new snoop
    input  wire                    new_snoop_invalidate;  // 1 = invalidate (another core writes), 0 = read
    output wire                    new_snoop_ready;       // a free entry exists
    // AMO lock of this bank: snoops to the locked line are held
    input  wire                    amo_lock_snoop_hold;   // the AMO lock holds snoops to amo_lock_line
    input  wire [LINE_ADDR_W-1:0]  amo_lock_line;         // line address the AMO lock is on
    input  wire                    amo_read_in_pipeline;  // an AMO read is in Stage 1 or 2 (its lock is not set yet)
    // TIMING OPT (Tier 2): the event that sets the AMO lock, one cycle before amo_lock_line shows the new line. l1d_reservation_unit changes
    // amo_lock_line ONLY at this event (s2_amo_read_completed, line s2_line_addr); the buffer uses it to keep "entry line == lock line" in a register.
    input  wire                    amo_lock_set_valid;    // the AMO lock is set at the next clock edge (bank: s2_amo_read_completed)
    input  wire [LINE_ADDR_W-1:0]  amo_lock_set_line;     // the line it is set on (bank: s2_line_addr); amo_lock_line becomes this value
    // entry offered to Priority Select
    output wire                    offer_valid;           // a snoop is offered to Priority Select
    output wire [SNP_ID_W-1:0]     offer_entry_id;        // entry number of the offered snoop
    output wire [LINE_ADDR_W-1:0]  offer_line_addr;       // line address of the offered snoop
    output wire                    offer_invalidate;      // 1 = invalidate, 0 = read
    input  wire                    offer_granted;         // Priority Select granted the offered snoop
    // Stage 2 outcome of the snoop in the pipeline
    input  wire                    s2_action_valid;       // Stage 2 decided the snoop must wait or is finished
    input  wire [SNP_ID_W-1:0]     s2_action_entry_id;    // entry number of that snoop
    input  wire [1:0]              s2_action_code;        // SACT_DONE / SACT_WMSHR / SACT_WVC
    input  wire [WID_W-1:0]        s2_action_wait_id;     // MSHR entry or victim-cache slot waited for
    // wake-ups
    input  wire [MSHR_ENTRIES-1:0] mshr_entry_freed;      // one bit per MSHR entry: became free (wake-up)
    input  wire [VC_SLOTS-1:0]     victim_slot_freed;     // one bit per victim-cache slot: became free (wake-up)
    output wire                    buffer_empty;          // no entry is in use

    // ---- the memory of the buffer: one record per entry ----
    reg                   ev  [0:N-1];                    // entry is in use
    reg [1:0]             hs  [0:N-1];                    // entry state (SH_READY / SH_INFL / SH_WMSHR / SH_WVC)
    reg [LINE_ADDR_W-1:0] el  [0:N-1];                    // line address of the snoop
    reg                   ei  [0:N-1];                    // 1 = invalidate, 0 = read
    reg [WID_W-1:0]       wid [0:N-1];                    // MSHR entry or victim slot the entry waits for
    // TIMING OPT (Tier 2): the two 58-bit line compares of the candidate logic are kept in registers, so that they leave the path to the offer:
    //   sm[c][o]  = (el[c] == el[o])           same line, a symmetric matrix; written when an entry is allocated (compare of the new line with the others)
    //   lm_q[c]   = (el[c] == amo_lock_line)   the entry is on the line of the AMO lock; amo_lock_line changes only at amo_lock_set_valid
    // Both are exact copies of the compares at every clock cycle (reset: all el and amo_lock_line are 0, so all are 1).
    reg [N-1:0]           sm  [0:N-1];                    // same-line matrix, N x N bits
    reg [N-1:0]           lm_q;                           // entry is on the AMO-locked line, N bits

    // ---------------- allocation: first free entry ----------------
    // a new snoop takes the lowest-numbered free entry
    wire [N-1:0]        used;
    // TIMING OPT (Tier 1, S-3): the lowest-numbered free entry as ONE-HOT bits straight from the in-use flags (prefix chain), not as a number
    // from l1d_pri_enc that every entry and the age matrix then decode again. Same entry as before. free_id (the number) is kept for the
    // simulation trace only.
    wire [N-1:0]        free_g;                           // lowest free entry, one-hot (zero when all are in use), N = 2..8 bits
    reg  [N-1:0]        free_g_r;
    reg                 free_seen;
    integer             fi;
    always @* begin
        free_seen = 1'b0;
        for (fi = 0; fi < N; fi = fi + 1) begin
            free_g_r[fi] = ~used[fi] & ~free_seen;
            free_seen    = free_seen | ~used[fi];
        end
    end
    assign free_g = free_g_r;
    wire                any_free = ~&used;                // some entry is free
    reg  [SNP_ID_W-1:0] free_id;                          // number of the free entry (trace only)
    integer             fj;
    always @* begin
        free_id = {SNP_ID_W{1'b0}};
        for (fj = 0; fj < N; fj = fj + 1)
            if (free_g[fj]) free_id = free_id | fj[SNP_ID_W-1:0];
    end
    assign new_snoop_ready = any_free;
    wire do_alloc   = new_snoop_valid && any_free;

    // ---------------- which entries may go ----------------
    // age matrix output (who arrived before whom), entries in the pipeline (infl), and the candidates that may go (cand)
    wire [N*N-1:0] older_flat;
    wire [N-1:0]   infl;
    reg  [N-1:0]   cand;
    integer c;
    integer o;

    genvar g;
    generate
        for (g = 0; g < N; g = g + 1) begin : g_ent
            assign used[g] = ev[g];
            assign infl[g] = ev[g] && (hs[g] == SH_INFL);
        end
    endgenerate

    // ---- which entries may be offered: READY, not held by the AMO lock, no AMO read still setting its lock,
    //      and no older snoop to the same line ----
    always @* begin
        for (c = 0; c < N; c = c + 1) begin
            cand[c] = ev[c] && (hs[c] == SH_READY) && !(amo_lock_snoop_hold && lm_q[c]) && !amo_read_in_pipeline;   // Tier 2: lm_q[c] = (el[c] == amo_lock_line)
            for (o = 0; o < N; o = o + 1)
                if (o != c && ev[o] && older_flat[o*N + c] && sm[o][c])    // Tier 2: sm[o][c] = (el[o] == el[c])
                    cand[c] = 1'b0;                  // an older snoop to the same line goes first
        end
    end

    // the oldest candidate is the one offered (age matrix)
    wire any_cand;
    wire [N-1:0] oldest_oh;                               // the oldest candidate, one-hot (zero when none), N = 2..8 bits
    l1d_age_matrix #(.N(N), .IDX_W(SNP_ID_W), .ONEHOT_ALLOC(1), .ONEHOT_ENC(1)) u_age (   // Tier 1: one-hot allocate, OR-encoded index
        .clk             (clk),
        .rst             (rst),
        .allocate_valid  (do_alloc),
        .allocate_index  ({SNP_ID_W{1'b0}}),
        .allocate_onehot (free_g),
        .candidate_mask  (cand),
        .older_than_flat (older_flat),
        .oldest_onehot   (oldest_oh),
        .oldest_index    (offer_entry_id),
        .any_candidate   (any_cand)
    );

    // ---- outputs: offer the oldest candidate, but only while no snoop of this bank is in the pipeline ----
    assign offer_valid = any_cand && !(|infl);
    // TIMING OPT (Tier 1, S-2): the line address and the invalidate bit by AND-OR with the one-hot oldest candidate, not by the binary number.
    // The old code showed entry 0 when nothing is offered (the number is 0 then); entry 0 is also selected when no other entry is, so the
    // ports are identical in every cycle.
    wire [N-1:0] pick = {oldest_oh[N-1:1], ~|oldest_oh[N-1:1]};   // one-hot select of the offered entry, N = 2..8 bits
    reg  [LINE_ADDR_W-1:0] offer_line_r;
    reg                    offer_inv_r;
    integer                pk;
    always @* begin
        offer_line_r = {LINE_ADDR_W{1'b0}};
        offer_inv_r  = 1'b0;
        for (pk = 0; pk < N; pk = pk + 1) begin
            offer_line_r = offer_line_r | ({LINE_ADDR_W{pick[pk]}} & el[pk]);
            offer_inv_r  = offer_inv_r  | (pick[pk] & ei[pk]);
        end
    end
    assign offer_line_addr  = offer_line_r;
    assign offer_invalidate = offer_inv_r;
    assign buffer_empty     = ~|used;

    // state an entry enters for a wait code, or READY if the event it would
    // wait for happens in this very cycle
    // wait_state(code, id): the entry state for a Stage 2 wait code, or READY if the awaited MSHR entry / victim slot was
    // freed in THIS very cycle (otherwise the wake-up would be missed).
    function [1:0] wait_state;
        input [1:0]       code;
        input [WID_W-1:0] id;
        begin
            case (code)
                SACT_WMSHR: wait_state = mshr_entry_freed[id[MSHR_ID_W-1:0]] ? SH_READY : SH_WMSHR;
                SACT_WVC:   wait_state = victim_slot_freed[id[VC_ID_W-1:0]]     ? SH_READY : SH_WVC;
                default:    wait_state = SH_READY;
            endcase
        end
    endfunction

    // ---- TIMING OPT (Tier 2): the compares that fill sm and lm_q. They sit at the END of a path (flop or input -> compare -> flop), not
    //      in front of the offer. eq_new_el[o] = the new snoop is on the line of entry o; eq_el_set[o] = entry o is on the line the AMO lock is
    //      being set on; eq_new_set / eq_new_lock = the new snoop is on that line / on the line of the lock that is set now. ----
    wire [N-1:0] eq_new_el;
    wire [N-1:0] eq_el_set;
    wire         eq_new_set  = (new_snoop_line_addr == amo_lock_set_line);
    wire         eq_new_lock = (new_snoop_line_addr == amo_lock_line);
    generate
        for (g = 0; g < N; g = g + 1) begin : g_cmp
            assign eq_new_el[g] = (new_snoop_line_addr == el[g]);
            assign eq_el_set[g] = (el[g] == amo_lock_set_line);
        end
    endgenerate

    // ---- the entry state machine (clocked) ----
    integer s;
    integer j2;
    always @(posedge clk) begin
        if (rst) begin
            for (s = 0; s < N; s = s + 1) begin
                ev[s]  <= 1'b0;
                hs[s]  <= SH_READY;
                el[s]  <= {LINE_ADDR_W{1'b0}};
                ei[s]  <= 1'b0;
                wid[s] <= {WID_W{1'b0}};
                sm[s]  <= {N{1'b1}};                  // Tier 2: after reset all lines are 0 and the lock line is 0
            end
            lm_q <= {N{1'b1}};
        end else begin
            // wake-ups (the explicit events below override them)
            for (s = 0; s < N; s = s + 1) begin
                case (hs[s])
                    SH_WMSHR: if (mshr_entry_freed[wid[s][MSHR_ID_W-1:0]]) hs[s] <= SH_READY;
                    SH_WVC:   if (victim_slot_freed[wid[s][VC_ID_W-1:0]])     hs[s] <= SH_READY;
                    default: ;
                endcase
                // TIMING OPT (Tier 1, S-2 / S-3): the events per entry with one-hot selects (free_g, oldest_oh) and no decoded numbers for the
                // allocation and the grant; same order of the statements as before, so the same event wins when two hit one entry.
                if (do_alloc && free_g[s]) begin
                    ev[s] <= 1'b1;
                    hs[s] <= SH_READY;
                    el[s] <= new_snoop_line_addr;
                    ei[s] <= new_snoop_invalidate;
                end
                // Tier 2: same-line matrix. When entry s or entry j2 is allocated, the pair (s, j2) is compared (with the old line of the other one).
                for (j2 = 0; j2 < N; j2 = j2 + 1) begin
                    if (j2 != s) begin
                        if (do_alloc && free_g[s])       sm[s][j2] <= eq_new_el[j2];
                        else if (do_alloc && free_g[j2]) sm[s][j2] <= eq_new_el[s];
                    end
                end
                // Tier 2: "entry on the AMO-locked line". amo_lock_line becomes amo_lock_set_line when amo_lock_set_valid, otherwise it holds;
                // the line of entry s becomes new_snoop_line_addr when entry s is allocated, otherwise it holds.
                if (amo_lock_set_valid)
                    lm_q[s] <= (do_alloc && free_g[s]) ? eq_new_set : eq_el_set[s];
                else if (do_alloc && free_g[s])
                    lm_q[s] <= eq_new_lock;
                if (offer_granted && oldest_oh[s])        // (offer_granted comes only with offer_valid)
                    hs[s] <= SH_INFL;
                if (s2_action_valid && (s2_action_entry_id == s)) begin
                    if (s2_action_code == SACT_DONE) begin
                        ev[s] <= 1'b0;
                        hs[s] <= SH_READY;
                    end else begin
                        hs[s]  <= wait_state(s2_action_code, s2_action_wait_id);
                        wid[s] <= s2_action_wait_id;
                    end
                end
            end
        end
    end

// ---- simulation-only trace (compiled only when L1D_DEBUG is defined): arrivals, state changes, STUCK report ----
`ifdef L1D_DEBUG
    // Debug trace (simulation only): arrivals, state changes, STUCK report.
    reg [1:0] dbg_ps  [0:N-1];
    reg       dbg_pv  [0:N-1];
    integer   dbg_age [0:N-1];
    integer   dbg_k;
    initial for (dbg_k = 0; dbg_k < N; dbg_k = dbg_k + 1) begin
        dbg_ps[dbg_k] = SH_READY; dbg_pv[dbg_k] = 1'b0; dbg_age[dbg_k] = 0;
    end
    always @(posedge clk) begin
        if (!rst) begin
            if (do_alloc)
                $display("[%0t] L1D SNPB %m snoop in: entry %0d %s line=%h", $time, free_id,
                         new_snoop_invalidate ? "INV " : "READ", {new_snoop_line_addr, {OFFSET_W{1'b0}}});
            if (new_snoop_valid && !any_free)
                $display("[%0t] L1D SNPB %m ERROR snoop pushed while the buffer is full", $time);
            for (dbg_k = 0; dbg_k < N; dbg_k = dbg_k + 1) begin
                if (ev[dbg_k] != dbg_pv[dbg_k] || (ev[dbg_k] && hs[dbg_k] != dbg_ps[dbg_k])) begin
                    if (ev[dbg_k])
                        $display("[%0t] L1D SNPB %m entry %0d state %0d line=%h wait_id=%0d",
                                 $time, dbg_k, hs[dbg_k], {el[dbg_k], {OFFSET_W{1'b0}}}, wid[dbg_k]);
                    dbg_pv[dbg_k]  = ev[dbg_k];
                    dbg_ps[dbg_k]  = hs[dbg_k];
                    dbg_age[dbg_k] = 0;
                end else if (ev[dbg_k]) begin
                    dbg_age[dbg_k] = dbg_age[dbg_k] + 1;
                    if (dbg_age[dbg_k] % `L1D_DBG_WD == 0)
                        $display("[%0t] L1D SNPB %m STUCK entry %0d in state %0d for %0d cycles line=%h locked=%b (0 READY; 2 WMSHR; 3 WVC)",
                                 $time, dbg_k, hs[dbg_k], dbg_age[dbg_k], {el[dbg_k], {OFFSET_W{1'b0}}},
                                 amo_lock_snoop_hold && el[dbg_k] == amo_lock_line);
                end
            end
        end
    end
`endif

endmodule

`default_nettype wire
