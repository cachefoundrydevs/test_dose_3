// ==========================================================================
// l1d_pf3_stride_entry.v - one entry of the stride table (RPT), with its
//                          own update and allocation logic
// ==========================================================================
// IN SIMPLE WORDS : one "note card" of the stride prefetcher. It belongs to
//   one load / store instruction (PC tag) and keeps: the last page offset
//   seen (lo), the stride between the last two accesses, the trust state
//   (Chen and Baer: Initial, Transient, Steady, No-prediction), the
//   ready-made prefetch target for the next access that keeps the stride
//   (tgt = lo + 3 x stride = next access + DIST x stride, DIST = 2) and the
//   ready-made expected next offset (exp = lo + stride). Every card checks
//   the training event itself and, if the event is its own, stores its new
//   contents in the same cycle (S2). A card chosen as the victim of a
//   missing event only notes "allocate" (alloc_q, a flop kept in 5 copies) and is written
//   in the next cycle from the parent's copy of that event (p_*). In that
//   cycle the next event must see the new contents: a hit on such a card
//   depends only on the two events, so the parent works the result out
//   once per event copy (l1d_pf3_stride_anew, a_*), while the card works
//   out its own result from its flops; alloc_q picks one of the two in the
//   last 2:1 before each flop. So the late allocation decision drives one
//   flop, and its copies drive only last-level selects (rule T-9).
//
// STAGE / CYCLE   : S2 of the prefetch system (cycle 2 after the training
//   record reaches the unit pins). Every input comes from a flop: the
//   card's own flops or one copy of the S1 event register of the parent.
//   Outputs hit / fire_* / tgt_lip are combinational (S2) and are
//   registered by the parent (per set, then read by the rank stage S3).
// INSTANTIATED BY : l1d_pf3_stride, generate g_entry, RPT_SETS x RPT_WAYS
//   = 64 copies.
//
// PARAMETERS:
//   name        default unit    meaning
//   TAG_W       12      bits    folded PC tag
//   PAGE_W      12      bits    page offset (4 KB page)
//   OFFSET_W    6       bits    byte offset in a line (64-byte line)
//   PH_W        10      bits    page hash (same-page check, filter key)
//   FILT_DEPTH  4       entries duplicate filter depth
//   NL          4       lanes   training lanes compared in S1
//
// PORTS (default widths; "event" = the training record now in S2):
//   clk, rst          in  1     clock, synchronous reset (high)
//   clear             in  1     invalidate (satp write)
//   ev_set_on         in  1     an event is in S2 and has this card's set
//   s1_tag, s1_st     in  4x12,4 S1: every lane's PC tag and store bit
//                               (compared with the card's key, registered)
//   ev_lane           in  4     the S2 event's lane, one-hot (flop)
//   ev_p2_key         in  4     S1 lane l had E-2's key (flop; the event's
//                               lane is used when the card was written
//                               last cycle)
//   s1_ph             in  4x10  S1: every lane's page hash (compared with
//                               the card's page hash, registered)
//   ev_ph_ev, ev_ph_p in  4, 4  S1 lane l had the page hash of the event
//                               now in S2 / of the event before (flops;
//                               the event's lane is used when the card's
//                               page hash was written last cycle)
//   ev_tag, ev_st     in  12, 1 the event's PC tag and store bit (L1D_DEBUG
//                               check of own_hit only)
//   ev_lo, ev_lo3     in  12,14 page offset of the event and 3 x it
//   ev_ph             in  10    page hash of the event
//   filt_pm           in  4     filter slot k: valid and same page hash
//                               as the event (slot 0 newest)
//   filt_lip          in  4x6   line-in-page of filter slot k
//   prev_pm           in  1     the event before has the same page hash
//   prev_lip          in  6     line-in-page of the event before's
//                               candidate (parent flop)
//   p_tag, p_st       in  12, 1 the event before (written now if alloc_q)
//   p_lo, p_lo3, p_ph in  12,14,10
//   a_key, a_correct  in  1, 1  shared: E has E-1's tag and store bit;
//                               that hit is correct (l1d_pf3_stride_anew)
//   a_stride, a_exp   in  13,13 shared: new contents of a card E-1
//   a_s3, a_tgt, a_nz in  15,15,1  allocated, if E hits it
//   alloc_en          in  1     the event missed in this set and this card
//                               is the set's victim: note it (alloc_q),
//                               write the card in the next cycle
//   hit               out 1     the event is this card's (combinational)
//   class_rank        out 3     replacement class: 0 free, 1 No-pred,
//                               2 Initial, 3 Transient, 4 Steady (flops)
//   fire_new          out 1     fires, and the target is not filter slot 3
//                               (used when the event before sent nothing)
//   fire_dup          out 1     fires, and the target is not the event
//                               before's candidate (used when it was sent)
//   tgt_lip           out 6     line-in-page of the stored target (flops)
//
// ONE-AT-A-TIME RULE (same as the 9-stage RTL and the README):
//   same page (sp)  = page hash equal to the card's
//   correct         = sp and (lo_new == lo + stride) = sp and lo_new == exp
//   keep            = correct or state Steady (stride kept)
//   new stride      = keep ? stride : (sp ? lo_new - lo : 0)
//   new state       = Chen and Baer table (README section 2)
//   issue           = correct, state not No-pred, stride not 0
//   target          = lo_new + 2 x stride = (stored) tgt, sent when it is
//                     inside the page, not the event's own line and not
//                     in the duplicate filter
//   allocation      = Initial, stride 0, tgt = exp = lo_new
//
// WORKED EXAMPLE (stride 128, Steady, lo = 0x100):
//   stored: stride 128, exp 0x180, s3 = 384, tgt = 0x100 + 384 = 0x280.
//   event lo = 0x180 == exp -> correct, fire with target 0x280 (line 0xA).
//   New contents: lo 0x180, exp 0x200, tgt = 0x180 + 384 = 0x300.
//
// LOGIC LEVELS (T-9, estimate): hit 4 (XNOR + 15-input AND), sp / exp
//   compare 4, keep 6; the six adders 6 in parallel; next value select:
//   sp choice 1 (ready before keep), hit and keep 2:1 (2), alloc_q 2:1
//   last (1): about 10 into the card's flops, all from flops (no view
//   select first). alloc_en -> alloc_q only (1 flop). Critical path:
//   ev_lo -> adder -> hit / same-page 2:1 -> Steady 2:1 -> alloc 2:1 ->
//   card flop (the keep compare only drives the trust state and fire).
//
// ASSUMPTIONS (V-9.2): A1 alloc_en and hit are never 1 together (the parent
//   allocates only in a set no card of which hit); a hit on a card with
//   alloc_q set takes the shared a_* contents. alloc_q means any
//   copy alloc_cp[k] (all equal). A card with alloc_q never fires (stride
//   0, not armed), so fire reads the card's flops only. A2 PAGE_W > OFFSET_W + 2
//   (the bank bits lie inside the line-in-page).
// ==========================================================================
`ifndef L1D_PF3_STRIDE_ENTRY_V
`define L1D_PF3_STRIDE_ENTRY_V
`default_nettype none

module l1d_pf3_stride_entry #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter TAG_W      = 12, // default 12 bits: folded PC tag
    parameter PAGE_W     = 12, // default 12 bits: page offset
    parameter OFFSET_W   = 6,  // default 6 bits: byte in line
    parameter PH_W       = 10, // default 10 bits: page hash
    parameter FILT_DEPTH = 4,  // default 4 entries: duplicate filter
    parameter NL         = 4   // default 4 lanes: training lanes in S1
) (
    // ---- clock and reset ----
    input  wire                                 clk,        // clock
    input  wire                                 rst,        // sync reset, high
    input  wire                                 clear,      // invalidate all
    // ---- the event in S2 (one copy of the parent's S1 register) ----
    input  wire                                 ev_set_on,  // event, this set
    // ---- S1 (one cycle earlier): the lanes' keys, compared here with
    //      the card's key and registered (the S2 hit is then flops) ----
    input  wire [NL*TAG_W-1:0]                  s1_tag,     // 4 x 12: lane PC tags
    input  wire [NL-1:0]                        s1_st,      // 4: lane store bits
    input  wire [NL-1:0]                        ev_lane,    // 4: lane of the S2
                                                            //   event (flop)
    input  wire [NL-1:0]                        ev_p2_key,  // 4: lane l had the
                                                            //   key of E-2 (flop)
    input  wire [NL*PH_W-1:0]                   s1_ph,      // 4 x 10: lane page
                                                            //   hashes (S1)
    input  wire [NL-1:0]                        ev_ph_ev,   // 4: lane l had the
                                                            //   S2 event's page
                                                            //   hash (flop)
    input  wire [NL-1:0]                        ev_ph_p,    // 4: lane l had E-1's
                                                            //   page hash (flop)
    // ev_tag / ev_st: read only by the L1D_DEBUG check of own_hit
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [TAG_W-1:0]                     ev_tag,     // TAG_W = 12 bits
    input  wire                                 ev_st,      // store event
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [PAGE_W-1:0]                    ev_lo,      // PAGE_W = 12 bits
    input  wire [PAGE_W+1:0]                    ev_lo3,     // 14 bits, 3 x lo
    input  wire [PH_W-1:0]                      ev_ph,      // PH_W = 10 bits
    // ---- duplicate filter and the event before (parent flops) ----
    input  wire [FILT_DEPTH-1:0]                filt_pm,    // 4: valid, same page
    input  wire [FILT_DEPTH*(PAGE_W-OFFSET_W)-1:0] filt_lip, // 4 x 6 bits
    input  wire                                 prev_pm,    // same page as E-1
    input  wire [PAGE_W-OFFSET_W-1:0]           prev_lip,   // 6 bits, E-1 cand.
    input  wire [TAG_W-1:0]                     p_tag,      // 12 bits: E-1 tag
    input  wire                                 p_st,       // E-1 store bit
    input  wire [PAGE_W-1:0]                    p_lo,       // 12 bits: E-1 offset
    input  wire [PAGE_W+1:0]                    p_lo3,      // 14 bits: 3 x p_lo
    input  wire [PH_W-1:0]                      p_ph,       // 10 bits: E-1 page
    // ---- new contents if E hits the card E-1 allocated (parent, shared) ----
    input  wire                                 a_key,      // E tag == E-1 tag
    input  wire                                 a_correct,  // correct such hit
    input  wire [PAGE_W:0]                      a_stride,   // 13 bits
    input  wire [PAGE_W:0]                      a_exp,      // 13 bits
    input  wire [PAGE_W+2:0]                    a_s3,       // 15 bits
    input  wire [PAGE_W+2:0]                    a_tgt,      // 15 bits
    input  wire                                 a_nz,       // stride != 0
    // ---- allocation (parent: set missed and this card is the victim) ----
    input  wire                                 alloc_en,   // write event now
    // ---- results ----
    output wire                                 hit,        // event is ours
    output reg  [2:0]                           class_rank, // replacement class
    output wire                                 fire_new,   // fire, not slot 3
    output wire                                 fire_dup,   // fire, not E-1 key
    output wire [PAGE_W-OFFSET_W-1:0]           tgt_lip     // 6 bits, target
);

    // ---- derived sizes ----
    localparam STRIDE_W = PAGE_W + 1;              // 13 bits: signed stride
    localparam TGT_W    = PAGE_W + 3;              // 15 bits: target + overflow
    localparam LO3_W    = PAGE_W + 2;              // 14 bits: 3 x lo
    localparam LIP_W    = PAGE_W - OFFSET_W;       // 6 bits: line in page
    // own_hit copies (rule T-6), one per register group: 0 stride / nz,
    // 1 exp, 2 s3, 3 tgt, 4 lo / lo3 / ph / state, 5 hit / fire / checks
    localparam OH_N     = 6;                       // 6 copies
    localparam GRP_N    = OH_N - 2;                // 4 groups with one-hot
                                                   //   selects (0 .. 3)
    localparam AQ_N     = 5;                       // 5 copies of alloc_q

    // ---- trust states (Chen and Baer) ----
    localparam [1:0] S_INIT   = 2'd0;              // Initial
    localparam [1:0] S_TRANS  = 2'd1;              // Transient
    localparam [1:0] S_STEADY = 2'd2;              // Steady
    localparam [1:0] S_NOPRED = 2'd3;              // No prediction

    // ---- card contents (flops) ----
    reg                  valid_q;   // card in use
    reg  [NL-1:0]        lane_eq_q; // 4: S1 lane l had this card's key
    reg                  wrote_q;   // the card's key was written at the
                                    //   end of the last cycle
    reg  [NL-1:0]        lane_eq;   // 4: S1 compare, next lane_eq_q
    wire [NL-1:0]        key_rows;  // NL = 4: key-match terms, one per
                                    //   lane (the event's lane only)
    reg  [NL-1:0]        lane_pheq_q; // 4: S1 lane l had this card's
                                    //   page hash
    reg  [NL-1:0]        lane_pheq; // 4: S1 compare, next lane_pheq_q
    reg                  ph_ev_q;   // ph_q was written with ev_ph at the
                                    //   end of the last cycle
    reg                  ph_p_q;    // ph_q was written with p_ph then
    wire [NL-1:0]        ph_rows;   // NL = 4: same-page terms, one per
                                    //   lane (the event's lane only)
    wire [OH_N-1:0]      key_cp;    // 6: the S2 event has this card's key
    wire [OH_N-1:0]      own_cp;    // 6: copies of own_hit
    reg                  st_q;      // store instruction
    reg  [TAG_W-1:0]     tag_q;     // TAG_W = 12 bits: PC tag
    reg  [1:0]           state_q;   // trust state
    reg  [STRIDE_W-1:0]  stride_q;  // STRIDE_W = 13 bits: stride
    reg                  nz_q;      // stride_q != 0, kept as a flag (T-8)
    reg  [STRIDE_W-1:0]  exp_q;     // 13 bits: lo + stride (expected next)
    reg  [TGT_W-1:0]     s3_q;      // TGT_W = 15 bits: 3 x stride
    reg  [TGT_W-1:0]     tgt_q;     // 15 bits: lo + 3 x stride
    reg  [PAGE_W-1:0]    lo_q;      // PAGE_W = 12 bits: last offset
    reg  [LO3_W-1:0]     lo3_q;     // LO3_W = 14 bits: 3 x lo
    reg  [PH_W-1:0]      ph_q;      // PH_W = 10 bits: page hash
    // allocated by the event before (write now): AQ_N copies of the flop,
    // each driving one group of view selects (rule T-6)
    wire [AQ_N-1:0]      alloc_cp;  // AQ_N = 5 copies
    wire                 alloc_clr; // reset or clear
    // ---- view: the card as written (alloc_q selects the new contents);
    //      only for the outputs (class, target), the rule reads the flops ----
    wire                 v_valid;   // card in use
    wire [1:0]           v_state;   // trust state
    wire [LIP_W-1:0]     v_tgt_lip; // 6 bits: line-in-page of the target
    // ---- the card E-1 allocated: hit and contents (late select) ----
    wire                 a_hit;     // E hits the card E-1 allocated
    reg  [1:0]           a_state;   // its new state
    reg  [STRIDE_W-1:0]  al_stride; // 13 bits: its next stride
    reg  [STRIDE_W-1:0]  al_exp;    // 13 bits
    reg  [TGT_W-1:0]     al_s3;     // 15 bits
    reg  [TGT_W-1:0]     al_tgt;    // 15 bits
    reg                  al_nz;     // flag
    wire                 own_hit;   // E hits the stored card (no alloc_q)

    // ---- compares (S2) ----
    wire                 same_page; // page hash equal
    wire                 exp_ok;    // event offset is the expected one
    wire                 correct;   // same page and stride kept
    wire                 steady;    // state Steady: the stride is kept
                                    //   (the rule's keep = correct or
                                    //   Steady; see "keep select" below)
    wire                 armed;     // may issue: not No-pred, stride not 0
    wire                 in_page;   // stored target inside the page
    wire                 own_line;  // target is the event's own line
    reg                  filt_main; // target in filter slots 0..2
    wire                 filt_last; // target equals filter slot 3
    wire                 dup_prev;  // target equals E-1's candidate
    wire                 fire;      // this card issues a prefetch

    // ---- operands of the adders ----
    wire [STRIDE_W-1:0]  lo_ext;    // 13 bits: {0, lo}
    wire [STRIDE_W-1:0]  ev_lo_ext; // 13 bits: {0, ev_lo}
    wire [STRIDE_W-1:0]  ev_lo2_ext;// 13 bits: 2 x ev_lo
    wire [TGT_W-1:0]     ev_lo_t;   // 15 bits: ev_lo
    wire [TGT_W-1:0]     ev_lo4_t;  // 15 bits: 4 x ev_lo
    wire [TGT_W-1:0]     ev_lo3_t;  // 15 bits: 3 x ev_lo
    wire [TGT_W-1:0]     lo3_t;     // 15 bits: 3 x lo

    // ---- the three candidate contents: K (keep), N (new stride), Z ----
    wire [STRIDE_W-1:0]  exp_keep;  // 13 bits: ev_lo + stride
    wire [TGT_W-1:0]     tgt_keep;  // 15 bits: ev_lo + 3 x stride
    wire [STRIDE_W-1:0]  delta;     // 13 bits: ev_lo - lo (new stride)
    wire [STRIDE_W-1:0]  exp_new;   // 13 bits: 2 x ev_lo - lo
    wire [TGT_W-1:0]     s3_new;    // 15 bits: 3 x ev_lo - 3 x lo
    wire [TGT_W-1:0]     tgt_new;   // 15 bits: 4 x ev_lo - 3 x lo
    wire                 nz_new;    // delta != 0

    reg  [1:0]           state_next;           // trust state after a hit
    // one-hot write sources of stride / exp / s3 / tgt / nz (the adders
    // then pass one AND-OR level only, rule T-9)
    // per register group (own_cp copy), GRP_N = 4 groups; sel_zero is
    // read only by the groups that restart (1 exp, 3 tgt)
    wire [GRP_N-1:0]     sel_keep;  // own hit, Steady: kept stride
    wire [GRP_N-1:0]     sel_new;   // own hit, same page: new stride
    /* verilator lint_off UNUSEDSIGNAL */
    wire [GRP_N-1:0]     sel_zero;  // own hit, other page: restart
    /* verilator lint_on UNUSEDSIGNAL */
    wire [GRP_N-1:0]     sel_hold;  // no own hit, no allocation: hold
    wire                 sel_alloc; // the card E-1 allocated

    // ======================================================================
    // compares: every operand is a flop (card or event copy)
    // ======================================================================
    // ---- view (allocation of the event before: Initial, stride 0,
    //      exp = tgt = its offset) ----
    assign alloc_clr = rst | clear;
    genvar copy_g; // generate index: alloc_q copy
    generate
        for (copy_g = 0; copy_g < AQ_N; copy_g = copy_g + 1) begin : g_aq
            l1d_pf3_rep #(.W (1), .HAS_RST (1), .HAS_EN (0)) u_alloc_q (
                .clk (clk), .rst (alloc_clr), .en (1'b1),
                .d   (alloc_en), .q (alloc_cp[copy_g])
            );
        end // for copy_g
    endgenerate
    // copy 0: valid, hit; 1: offsets, page; 2: 3 x stride, target; 3:
    // expected, stride; 4: state, flag, store bit, tag. The copies drive
    // only the last 2:1 of each register (alloc_q: the card E-1 allocated,
    // its contents come from l1d_pf3_stride_anew; else the card's own rule
    // on its flops)
    assign v_valid  = valid_q | alloc_cp[0];
    assign v_tgt_lip = alloc_cp[2] ? p_lo[PAGE_W-1:OFFSET_W]
                                   : tgt_q[PAGE_W-1:OFFSET_W];
    assign v_state  = alloc_cp[4] ? S_INIT : state_q;

    assign a_hit     = ev_set_on & a_key;
    // own hit from flops: the key compares were made in S1 against each
    // lane (lane_eq_q); the card's key changes only when an allocation
    // is written (alloc_cp[4], with p_*), so if that happened at the end
    // of the last cycle (wrote_q) the card now holds E-2's key and the
    // parent's per-lane E-2 compare (ev_p2_key) is used instead; the 2:1
    // between them reads flops only, so it is off the late path
    // key match = |(ev_lane & (wrote_q ? ev_p2_key : lane_eq_q)), an OR
    // of NL terms; OH_N kept-hierarchy OR copies so each register
    // group has its own own_hit (the ~200 card flops would otherwise hang
    // on one net through a buffer tree, rule T-6)
    assign key_rows  = ev_lane & (wrote_q ? ev_p2_key : lane_eq_q);
    genvar oh_g; // generate index: own_hit copy
    generate
        for (oh_g = 0; oh_g < OH_N; oh_g = oh_g + 1) begin : g_own
            l1d_pf3_or_rows #(.W (1), .N (NL)) u_key_or (
                .rows   (key_rows),
                .or_out (key_cp[oh_g])
            );
            assign own_cp[oh_g] = ev_set_on & valid_q & key_cp[oh_g];
        end // for oh_g
    endgenerate
    assign own_hit   = own_cp[OH_N-1];
    assign hit       = alloc_cp[0] ? a_hit : own_hit;   // A1: no mask needed
    // same page from flops: the page hash compares were made in S1
    // against each lane (lane_pheq_q); if ph_q was written at the end of
    // the last cycle (ph_ev_q: with the S2 event's hash, ph_p_q: with the
    // event before's), the parent's per-lane compares with those hashes
    // are used instead; the 3:1 reads flops only, off the late path
    assign ph_rows   = ev_lane & (ph_ev_q ? ev_ph_ev
                                          : (ph_p_q ? ev_ph_p : lane_pheq_q));
    assign same_page = |ph_rows;                        // = (ph_q == ev_ph)
    assign lo_ext    = {1'b0, lo_q};
    assign ev_lo_ext = {1'b0, ev_lo};
    assign exp_ok    = (exp_q == ev_lo_ext);            // lo + stride == ev_lo
    assign correct   = same_page & exp_ok;
    assign steady    = (state_q == S_STEADY);
    assign armed     = (state_q != S_NOPRED) & nz_q;
    assign tgt_lip   = v_tgt_lip;                       // bits [11:6]
    // fire reads the stored target: a card E-1 allocated is not armed
    // (stride 0), so it never fires (fire needs ~alloc_q)
    assign in_page   = (tgt_q[TGT_W-1:PAGE_W] == {(TGT_W-PAGE_W){1'b0}});
    assign own_line  = (tgt_q[PAGE_W-1:OFFSET_W] == ev_lo[PAGE_W-1:OFFSET_W]);

    // filter: the target must not be one of the last keys sent; slots 0..2
    // are always inside the window of this event, slot 3 only when the
    // event before sent nothing, the event before's key only when it sent
    always @* begin : filter_proc
        integer slot_idx; // filter slot, 0 = newest
        filt_main = 1'b0;
        for (slot_idx = 0; slot_idx < FILT_DEPTH - 1; slot_idx = slot_idx + 1) begin
            // slot valid, same page hash and same line-in-page -> duplicate
            if (filt_pm[slot_idx]
                && (filt_lip[slot_idx*LIP_W +: LIP_W] == tgt_q[PAGE_W-1:OFFSET_W])) begin
                filt_main = 1'b1;
            end
        end // for slot_idx
    end // always filter_proc
    assign filt_last = filt_pm[FILT_DEPTH-1]
                       & (filt_lip[(FILT_DEPTH-1)*LIP_W +: LIP_W] == tgt_q[PAGE_W-1:OFFSET_W]);
    assign dup_prev  = prev_pm & (prev_lip == tgt_q[PAGE_W-1:OFFSET_W]);

    // fire: the event is ours, keeps the stride, the card may issue and the
    // stored target is a new line inside the page
    assign fire     = ~alloc_cp[0] & own_hit & correct & armed & in_page
                      & ~own_line & ~filt_main;
    assign fire_new = fire & ~filt_last;
    assign fire_dup = fire & ~dup_prev;

    // ======================================================================
    // adders (parallel with the compares, all operands are flops)
    // ======================================================================
    assign ev_lo2_ext = {ev_lo, 1'b0};
    assign ev_lo_t    = {{(TGT_W-PAGE_W){1'b0}}, ev_lo};
    assign ev_lo4_t   = {1'b0, ev_lo, 2'b00};
    assign ev_lo3_t   = {1'b0, ev_lo3};
    assign lo3_t      = {1'b0, lo3_q};
    assign nz_new     = (ev_lo != lo_q);                // delta != 0

    l1d_pf3_ks_add #(.W(STRIDE_W), .SUB(0)) u_exp_keep (
        .a (ev_lo_ext), .b (stride_q), .sum (exp_keep)   // ev_lo + stride
    );
    l1d_pf3_ks_add #(.W(TGT_W), .SUB(0)) u_tgt_keep (
        .a (ev_lo_t), .b (s3_q), .sum (tgt_keep)         // ev_lo + 3 stride
    );
    l1d_pf3_ks_add #(.W(STRIDE_W), .SUB(1)) u_delta (
        .a (ev_lo_ext), .b (lo_ext), .sum (delta)        // ev_lo - lo
    );
    l1d_pf3_ks_add #(.W(STRIDE_W), .SUB(1)) u_exp_new (
        .a (ev_lo2_ext), .b (lo_ext), .sum (exp_new)     // 2 ev_lo - lo
    );
    l1d_pf3_ks_add #(.W(TGT_W), .SUB(1)) u_s3_new (
        .a (ev_lo3_t), .b (lo3_t), .sum (s3_new)         // 3 ev_lo - 3 lo
    );
    l1d_pf3_ks_add #(.W(TGT_W), .SUB(1)) u_tgt_new (
        .a (ev_lo4_t), .b (lo3_t), .sum (tgt_new)        // 4 ev_lo - 3 lo
    );

    // ======================================================================
    // next values (hit, allocation or hold) for both keep outcomes
    // ======================================================================
    always @* begin : next_proc
        // trust state after a hit (Chen and Baer table)
        case (state_q)
            S_INIT:   state_next = correct ? S_STEADY : S_TRANS;
            S_TRANS:  state_next = correct ? S_STEADY : S_NOPRED;
            S_STEADY: state_next = correct ? S_STEADY : S_INIT;
            default:  state_next = correct ? S_TRANS  : S_NOPRED;
        endcase
        // the card E-1 allocated (Initial, stride 0, exp = tgt = p_lo):
        // the shared new contents if E hits it, else E-1's values
        a_state = a_correct ? S_STEADY : S_TRANS;
        if (a_hit) begin
            al_stride = a_stride;
            al_exp    = a_exp;
            al_s3     = a_s3;
            al_tgt    = a_tgt;
            al_nz     = a_nz;
        end
        else begin
            al_stride = {STRIDE_W{1'b0}};
            al_exp    = {1'b0, p_lo};
            al_s3     = {TGT_W{1'b0}};
            al_tgt    = {{(TGT_W-PAGE_W){1'b0}}, p_lo};
            al_nz     = 1'b0;
        end // a_hit
    end // always next_proc

    // the write sources, one-hot (alloc_cp copies as before: 2 for the
    // target group, 3 for stride / exp, 4 for the flag; all copies equal)
    // (one set per own_hit copy / register group)
    assign sel_alloc = alloc_cp[3];
    assign sel_hold  = {GRP_N{~alloc_cp[3]}} & ~own_cp[GRP_N-1:0];
    assign sel_keep  = {GRP_N{~alloc_cp[3] & steady}} & own_cp[GRP_N-1:0];
    assign sel_new   = {GRP_N{~alloc_cp[3] & ~steady & same_page}}
                       & own_cp[GRP_N-1:0];
    assign sel_zero  = {GRP_N{~alloc_cp[3] & ~steady & ~same_page}}
                       & own_cp[GRP_N-1:0];

    // ======================================================================
    // card registers
    // ======================================================================
    always @(posedge clk) begin : card_ctl_proc
        if (rst || clear) begin
            valid_q <= 1'b0;                 // every card free
            state_q <= S_INIT;
        end
        else begin
            valid_q <= v_valid;              // allocation of E-1 lands
            if (alloc_cp[4]) begin           // trust update
                state_q <= a_hit ? a_state : S_INIT;
            end
            else if (own_cp[4]) begin
                state_q <= state_next;
            end
        end // rst / run
    end // always card_ctl_proc

    always @(posedge clk) begin : card_data_proc
        // stride, exp, 3 x stride, target, flag. Keep select: the rule
        // keeps the stride when correct or Steady. When correct, the new
        // stride values equal the kept ones (ev_lo = lo + stride, so
        // delta = stride, 2 ev_lo - lo = ev_lo + stride, 3 delta = s3,
        // 4 ev_lo - 3 lo = ev_lo + s3, delta != 0 = nz), because the card
        // always holds exp = lo + stride, s3 = 3 stride, tgt = lo + s3 and
        // nz = (stride != 0) (checked under L1D_DEBUG). So "Steady" alone
        // picks kept or new: a flop select instead of the 13-bit compare
        // (rule T-9).
        // allocated card: al_*; own hit and Steady: kept stride (ev_lo +
        // stride, ev_lo + s3); own hit, same page: new stride; own hit,
        // other page: restart at ev_lo; else hold. Written as one AND-OR of
        // the five one-hot sources
        stride_q <= ({STRIDE_W{sel_keep[0] | sel_hold[0]}} & stride_q)
                  | ({STRIDE_W{sel_new[0]}}   & delta)
                  | ({STRIDE_W{sel_alloc}}    & al_stride);
        exp_q    <= ({STRIDE_W{sel_keep[1]}}  & exp_keep)
                  | ({STRIDE_W{sel_new[1]}}   & exp_new)
                  | ({STRIDE_W{sel_zero[1]}}  & ev_lo_ext)
                  | ({STRIDE_W{sel_hold[1]}}  & exp_q)
                  | ({STRIDE_W{sel_alloc}}    & al_exp);
        s3_q     <= ({TGT_W{sel_keep[2] | sel_hold[2]}} & s3_q)
                  | ({TGT_W{sel_new[2]}}   & s3_new)
                  | ({TGT_W{sel_alloc}}    & al_s3);
        tgt_q    <= ({TGT_W{sel_keep[3]}}  & tgt_keep)
                  | ({TGT_W{sel_new[3]}}   & tgt_new)
                  | ({TGT_W{sel_zero[3]}}  & ev_lo_t)
                  | ({TGT_W{sel_hold[3]}}  & tgt_q)
                  | ({TGT_W{sel_alloc}}    & al_tgt);
        nz_q     <= ((sel_keep[0] | sel_hold[0]) & nz_q)
                  | (sel_new[0] & nz_new)
                  | (sel_alloc & al_nz);
        // offsets and page hash follow every event of the card; alloc_q
        // writes the event before (or this event if it hits that card)
        if (alloc_cp[1]) begin
            lo_q  <= a_hit ? ev_lo  : p_lo;
            lo3_q <= a_hit ? ev_lo3 : p_lo3;
            ph_q  <= a_hit ? ev_ph  : p_ph;
        end
        else if (own_cp[4]) begin
            lo_q  <= ev_lo;
            lo3_q <= ev_lo3;
            ph_q  <= ev_ph;
        end
        if (alloc_cp[4]) begin
            st_q  <= p_st;
            tag_q <= p_tag;
        end
        lane_eq_q <= lane_eq;   // no reset: ev_set_on and valid_q guard it
    end // always card_data_proc

    // S1: each lane's page hash against the card's (unless written,
    // ph_ev_q / ph_p_q, these are the flops S2 compares with)
    always @* begin : s1_ph_proc
        integer lane_idx; // walks the lanes
        for (lane_idx = 0; lane_idx < NL; lane_idx = lane_idx + 1) begin
            lane_pheq[lane_idx] = (s1_ph[lane_idx*PH_W +: PH_W] == ph_q);
        end // for lane_idx
    end // always s1_ph_proc

    // which value ph_q was written with (same conditions as the write)
    always @(posedge clk) begin : ph_wrote_proc
        if (rst) begin
            ph_ev_q <= 1'b0;
            ph_p_q  <= 1'b0;
        end
        else begin
            ph_ev_q <= alloc_cp[1] ? a_hit : own_cp[4];
            ph_p_q  <= alloc_cp[1] & ~a_hit;
        end // rst
        lane_pheq_q <= lane_pheq; // no reset: ev_lane and own_cp guard it
    end // always ph_wrote_proc

    // S1: each lane's key against the card's key (the key flops of the
    // last cycle are the ones S2 compares with, unless written, wrote_q)
    always @* begin : s1_key_proc
        integer lane_idx; // walks the lanes
        for (lane_idx = 0; lane_idx < NL; lane_idx = lane_idx + 1) begin
            lane_eq[lane_idx] = (s1_tag[lane_idx*TAG_W +: TAG_W] == tag_q)
                                & (s1_st[lane_idx] == st_q);
        end // for lane_idx
    end // always s1_key_proc

    always @(posedge clk) begin : wrote_proc
        if (rst) begin
            wrote_q <= 1'b0;
        end
        else begin
            wrote_q <= alloc_cp[4];
        end // rst
    end // always wrote_proc

    // ---- replacement class (lowest is replaced first) ----
    always @* begin : class_proc
        if (!v_valid) begin
            class_rank = 3'd0;               // free card
        end
        else begin
            case (v_state)
                S_NOPRED: class_rank = 3'd1;
                S_INIT:   class_rank = 3'd2;
                S_TRANS:  class_rank = 3'd3;
                default:  class_rank = 3'd4; // Steady
            endcase
        end // valid_q
    end // always class_proc

`ifdef L1D_DEBUG
    // ---- the card's invariants the keep select relies on (V-9.3) ----
    wire [TGT_W-1:0] dbg_stride_t = {{(TGT_W-STRIDE_W){stride_q[STRIDE_W-1]}},
                                     stride_q};                  // 15 bits
    always @(posedge clk) begin : debug_proc
        if (!rst && valid_q && !alloc_cp[0]) begin
            if ((exp_q != ({1'b0, lo_q} + stride_q))
                || (s3_q != (dbg_stride_t + {dbg_stride_t[TGT_W-2:0], 1'b0}))
                || (tgt_q != ({{(TGT_W-PAGE_W){1'b0}}, lo_q} + s3_q))
                || (nz_q != (stride_q != {STRIDE_W{1'b0}}))) begin
                $display("[%0t] L1D PF3_STRIDE_ENTRY ERROR card invariant", $time);
            end
        end
        // the S1-compared hit equals the direct S2 compare (V-9.3)
        if (!rst && (own_hit != (ev_set_on && valid_q && (tag_q == ev_tag)
                                 && (st_q == ev_st)))) begin
            $display("[%0t] L1D PF3_STRIDE_ENTRY ERROR own hit != key compare",
                     $time);
        end
        // the S1-compared page hash equals the direct S2 compare when the
        // event is in this set (V-9.3)
        if (!rst && ev_set_on && (same_page != (ph_q == ev_ph))) begin
            $display("[%0t] L1D PF3_STRIDE_ENTRY ERROR same page != ph compare",
                     $time);
        end
    end // always debug_proc
`endif

endmodule

`default_nettype wire
`endif // L1D_PF3_STRIDE_ENTRY_V
