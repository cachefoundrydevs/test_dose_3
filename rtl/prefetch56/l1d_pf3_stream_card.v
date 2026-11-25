// ==========================================================================
// l1d_pf3_stream_card.v - one card of the stream table, with its own
//                         compare (S1) and update (S2) logic
// ==========================================================================
// IN SIMPLE WORDS : one "note card" of the stream prefetcher: one 4 KB
//   page the program walks line after line. The card keeps the page, the
//   last line seen, the direction, a confidence counter, how far ahead
//   requests were already made (ahead) and the line to reach (target).
//   The card compares the new training event with itself (S1), applies
//   the step rule and stores the result (S2), and offers the first
//   request of a newly confirmed stream (direct candidate) in S2, so
//   that request reaches the unit's output three cycles after training.
//   Later requests of the card come from the scan of l1d_pf3_stream.
//   A card chosen to be allocated only notes it (alloc_q, a flop kept in 3 copies) and
//   is written in the next cycle from the parent's flops of that event
//   (p_*); meanwhile every read of the card goes through a "view" that
//   already shows the new contents (2:1 select by alloc_q), so the late
//   allocation decision drives one flop instead of the card's data flops.
//
// STAGE / CYCLE   : S1 (compare against the lane records, registered),
//   S2 (step rule, write, direct candidate, scan values; from flops).
// INSTANTIATED BY : l1d_pf3_stream, generate g_card, STR_ENTRIES = 16
//   copies, index passed as CARD_ID.
//
// PARAMETERS:
//   name        default unit    meaning
//   NB          4       lanes   training lanes (banks)
//   TAG_W       12      bits    hashed page tag
//   PAGE_NUM_W  44      bits    page number
//   LIP_W       6       bits    line in page
//   CONF_W      2       bits    confidence counter
//   CONF_ON     2       count   confidence at which requests start
//   CONF_MAX    3       count   confidence saturation
//   RECENT_N    4       keys    duplicate window: the stream's last sent
//                               keys
//
// PORTS (default widths; "event" = training record; S1 = lanes from the
//   unit pins, S2 = the event register e1_* of the parent):
//   clk, rst, flush      in  1     clock, reset, flush (prefetch off)
//   lane_pick            in  4     S1: the lane the parent picks (one-hot)
//   lane_tag, lane_line  in  4x12, 4x6  S1: each lane's page tag, line
//   lane_line_p1         in  4x7   S1: each lane's line + 1 (7 bits)
//   ev_vld               in  1     S2: an event is in S2
//   ev_page              in  44    S2: the event's page
//   ev_line              in  6     S2: its line in the page
//   ev_line_p1, _m1      in  6, 6  S2: line + 1, line - 1
//   ev_top, ev_bot       in  1, 1  S2: line is 63 / line is 0
//   ev_tgt_up, _dn       in  6, 6  S2: line + DEPTH / - DEPTH, clamped
//   ev_trig              in  1     S2: miss or prefetched hit
//   prev_card            in  1     S2: E-1 wrote this card (flop)
//   prev_same_tag        in  1     S2: E's tag == E-1's tag (flop)
//   prev_same, _up, _dn  in  1     S2: E's line vs E-1's line (flops)
//   alloc_en             in  1     S2: the event allocates this card
//   p_tag, p_page        in  12,44 the event before (written if alloc_q)
//   p_line, p_line_p1w   in  6, 7  its line and line + 1
//   p_tgt_dn, p_store    in  6, 1  its down target and store bit
//   scan_idle            in  1     S2: no card has lines left (a direct
//                                  candidate may be made)
//   scan_sel             in  1     S2: the scan picked this card (flop)
//   slot_go              in  1     S2: the offer place is free, prefetch
//                                  on (a candidate made now is used)
//   recent_*             in        S2: the key window, 4 keys (vld, tag,
//                                  tag == event tag, line + 1, line - 1)
//   ev_dup_p1, _m1       in  1, 1  S2: event line +/- 1 is in the window
//   hit                  out 1     S2: the event is this card's
//   pend_out             out 1     the card has lines left to ask
//   class_out            out 5     victim class, one-hot (a flop): bit 0
//                                  = free, bit k = valid with confidence
//                                  k - 1 (NCLS = CONF_MAX + 2 classes)
//   dir_vld, dir_evp,    out       S2: direct candidate of this card: valid,
//   dir_up, dir_kp,                from the event line (and up / down) or
//   dir_kp_p1, _m1,                the kept ahead line (+1, -1), duplicate,
//   dir_dup, dir_store             store stream
//   scan_go, scan_line,  out       S2: scan values of this card: a line to
//   scan_lp1, _lm1,                ask, the line, +1, -1, duplicate
//   scan_dup
//   page_out, tag_out,   out 44,12,1  the card's page, tag, store bit
//   store_out
//
// STEP RULE (same as l1d_pf_stream, README section 5):
//   same line     -> card unchanged
//   +1 / -1 in the direction (or confidence 0) -> confidence + 1,
//                    ahead = line if confidence was < CONF_ON or the
//                    program passed it, target = line +/- DEPTH
//   +1 / -1 against the direction -> confidence 0, direction flips
//   any other jump -> confidence 0, direction unset, ahead = line
//   pending = confidence >= CONF_ON and (miss or prefetched hit or it
//             was pending already)
//
// WORKED EXAMPLE (DEPTH 4, lines 50, 51, 52, all misses):
//   50: allocate (conf 0). 51: step up, conf 1, ahead 51. 52: step up,
//   conf 2 = CONF_ON, ahead 52, target 56, pending: the direct
//   candidate asks line 53 in S2 and ahead becomes 53; the scan asks
//   54, 55, 56 later and clears pending.
//
// LOGIC LEVELS (T-9, estimate): S1 compare 4 (+ lane select 2 in the
//   register mux, + view select 1); S2: ahead compares 4 from flops (in
//   parallel: behind, done, key +/- 1 duplicates), compare select 1, rule
//   3, line / duplicate select 2, write select 1 = about 10; alloc_en ->
//   alloc_q (1 flop).
//
// ASSUMPTIONS (V-9.2): A1 alloc_en and hit are never 1 together (a hit
//   on a card with alloc_q updates the view). A2 the
//   training update wins over the scan write in the same cycle (as the
//   5-stage RTL: "scan side loses to S1"). A3 CONF_ON >= 1 (an allocated
//   card, confidence 0, then never makes a request in its first S2, so
//   the request paths read the stored ahead / target / tag flops, not the
//   view).
// ==========================================================================
`ifndef L1D_PF3_STREAM_CARD_V
`define L1D_PF3_STREAM_CARD_V
`default_nettype none

module l1d_pf3_stream_card #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter NB         = 4,  // default 4 lanes
    parameter TAG_W      = 12, // default 12 bits: page tag
    parameter PAGE_NUM_W = 44, // default 44 bits: page number
    parameter LIP_W      = 6,  // default 6 bits: line in page
    parameter CONF_W     = 2,  // default 2 bits: confidence
    parameter CONF_ON    = 2,  // default 2: confidence to start
    parameter CONF_MAX   = 3,  // default 3: confidence maximum
    parameter RECENT_N   = 4,  // default 4 keys: duplicate window
    parameter DEPTH      = 4   // default 4 lines: lead kept ahead
) (
    // ---- clock and reset ----
    input  wire                      clk,           // clock
    input  wire                      rst,           // sync reset, high
    input  wire                      flush,         // prefetch off: clear pend
    // ---- S1: the lane records ----
    input  wire [NB-1:0]             lane_pick,     // 4: chosen lane
    input  wire [NB*TAG_W-1:0]       lane_tag,      // 4 x 12
    input  wire [NB*LIP_W-1:0]       lane_line,     // 4 x 6
    input  wire [NB*(LIP_W+1)-1:0]   lane_line_p1,  // 4 x 7, line + 1
    // ---- S2: the event ----
    input  wire                      ev_vld,        // event in S2
    input  wire [PAGE_NUM_W-1:0]     ev_page,       // 44 bits
    input  wire [LIP_W-1:0]          ev_line,       // 6 bits
    input  wire [LIP_W-1:0]          ev_line_p1,    // 6 bits, line + 1
    input  wire [LIP_W-1:0]          ev_line_m1,    // 6 bits, line - 1
    input  wire                      ev_top,        // line is the last
    input  wire                      ev_bot,        // line is 0
    input  wire [LIP_W-1:0]          ev_tgt_up,     // 6 bits
    input  wire [LIP_W-1:0]          ev_tgt_dn,     // 6 bits
    input  wire                      ev_trig,       // miss or pf hit
    input  wire                      prev_card,     // E-1 wrote this card
    input  wire                      prev_same_tag, // E tag == E-1 tag
    input  wire                      prev_same,     // E line == E-1 line
    input  wire                      prev_up,       // E line == E-1 + 1
    input  wire                      prev_dn,       // E line == E-1 - 1
    input  wire                      alloc_en,      // allocate this card
    input  wire [TAG_W-1:0]          p_tag,         // 12 bits: event before
    input  wire [PAGE_NUM_W-1:0]     p_page,        // 44 bits
    input  wire [LIP_W-1:0]          p_line,        // 6 bits
    input  wire [LIP_W:0]            p_line_p1w,    // 7 bits, line + 1
    input  wire [LIP_W-1:0]          p_tgt_dn,      // 6 bits
    input  wire                      p_store,       // store access
    input  wire                      scan_idle,     // no scan card picked
                                                    //   (flop): direct allowed
    input  wire                      scan_sel,      // scan picked this card
    input  wire                      slot_go,       // the offer place is free
                                                    //   (late, S3 flags): the
                                                    //   scan line goes, a
                                                    //   direct line may go
    input  wire [RECENT_N-1:0]       recent_tag_eq, // 4: recent tag == ev_tag
    input  wire                      ev_dup_p1,     // event line + 1 was sent
    input  wire                      ev_dup_m1,     // event line - 1 was sent
    input  wire [RECENT_N*LIP_W-1:0] recent_lp1,    // 4 x 6: key line + 1
    input  wire [RECENT_N*LIP_W-1:0] recent_lm1,    // 4 x 6: key line - 1
    input  wire [RECENT_N*TAG_W-1:0] recent_tag,    // 4 x 12
    input  wire [RECENT_N-1:0]       recent_vld,    // 4
    // ---- outputs ----
    output wire                      hit,           // S2: event's card
    output wire                      pend_out,      // lines left to ask
    output wire [CONF_MAX+1:0]       class_out,     // NCLS = 5: victim class
    output wire                      dir_vld,       // S2: direct candidate
    output wire                      dir_evp,       // S2: line from the event
                                                    //     line (else kept ahead)
    output wire                      dir_up,        // S2: the event steps up
    output wire [LIP_W-1:0]          dir_kp,        // S2: kept: ahead +/- 1
    output wire [LIP_W-1:0]          dir_kp_p1,     // S2: kept: that + 1
    output wire [LIP_W-1:0]          dir_kp_m1,     // S2: kept: that - 1
    output wire                      dir_dup,       // S2: it is a duplicate
    output wire                      dir_store,     // S2: store stream
    output wire                      scan_go,       // S2: scan: a line to ask
    output wire [LIP_W-1:0]          scan_line,     // S2: scan: the line
    output wire [LIP_W-1:0]          scan_lp1,      // S2: scan: line + 1
    output wire [LIP_W-1:0]          scan_lm1,      // S2: scan: line - 1
    output wire                      scan_dup,      // S2: scan: duplicate
    output wire [PAGE_NUM_W-1:0]     page_out,      // card's page
    output wire [TAG_W-1:0]          tag_out,       // card's tag
    output wire                      store_out      // card's store bit
);

    // ---- local constants (value at the default) ----
    localparam [CONF_W-1:0] CONF_ON_V  = CONF_ON;   // 2
    localparam [CONF_W-1:0] CONF_MAX_V = CONF_MAX;  // 3
    localparam [CONF_W-1:0] CONF_ONE   = 1;         // 1
    localparam [LIP_W-1:0]  LIP_ONE    = 1;         // 1
    localparam [LIP_W-1:0]  LIP_TWO    = 2;         // 2
    localparam AQ_N = 3;                            // 3 copies of alloc_q
    localparam NCLS = CONF_MAX + 2;                 // 5 victim classes
    localparam [NCLS-1:0] CLASS_FREE = 1;           // 5 bits: free card
    localparam [NCLS-1:0] CLASS_NEW  = 2;           // 5 bits: valid, conf 0
    localparam WIDE_W = LIP_W + 2;                  // 8 bits: no wrap
    localparam [WIDE_W-1:0] STEP_LEAD = DEPTH + 1;  // 5: target of last +/- 1
    localparam [WIDE_W-1:0] LAST_WIDE = (1 << LIP_W) - 1; // 63
    localparam [LIP_W-1:0]  LAST_LINE = (1 << LIP_W) - 1; // 63

    // ---- the card (flops) ----
    reg                   valid_q;   // card in use
    reg  [TAG_W-1:0]      tag_q;     // 12 bits, hashed page
    reg  [PAGE_NUM_W-1:0] page_q;    // 44 bits, latest page number
    reg  [LIP_W-1:0]      last_q;    // 6 bits, latest line
    reg  [LIP_W:0]        last_p1_q; // 7 bits, latest line + 1
    reg                   dir_q;     // 1 = up, 0 = down
    reg  [CONF_W-1:0]     conf_q;    // 2 bits, confidence
    // the victim class of the card's next state, kept as a flop so the
    // parent's victim pick starts from flops (not from the view)
    reg  [NCLS-1:0]       class_q;   // 5: class, one-hot
    reg  [NCLS-1:0]       class_nx;  // 5: class of the next state
    reg  [CONF_W-1:0]     conf_nx;   // 2 bits: next confidence
    reg  [CONF_W-1:0]     class_conf;// scratch: confidence of class k
    reg  [LIP_W-1:0]      ahead_q;   // 6 bits, furthest line asked
    reg  [LIP_W-1:0]      tgt_q;     // 6 bits, line to reach
    // the targets a step from last_q would set (up: line last + 1, down:
    // line last - 1), written with last_q, so the kept-ahead done check
    // compares two card flops (rule T-9)
    reg  [LIP_W-1:0]      tup_q;     // 6 bits: min(last + 1 + DEPTH, 63)
    reg  [LIP_W-1:0]      tdn_q;     // 6 bits: max(last - 1 - DEPTH, 0)
    reg  [LIP_W-1:0]      ev_tup;    // 6 bits: tup of the event line
    reg  [LIP_W-1:0]      ev_tdn;    // 6 bits: tdn of the event line
    reg  [LIP_W-1:0]      p_tup;     // 6 bits: tup of the line before
    reg  [LIP_W-1:0]      p_tdn;     // 6 bits: tdn of the line before
    wire [LIP_W-1:0]      v_tup;     // 6 bits: view of tup_q
    wire [LIP_W-1:0]      v_tdn;     // 6 bits: view of tdn_q
    reg                   store_q;   // store stream
    reg                   pend_q;    // lines still to ask
    // allocated by the event before: AQ_N copies (rule T-6)
    wire [AQ_N-1:0]       alloc_cp;  // AQ_N = 3 copies
    wire                  alloc_d;   // next value: allocate, not while off
    // ---- view: the card as written (alloc_q: the new contents) ----
    wire                  v_valid;   // card in use
    wire [TAG_W-1:0]      v_tag;     // 12 bits
    wire [PAGE_NUM_W-1:0] v_page;    // 44 bits
    wire [LIP_W-1:0]      v_last;    // 6 bits
    wire [LIP_W:0]        v_last_p1; // 7 bits
    wire                  v_dir;     // direction
    wire [CONF_W-1:0]     v_conf;    // 2 bits
    wire [LIP_W-1:0]      v_ahead;   // 6 bits
    wire [LIP_W-1:0]      v_tgt;     // 6 bits
    wire                  v_store;   // store stream
    wire                  v_pend;    // lines still to ask

    // ---- S1 compare results (vs the picked lane), registered ----
    reg                   s1_hit_q;  // tag equal
    reg                   s1_same_q; // same line as last
    reg                   s1_up_q;   // last + 1
    reg                   s1_dn_q;   // last - 1
    reg  [NB-1:0]         lane_hit;  // 4: per lane tag equal
    reg  [NB-1:0]         lane_same; // 4: per lane same line
    reg  [NB-1:0]         lane_up;   // 4: per lane last + 1
    reg  [NB-1:0]         lane_dn;   // 4: per lane last - 1

    // ---- S2 ----
    wire                  c_same;       // same line (selected source)
    wire                  c_up;         // step up
    wire                  c_dn;         // step down
    wire                  behind_up;    // ahead < line
    wire                  behind_dn;    // ahead > line
    wire                  conf_unset;   // confidence 0
    wire                  step_up;      // counts as a step up
    wire                  step_dn;      // counts as a step down
    wire                  step_ok;      // step in the direction
    wire                  flip_ok;      // step against the direction
    wire                  ahead_behind; // program passed the ahead line
    reg  [CONF_W-1:0]     new_conf;     // 2 bits
    reg                   new_dir;      // direction after
    reg  [LIP_W-1:0]      new_tgt;      // 6 bits
    wire                  new_pend;     // pending after the rule
    wire [LIP_W-1:0]      ahead_p1;     // 6 bits, ahead + 1
    wire [LIP_W-1:0]      ahead_m1;     // 6 bits, ahead - 1
    wire [LIP_W-1:0]      ahead_p2;     // 6 bits, ahead + 2
    wire [LIP_W-1:0]      ahead_m2;     // 6 bits, ahead - 2
    // the direct line from the event line (ev_*) or from the kept ahead
    // (kp_*), both ready before ev_path picks one (rule T-9)
    wire [LIP_W-1:0]      ev_next;      // 6 bits: event line +/- 1
    wire                  ev_done;      // it leaves the page
    wire                  ev_dup;       // it was sent
    wire                  kept_dir;     // direction when ahead is kept
    wire [LIP_W-1:0]      kp_next;      // 6 bits: ahead +/- 1
    wire                  kp_done;      // ahead reached the target
    wire                  kp_dup;       // it was sent
    wire [LIP_W-1:0]      kp_lp1;       // 6 bits: kp_next + 1
    wire [LIP_W-1:0]      kp_lm1;       // 6 bits: kp_next - 1
    wire                  pend_go;      // hit, pending after, scan idle
    wire                  mk_ev;        // direct line made if slot_go, event
    wire                  mk_kp;        // direct line made if slot_go, kept
    wire                  d_make;       // made if slot_go
    wire [LIP_W:0]        go_pick;      // 7 bits: {pending, ahead} picked
                                        //   by the late ev_hit
    wire                  ev_hit;       // hit and ahead from the event line
    wire [LIP_W-1:0]      go_ev;        // 6 bits: next ahead if ev_hit
    wire                  pend_ev;      // next pending if ev_hit
    wire [LIP_W-1:0]      go_oth;       // 6 bits: next ahead if not ev_hit
    wire                  pend_oth;     // next pending if not ev_hit
    wire [LIP_W-1:0]      oth_go;       // 6 bits: not ev_hit, slot_go
    wire                  poth_go;      // pending: not ev_hit, slot_go
    wire                  poth_stop;    // pending: not ev_hit, no slot_go
    wire                  cur_done;     // scan: ahead reached the target
    wire                  scan_done;    // S2: scan: card finished (this cycle's line is the last)
    wire [LIP_W-1:0]      cur_next;     // scan: next line
    wire                  ev_path;      // direct line from the event line
    wire                  kept_done_up; // ahead kept, up: target reached
    wire                  kept_done_dn; // ahead kept, down: target reached
    reg                   ah_dup_p1;    // {event tag, ahead + 1} was sent
    reg                   ah_dup_m1;    // {event tag, ahead - 1} was sent
    reg                   own_dup_p1;   // {card tag, ahead + 1} was sent
    reg                   own_dup_m1;   // {card tag, ahead - 1} was sent
    reg                   s_dup_r;      // scan candidate duplicate

    // ======================================================================
    // S1: compare the lanes with this card, keep the picked lane's result
    // ======================================================================
    // ---- view: allocation of the event before (conf 0, down, ahead =
    //      its line, target = its down target) ----
    assign alloc_d = alloc_en & ~flush;
    genvar copy_g; // generate index: alloc_q copy
    generate
        for (copy_g = 0; copy_g < AQ_N; copy_g = copy_g + 1) begin : g_aq
            l1d_pf3_rep #(.W (1), .HAS_RST (1), .HAS_EN (0)) u_alloc_q (
                .clk (clk), .rst (rst), .en (1'b1),
                .d   (alloc_d), .q (alloc_cp[copy_g])
            );
        end // for copy_g
    endgenerate
    // copy 0: identity and last line (S1 compares, hit), 1: step rule
    // state, 2: page, store bit, pending
    assign v_valid   = valid_q | alloc_cp[0];
    assign v_tag     = alloc_cp[0] ? p_tag      : tag_q;
    assign v_page    = alloc_cp[2] ? p_page     : page_q;
    assign v_last    = alloc_cp[0] ? p_line     : last_q;
    assign v_last_p1 = alloc_cp[0] ? p_line_p1w : last_p1_q;
    assign v_dir     = alloc_cp[1] ? 1'b0       : dir_q;
    assign v_conf    = alloc_cp[1] ? {CONF_W{1'b0}} : conf_q;
    assign v_ahead   = alloc_cp[1] ? p_line     : ahead_q;
    assign v_tgt     = alloc_cp[1] ? p_tgt_dn   : tgt_q;
    assign v_store   = alloc_cp[2] ? p_store    : store_q;
    assign v_pend    = alloc_cp[2] ? 1'b0       : pend_q;
    assign v_tup     = alloc_cp[1] ? p_tup      : tup_q;
    assign v_tdn     = alloc_cp[1] ? p_tdn      : tdn_q;

    // targets of a step from a line x: up min(x + 1 + DEPTH, 63), down
    // max(x - 1 - DEPTH, 0) (the parent's ev_tgt_* of line x +/- 1)
    always @* begin : step_tgt_proc
        reg [WIDE_W-1:0] ev_wide; // 8 bits: event line, no wrap
        reg [WIDE_W-1:0] p_wide;  // 8 bits: line before, no wrap
        reg [WIDE_W-1:0] ev_sum;  // 8 bits: event line + DEPTH + 1
        reg [WIDE_W-1:0] p_sum;   // 8 bits: line before + DEPTH + 1
        reg [LIP_W-1:0]  ev_dif;  // 6 bits: event line - DEPTH - 1
        reg [LIP_W-1:0]  p_dif;   // 6 bits: line before - DEPTH - 1
        ev_wide = {{(WIDE_W-LIP_W){1'b0}}, ev_line};
        p_wide  = {{(WIDE_W-LIP_W){1'b0}}, p_line};
        ev_sum  = ev_wide + STEP_LEAD;
        p_sum   = p_wide + STEP_LEAD;
        ev_dif  = ev_line - STEP_LEAD[LIP_W-1:0];
        p_dif   = p_line - STEP_LEAD[LIP_W-1:0];
        ev_tup  = (ev_sum > LAST_WIDE) ? LAST_LINE : ev_sum[LIP_W-1:0];
        p_tup   = (p_sum > LAST_WIDE) ? LAST_LINE : p_sum[LIP_W-1:0];
        ev_tdn  = (ev_wide >= STEP_LEAD) ? ev_dif : {LIP_W{1'b0}};
        p_tdn   = (p_wide >= STEP_LEAD) ? p_dif : {LIP_W{1'b0}};
    end // always step_tgt_proc

    always @* begin : s1_cmp_proc
        integer lane_idx; // walks the lanes
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_hit[lane_idx]  = v_valid
                & (v_tag == lane_tag[lane_idx*TAG_W +: TAG_W]);
            lane_same[lane_idx] = (v_last == lane_line[lane_idx*LIP_W +: LIP_W]);
            lane_up[lane_idx]   = ({1'b0, lane_line[lane_idx*LIP_W +: LIP_W]}
                                   == v_last_p1);
            lane_dn[lane_idx]   = (lane_line_p1[lane_idx*(LIP_W+1) +: LIP_W+1]
                                   == {1'b0, v_last});
        end // for lane_idx
    end // always s1_cmp_proc

    always @(posedge clk) begin : s1_reg_proc
        s1_hit_q  <= |(lane_pick & lane_hit);
        s1_same_q <= |(lane_pick & lane_same);
        s1_up_q   <= |(lane_pick & lane_up);
        s1_dn_q   <= |(lane_pick & lane_dn);
    end // always s1_reg_proc

    // ======================================================================
    // S2: the compare bits (E-1 wrote this card: use the event compares)
    // ======================================================================
    assign hit    = ev_vld & (prev_card ? prev_same_tag : s1_hit_q);
    assign c_same = prev_card ? prev_same : s1_same_q;
    assign c_up   = prev_card ? prev_up   : s1_up_q;
    assign c_dn   = prev_card ? prev_dn   : s1_dn_q;
    // Raw flops instead of the view for ahead / target / direction / tag
    // where they decide a request (A3): an allocated view has confidence 0
    // and no pending, so with CONF_ON >= 1 its direct line comes from the
    // event line and it makes no scan line; the view select is then off
    // these paths (rule T-9).
    // "behind" is used only on a step (c_up: line = last + 1, c_dn: line =
    // last - 1, no wrap), so ahead < line is ahead <= last and ahead > line
    // is ahead >= last: two card flops, not the event copy (rule T-9)
    assign behind_up = (ahead_q <= last_q);
    assign behind_dn = (ahead_q >= last_q);

    assign conf_unset   = (v_conf == {CONF_W{1'b0}});
    assign step_up      = c_up & (conf_unset | v_dir);
    assign step_dn      = c_dn & (conf_unset | ~v_dir);
    assign step_ok      = step_up | step_dn;
    assign flip_ok      = (c_up | c_dn) & ~step_ok;
    assign ahead_behind = c_up ? behind_up : behind_dn;

    always @* begin : rule_proc
        if (c_same) begin
            // same line again: card unchanged
            new_conf  = v_conf;
            new_dir   = v_dir;
            new_tgt   = v_tgt;
        end
        else if (step_ok) begin
            // a step in the direction (or the first step)
            new_conf  = (v_conf == CONF_MAX_V) ? v_conf : (v_conf + CONF_ONE);
            new_dir   = c_up;
            new_tgt   = c_up ? ev_tgt_up : ev_tgt_dn;
        end
        else if (flip_ok) begin
            // a step against the direction: restart, direction flips
            new_conf  = {CONF_W{1'b0}};
            new_dir   = c_up;
            new_tgt   = c_up ? ev_tgt_up : ev_tgt_dn;
        end
        else begin
            // any other jump: restart in place, direction unset
            new_conf  = {CONF_W{1'b0}};
            new_dir   = 1'b0;
            new_tgt   = ev_tgt_dn;
        end // else (jump)
    end // always rule_proc
    assign new_pend = (new_conf >= CONF_ON_V) & (ev_trig | v_pend);

    // ======================================================================
    // S2: the direct candidate (the first request right after training)
    // ======================================================================
    assign ahead_p1 = ahead_q + LIP_ONE;
    assign ahead_m1 = ahead_q - LIP_ONE;
    assign ahead_p2 = ahead_q + LIP_TWO;
    assign ahead_m2 = ahead_q - LIP_TWO;

    // ---- duplicates, before the rule picks the line: ahead + 1 == key
    //      is ahead == key - 1 (6-bit wrap), so the stored ahead is compared
    //      with the parent's key +/- 1 directly ----
    always @* begin : dup_proc
        integer key_idx; // walks the recent keys
        ah_dup_p1  = 1'b0;
        ah_dup_m1  = 1'b0;
        own_dup_p1 = 1'b0;
        own_dup_m1 = 1'b0;
        for (key_idx = 0; key_idx < RECENT_N; key_idx = key_idx + 1) begin
            ah_dup_p1 = ah_dup_p1 | (recent_vld[key_idx] & recent_tag_eq[key_idx]
                & (recent_lm1[key_idx*LIP_W +: LIP_W] == ahead_q));
            ah_dup_m1 = ah_dup_m1 | (recent_vld[key_idx] & recent_tag_eq[key_idx]
                & (recent_lp1[key_idx*LIP_W +: LIP_W] == ahead_q));
            own_dup_p1 = own_dup_p1 | (recent_vld[key_idx]
                & (recent_tag[key_idx*TAG_W +: TAG_W] == tag_q)
                & (recent_lm1[key_idx*LIP_W +: LIP_W] == ahead_q));
            own_dup_m1 = own_dup_m1 | (recent_vld[key_idx]
                & (recent_tag[key_idx*TAG_W +: TAG_W] == tag_q)
                & (recent_lp1[key_idx*LIP_W +: LIP_W] == ahead_q));
        end // for key_idx
    end // always dup_proc

    // the next line and the done check, from the rule's outcome:
    // ahead = event line (ahead reset) or ahead = card's ahead (kept)
    assign ev_path = ~c_same & (~step_ok | (v_conf < CONF_ON_V) | ahead_behind);
    // ahead kept after a step: new target = event target, direction = c_up;
    // on a step the event target is the step target of last_q (tup_q,
    // tdn_q), so the check compares two card flops (rule T-9)
    assign kept_done_up = (ahead_q >= tup_q);
    assign kept_done_dn = (ahead_q <= tdn_q);

    // event branch: the rule restarts ahead at the event line; the new
    // direction is c_up (a step or flip sets it, a jump has c_up = 0)
    assign ev_next = c_up ? ev_line_p1 : ev_line_m1;
    assign ev_done = c_up ? ev_top : ev_bot;
    assign ev_dup  = c_up ? ev_dup_p1 : ev_dup_m1;
    // kept branch: same line (card unchanged) or a step with ahead kept
    assign kept_dir = c_same ? dir_q : c_up;
    assign kp_next  = kept_dir ? ahead_p1 : ahead_m1;
    assign kp_done  = c_same ? cur_done : (c_up ? kept_done_up : kept_done_dn);
    assign kp_dup   = kept_dir ? ah_dup_p1 : ah_dup_m1;
    assign kp_lp1   = kept_dir ? ahead_p2 : ahead_q;
    assign kp_lm1   = kept_dir ? ahead_q : ahead_m2;
    // the late ev_path picks a branch last
    // the late slot_go (S3 offer flags) is kept to the last 2:1 select:
    // every next value is built for both of its values first (rule T-9)
    assign pend_go  = hit & new_pend & scan_idle;
    assign mk_ev    = pend_go & ~ev_done;
    assign mk_kp    = pend_go & ~kp_done;
    assign d_make   = ev_path ? mk_ev : mk_kp;

    assign dir_vld   = slot_go & d_make;
    // the line itself is put together by the parent: one late bit
    // (ev_path) per card instead of a 6-bit select per card (rule T-9)
    assign dir_evp   = ev_path;
    assign dir_up    = c_up;
    assign dir_kp    = kp_next;
    assign dir_kp_p1 = kp_lp1;
    assign dir_kp_m1 = kp_lm1;
    assign dir_dup   = ev_path ? ev_dup  : kp_dup;
    assign dir_store = v_store;

    // ======================================================================
    // S2: the scan values of this card (used when the scan picked it)
    // ======================================================================
    assign cur_done = dir_q ? (ahead_q >= tgt_q) : (ahead_q <= tgt_q);
    assign cur_next = dir_q ? ahead_p1 : ahead_m1;
    always @* s_dup_r = dir_q ? own_dup_p1 : own_dup_m1;

    assign scan_go   = scan_sel & v_pend & ~cur_done;
    assign scan_done = scan_sel & v_pend & cur_done;
    assign scan_line = cur_next;
    assign scan_lp1  = dir_q ? ahead_p2 : ahead_q;
    assign scan_lm1  = dir_q ? ahead_q : ahead_m2;
    assign scan_dup  = s_dup_r;

    // {pending, ahead} next: the late bit is ev_hit (the behind compare
    // and the rule), slot_go is early; so both ev_hit values are built
    // first and a kept-hierarchy 2:1 by ev_hit is the last gate (rule T-9)
    //   ev_hit = 1: ahead = event line +/- 1 if a line goes, else the
    //               event line; pending = new_pend & ~ev_done
    //   ev_hit = 0: hit: kept ahead +/- 1 if a line goes, else the view;
    //               no hit: the scan line if it goes, else the view;
    //               pending ends when the kept ahead or the scan is done
    assign ev_hit    = hit & ev_path;
    assign go_ev     = (slot_go & mk_ev) ? ev_next : ev_line;
    assign pend_ev   = new_pend & ~ev_done;
    assign oth_go    = hit ? (mk_kp ? kp_next : v_ahead)
                           : (scan_go ? cur_next : v_ahead);
    assign poth_go   = hit ? (new_pend & ~kp_done) : (v_pend & ~scan_done);
    assign poth_stop = hit ? (new_pend & ~kp_done) : v_pend;
    assign go_oth    = slot_go ? oth_go : v_ahead;
    assign pend_oth  = slot_go ? poth_go : poth_stop;
    l1d_pf3_mux2 #(.W (LIP_W + 1)) u_go_sel (
        .sel (ev_hit),
        .in1 ({pend_ev, go_ev}),
        .in0 ({pend_oth, go_oth}),
        .out (go_pick)
    );

    // ======================================================================
    // the card's flops: training update (wins), scan write, allocation
    // ======================================================================
    // next victim class: an allocation now -> valid, confidence 0 (class
    // 1); else from the next valid bit and confidence (as the flops below)
    always @* begin : class_next_proc
        integer class_idx; // victim class
        conf_nx    = hit ? new_conf : v_conf;
        class_conf = {CONF_W{1'b0}};
        class_nx   = {NCLS{1'b0}};
        class_nx[0] = ~v_valid;
        for (class_idx = 1; class_idx < NCLS; class_idx = class_idx + 1) begin
            class_nx[class_idx] = v_valid & (conf_nx == class_conf);
            class_conf = class_conf + CONF_ONE;
        end // for class_idx
    end // always class_next_proc

    always @(posedge clk) begin : class_proc
        if (rst) begin
            class_q <= CLASS_FREE;
        end
        else if (alloc_d) begin
            class_q <= CLASS_NEW;
        end
        else begin
            class_q <= class_nx;
        end // else (no allocation)
    end // always class_proc

    always @(posedge clk) begin : card_ctl_proc
        if (rst) begin
            valid_q <= 1'b0;
            pend_q  <= 1'b0;
        end
        else begin
            valid_q <= v_valid;
            if (flush) begin
                pend_q <= 1'b0;
            end
            else begin
                pend_q <= go_pick[LIP_W];
            end // else (hit / scan / hold / allocation)
        end // else (run)
    end // always card_ctl_proc

    always @(posedge clk) begin : card_data_proc
        // identity: the view (written by an allocation of the event before)
        tag_q   <= v_tag;
        store_q <= v_store;
        if (hit) begin
            page_q    <= ev_page;
            last_q    <= ev_line;
            last_p1_q <= {1'b0, ev_line} + {{LIP_W{1'b0}}, 1'b1};
            dir_q     <= new_dir;
            conf_q    <= new_conf;
            tgt_q     <= new_tgt;
            tup_q     <= ev_tup;
            tdn_q     <= ev_tdn;
        end
        else begin
            page_q    <= v_page;
            last_q    <= v_last;
            last_p1_q <= v_last_p1;
            dir_q     <= v_dir;
            conf_q    <= v_conf;
            tgt_q     <= v_tgt;
            tup_q     <= v_tup;
            tdn_q     <= v_tdn;
        end // else (scan / hold / allocation)
        ahead_q <= go_pick[LIP_W-1:0];
    end // always card_data_proc

    assign pend_out  = v_pend;
    assign class_out = class_q;
    assign page_out  = v_page;
    assign tag_out   = v_tag;
    assign store_out = v_store;

`ifdef L1D_DEBUG
    // ---- check (simulation only): the class flop equals the class of the
    //      view (valid, confidence) ----
    reg [NCLS-1:0]   class_view; // 5: class from the view
    reg [CONF_W-1:0] view_conf;  // scratch: confidence of class k
    always @* begin : class_view_proc
        integer class_idx; // victim class
        view_conf     = {CONF_W{1'b0}};
        class_view    = {NCLS{1'b0}};
        class_view[0] = ~v_valid;
        for (class_idx = 1; class_idx < NCLS; class_idx = class_idx + 1) begin
            class_view[class_idx] = v_valid & (v_conf == view_conf);
            view_conf = view_conf + CONF_ONE;
        end // for class_idx
    end // always class_view_proc
    // check: on a step that keeps the stream (the only case where they are
    // used) the flop compares equal the event compares, and the ev_hit
    // select equals the old slot_go select
    wire chk_step = hit & ~c_same & step_ok & (v_conf >= CONF_ON_V)
                    & ~alloc_cp[0] & ~alloc_cp[1];
    wire chk_behind = c_up ? (ahead_q < ev_line) : (ahead_q > ev_line);
    wire chk_kdone  = c_up ? (ahead_q >= ev_tgt_up) : (ahead_q <= ev_tgt_dn);
    // the old form: slot_go picks last from the four values below
    wire [LIP_W-1:0] ahead_go;     // 6 bits: next ahead if slot_go
    wire [LIP_W-1:0] ahead_stop;   // 6 bits: next ahead if not
    wire             pend_go_nx;   // next pending if slot_go
    wire             pend_stop_nx; // next pending if not
    wire             d_finish;     // pending ends at once
    assign d_finish = hit & new_pend & (ev_path ? ev_done : kp_done);
    // ahead: a hit takes the direct line if one is made, else the rule's
    // value; no hit: the scan line if it goes, else hold / allocation
    assign ahead_go   = hit ? (ev_path ? (mk_ev ? ev_next : ev_line)
                                       : (mk_kp ? kp_next : v_ahead))
                            : (scan_go ? cur_next : v_ahead);
    assign ahead_stop = (hit && ev_path) ? ev_line : v_ahead;
    // pending: a hit takes the rule's value; no hit: the scan ends it
    assign pend_go_nx   = hit ? (new_pend & ~d_finish) : (v_pend & ~scan_done);
    assign pend_stop_nx = hit ? (new_pend & ~d_finish) : v_pend;
    wire [LIP_W:0] chk_pick = slot_go ? {pend_go_nx, ahead_go}
                                      : {pend_stop_nx, ahead_stop};
    always @(posedge clk) begin : debug_proc
        if (!rst && (class_q != class_view)) begin
            $display("[%0t] L1D PF3_STREAM_CARD ERROR class flop != view class",
                     $time);
        end
        if (!rst && chk_step && (ahead_behind != chk_behind)) begin
            $display("[%0t] L1D PF3_STREAM_CARD ERROR behind flop != event",
                     $time);
        end
        if (!rst && chk_step && (c_up ? kept_done_up : kept_done_dn) != chk_kdone) begin
            $display("[%0t] L1D PF3_STREAM_CARD ERROR kept done flop != event",
                     $time);
        end
        if (!rst && !flush && (go_pick != chk_pick)) begin
            $display("[%0t] L1D PF3_STREAM_CARD ERROR go select != slot_go select",
                     $time);
        end
    end // always debug_proc
`endif

endmodule

`default_nettype wire
`endif // L1D_PF3_STREAM_CARD_V
