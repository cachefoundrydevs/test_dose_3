// ==========================================================================
// l1d_pf3_rank.v - S3 of the prefetch system: per bank the highest-rank
//                  request (stride > stream > SMS), hold, duplicates, SMS
//                  bank credits, and the output register pf_req_*
// ==========================================================================
// IN SIMPLE WORDS : each prefetcher offers at most one request per cycle.
//   For every bank this block sends the best offer for that bank (rule
//   SH-1: stride first, then stream, then SMS), if the bank's prefetch
//   register has room. An offer that loses waits here ("held") and tries
//   again, at most PF_HOLD_MAX decision cycles in total, then it is
//   dropped. An offer for the same line as a higher-rank offer is
//   dropped at once (duplicate). A newer stride offer replaces a held
//   stride offer (the stride's "newest first" rule); stream and SMS only
//   offer when their place is empty, so their offers are never replaced.
//   SMS requests also need a bank credit: at most CREDIT_PER_BANK SMS
//   requests per bank wait for their feedback (as the SMS expander did).
//
// STAGE / CYCLE   : S3. Inputs: the stride candidate (combinational from
//   the stride's S2 flops, about 5 cells), the stream and SMS candidates
//   (flops), the held offers (flops here), pf_req_ready (bank flops).
//   Outputs pf_req_* are registered: they are the unit's output register.
// INSTANTIATED BY : l1d_prefetch_unit, instance u_rank, 1 copy.
//
// PARAMETERS:
//   name             default unit    meaning
//   NB               4       banks   cache banks
//   LINE_W           58      bits    cache line address
//   HOLD_MAX         2       cycles  decision cycles of an offer (>= 1)
//   CREDIT_PER_BANK  2       count   SMS requests without feedback / bank
//   CREDIT_WAIT_MAX  64      cycles  SMS wait for a credit, then all
//                                    credits are given back (bound, S-1)
//
// PORTS (default widths):
//   clk, rst, pf_enable          in  1        clock, reset, on / off
//   pf_req_ready                 in  4        bank b takes a hint (flop)
//   s_valid, s_line, s_bank,     in  1/58/4/1 stride candidate (S3 comb.)
//   s_unique
//   t_valid_dp                   in  5        t_valid, NB+1 more copies
//   s_valid_dp                   in  5        s_valid, NB+1 more copies:
//                                             bit b picks bank b's line
//                                             (last, kept 2:1), bit NB the
//                                             offer line / unique / loads
//   t_valid, t_line, t_bank,     in  1/58/4/1 stream candidate (flops)
//   t_unique
//   m_valid, m_line, m_bank      in  1/58/4   SMS candidate (flops)
//   m_valid_dp                   in  1        the same valid, a separate
//                                             copy for the line / hold
//                                             selects (fanout split)
//   t_line_scan, t_line_direct,  in  58/58/1  the stream's two possible
//   t_pick_scan                               lines (flops) and its own
//                                             copy of the select flop:
//                                             t_line = pick ? scan : direct
//   m_line_scan, m_line_direct,  in  58/58/1  the same for the SMS line
//   m_pick_scan                               (the direct one is the load)
//   fb_valid                     in  4        feedback from bank b: returns
//                                             one SMS credit of bank b
//   t_held, m_held               out 1        stream / SMS place is busy
//   pf_req_valid/_line/_unique   out 4/4x58/4 per bank hint (registered)
//
// WORKED EXAMPLE : stride and stream both offer line 0x1200 (bank 0),
//   bank 0 ready: stride's offer goes out, stream's is a duplicate and is
//   dropped. Next cycle stream offers 0x1201 (bank 1), bank 1 not ready:
//   it is held; one cycle later bank 1 is ready and it goes out.
//
// LOGIC LEVELS (T-9, estimate): offer select 1-2, bank & ready 1, rank
//   chain 1, valid OR 1; the stride offer comes in at about 5 cells, so
//   pf_req_valid D is at about 8; pf_req_line: the line for both stride
//   cases and both stream cases is built from flops and the banks, and
//   each bank's own copy of t_valid, then of s_valid, picks (kept 2:1
//   last), so s_valid -> pf_req_line D is about 3 with its buffer. SMS credit:
//   both next counts from flops, m_win (about 7) + 2:1 select 1 = 8 (the
//   select is a kept l1d_pf3_mux2 per bank, so it stays last).
//   SMS offer: sent / duplicate / kept / waits are made for a new and
//   for the held offer (A1), the late m_valid picks last (1); the same
//   for the stream offer and t_valid (the S3 OR of the stream's flags).
//   Stream / SMS decisions: two copies of l1d_pf3_rank_low (u_low_new,
//   u_low_held), one per stride case, about 6 levels from flops; the
//   stride's late s_valid picks last (1), so s_valid -> hold flags /
//   credits is about 3 levels. The stream / SMS hold valids: per case
//   "would keep" AND the case select (and m_valid), then the duplicate
//   bits from the line compares enter last in l1d_pf3_keep_or (u_t_keep,
//   u_m_keep, 3 levels), so a compare result is about 4 levels from the
//   hold flop.
//
// ASSUMPTIONS (V-9.2): A1 t_valid only when t_held is 0, m_valid only when
//   m_held is 0 (the prefetchers check it). A2 the bank of a line is its
//   lowest log2(NB) bits, and *_bank is that one-hot.
// ==========================================================================
`ifndef L1D_PF3_RANK_V
`define L1D_PF3_RANK_V
`default_nettype none

module l1d_pf3_rank #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter NB              = 4,  // default 4 banks
    parameter LINE_W          = 58, // default 58 bits: line address
    parameter HOLD_MAX        = 2,  // default 2 decision cycles
    parameter CREDIT_PER_BANK = 2,  // default 2 SMS credits per bank
    parameter CREDIT_WAIT_MAX = 64  // default 64 cycles: credit wait
) (
    // ---- clock and reset ----
    input  wire                 clk,          // clock
    input  wire                 rst,          // sync reset, high
    input  wire                 pf_enable,    // 1 = on
    // ---- banks ----
    input  wire [NB-1:0]        pf_req_ready, // 4: bank takes a hint
    // ---- stride candidate (S3, combinational) ----
    input  wire                 s_valid,      // candidate valid (late)
    input  wire [NB:0]          s_valid_dp,   // NB+1 = 5: same, data copies
    input  wire [LINE_W-1:0]    s_line,       // 58 bits
    input  wire [NB-1:0]        s_bank,       // 4: one-hot bank
    input  wire                 s_unique,     // ask unique
    // ---- stream candidate (flops) ----
    input  wire                 t_valid,      // candidate valid
    input  wire [NB:0]          t_valid_dp,   // NB+1 = 5: same, data copies
    input  wire [LINE_W-1:0]    t_line,       // 58 bits
    input  wire [LINE_W-1:0]    t_line_scan,  // 58 bits: scan line
    input  wire [LINE_W-1:0]    t_line_direct,// 58 bits: direct line
    input  wire                 t_pick_scan,  // t_line = scan line
    input  wire [NB-1:0]        t_bank,       // 4: one-hot bank
    input  wire                 t_unique,     // ask unique
    // ---- SMS candidate (flops) ----
    input  wire                 m_valid,      // candidate valid
    input  wire                 m_valid_dp,   // same, data copy
    input  wire [LINE_W-1:0]    m_line,       // 58 bits
    input  wire [LINE_W-1:0]    m_line_scan,  // 58 bits: scan line
    input  wire [LINE_W-1:0]    m_line_direct,// 58 bits: direct line
    input  wire                 m_pick_scan,  // m_line = scan line
    input  wire [NB-1:0]        m_bank,       // 4: one-hot bank
    // ---- SMS credit return ----
    input  wire [NB-1:0]        fb_valid,     // 4: feedback per bank
    // ---- to the prefetchers ----
    output wire                 t_held,       // stream place busy
    output wire                 m_held,       // SMS place busy
    // ---- the unit's output register ----
    output reg  [NB-1:0]        pf_req_valid, // 4: hint per bank
    output reg  [NB*LINE_W-1:0] pf_req_line,  // 4 x 58 bits
    output reg  [NB-1:0]        pf_req_unique // 4: ask unique
);

    // ---- local constants (value at the default) ----
    localparam HOLD_W = $clog2(HOLD_MAX + 1);          // 2 bits
    localparam CRED_W = $clog2(CREDIT_PER_BANK + 1);   // 2 bits
    localparam WAIT_W = $clog2(CREDIT_WAIT_MAX + 1);   // 7 bits
    localparam [HOLD_W-1:0] HOLD_ONE  = 1;             // first held cycle
    localparam [HOLD_W-1:0] HOLD_LAST = HOLD_MAX - 1;  // last chance count
    localparam [CRED_W-1:0] CRED_MAX  = CREDIT_PER_BANK; // 2
    localparam LOW_W  = 2*NB;                      // 8 bits: t / m wins
    localparam KB_W   = 6;    // 6 keep bits per case: t base / dup, new
                              //   SMS base / dup, held SMS base / dup
    localparam CASE_N = 4;                         // 4 cases: s_valid x t_valid
    localparam [WAIT_W-1:0] WAIT_LAST = CREDIT_WAIT_MAX; // 64
    localparam MEQ_N  = 5;    // 5 compares with the SMS line, picked by
                              //   m_pick_scan
    localparam TEQ_N  = 4;    // 4 compares with the stream line, picked
                              //   by t_pick_scan

    // ---- held offers (one place per prefetcher) ----
    reg                  s_hold_vld;   // stride offer held
    reg  [LINE_W-1:0]    s_hold_line;  // 58 bits
    reg  [NB-1:0]        s_hold_bank;  // 4
    reg                  s_hold_uniq;  // ask unique
    reg  [HOLD_W-1:0]    s_hold_cnt;   // decision cycles spent, 2 bits
    reg                  t_hold_vld;   // stream offer held
    reg  [LINE_W-1:0]    t_hold_line;  // 58 bits
    reg  [NB-1:0]        t_hold_bank;  // 4
    reg                  t_hold_uniq;  // ask unique
    reg  [HOLD_W-1:0]    t_hold_cnt;   // 2 bits
    reg                  m_hold_vld;   // SMS offer held
    reg  [LINE_W-1:0]    m_hold_line;  // 58 bits
    reg  [NB-1:0]        m_hold_bank;  // 4
    reg  [HOLD_W-1:0]    m_hold_cnt;   // 2 bits

    // ---- SMS credits ----
    reg  [NB*CRED_W-1:0] credit_q;     // 4 x 2: SMS hints in flight
    reg  [NB-1:0]        credit_ok_q;  // 4: credit_q < CREDIT_PER_BANK
    reg  [WAIT_W-1:0]    wait_q;       // 7: cycles an SMS offer waited
    reg  [NB*CRED_W-1:0] credit_next;  // 4 x 2
    reg  [CRED_W-1:0]    one_credit;   // scratch: one bank's count
    reg  [CRED_W-1:0]    credit_win;   // scratch: count if bank b's SMS
                                       //   hint is sent now, 2 bits
    reg  [CRED_W-1:0]    credit_stay;  // scratch: count if it is not, 2 bits
    reg  [NB-1:0]        ok_next;      // 4: next credit_ok_q
    reg  [NB*(CRED_W+1)-1:0] cred_win_all;  // 4 x 3: {ok, count} if sent
    reg  [NB*(CRED_W+1)-1:0] cred_stay_all; // 4 x 3: {ok, count} if not
    wire [NB*(CRED_W+1)-1:0] cred_pick;     // 4 x 3: the picked one

    // ---- this cycle's offers ----
    wire                 s_off;        // stride offers
    wire [NB-1:0]        s_off_bank;   // 4
    wire                 s_off_uniq;   // ask unique
    wire [HOLD_W-1:0]    s_off_cnt;    // decisions before this one
    wire [NB-1:0]        t_off_bank;   // 4
    wire                 t_off_uniq;   // ask unique
    wire [HOLD_W-1:0]    t_off_cnt;    // 2 bits
    wire [LINE_W-1:0]    m_off_line;   // 58 bits
    wire [HOLD_W-1:0]    m_off_cnt;    // 2 bits
    wire [NB-1:0]        s_want;       // 4: stride wants bank b
    wire [NB-1:0]        t_want;       // 4: stream wants bank b
    wire [NB-1:0]        s_win;        // 4: stride sends to bank b
    wire [NB-1:0]        t_win;        // 4: stream sends to bank b
    wire [NB-1:0]        m_win;        // 4: SMS sends to bank b
    wire                 s_taken;      // stride offer sent
    wire                 s_keep;       // stride offer stays held
    wire                 t_keep;       // stream offer stays held
    wire                 m_keep;       // SMS offer stays held
    wire                 m_waits;      // SMS offer has no credit
    // the stream / SMS decisions are made by four copies of
    // l1d_pf3_rank_low: stride offer new (s_valid = 1) or held, times a
    // new stream offer (t_valid = 1) or not; both late bits pick through
    // one kept one-hot AND-OR (rule T-9). Inside, m_valid picks the new
    // or held SMS offer (A1)
    wire [NB-1:0]        s_want_n;     // 4: new stride offer's bank (on)
    wire [NB-1:0]        s_want_h;     // 4: held stride offer's bank (on)
    wire                 sn_eq_tn;     // new stride line = new stream line
    wire                 sn_eq_th;     // new stride line = held stream line
    wire                 sn_eq_mn;     // new stride line = new SMS line
    wire                 sn_eq_mh;     // new stride line = held SMS line
    wire                 sh_eq_tn;     // held stride line = new stream line
    wire                 sh_eq_th;     // held stride line = held stream line
    wire                 sh_eq_mn;     // held stride line = new SMS line
    wire                 sh_eq_mh;     // held stride line = held SMS line
    wire                 tn_mn_eq;     // new stream line = new SMS line
    wire                 th_mn_eq;     // held stream line = new SMS line
    wire                 tn_mh_eq;     // new stream line = held SMS line
    wire                 th_mh_eq;     // held stream line = held SMS line
    wire [MEQ_N-1:0]     meq_scan;     // MEQ_N = 5: compares with the
                                       //   SMS scan line
    wire [MEQ_N-1:0]     meq_direct;   // 5: the same with the direct line
    wire [MEQ_N-1:0]     meq_pick;     // 5: the picked ones
    wire [TEQ_N-1:0]     teq_scan;     // TEQ_N = 4: compares with the
                                       //   stream scan line
    wire [TEQ_N-1:0]     teq_direct;   // 4: the same with the direct line
    wire [TEQ_N-1:0]     teq_pick;     // 4: the picked ones
    wire                 ts_mn_eq;     // stream scan line = new SMS line
    wire                 td_mn_eq;     // stream direct line = new SMS line
    wire                 mn_waits;     // new SMS offer has no credit
    wire                 mh_waits;     // held SMS offer has no credit
    wire [CASE_N*LOW_W-1:0] low_case;  // 4 x 8: wins per case
    wire [CASE_N*KB_W-1:0] keep_case;  // 4 x 6: keep bits per case
    reg  [CASE_N-1:0]    t_keep_go;    // 4: case k chosen, stream would keep
    reg  [CASE_N-1:0]    t_keep_drop;  // 4: case k's stream offer is a dup
    reg  [2*CASE_N-1:0]  m_keep_go;    // 8: case k x (new, held) SMS offer
    reg  [2*CASE_N-1:0]  m_keep_drop;  // 8: the same, duplicate
                                       //   (2 x t_valid + s_valid)
    wire [CASE_N-1:0]    low_sel_oh;   // CASE_N = 4: one-hot case select
    wire                 m_timeout;    // the credit wait reached its bound
    reg  [NB-1:0]        valid_next;   // 4: hint per bank next cycle
    wire [NB*LINE_W-1:0] line_pick;    // 4 x 58: next pf_req_line
    reg  [NB-1:0]        unique_next;  // 4
    reg  [LINE_W-1:0]    tm_new_t;     // 58: stream / SMS line, new stream
    reg  [LINE_W-1:0]    tm_old_t;     // 58: stream / SMS line, no new one
    // the line per bank for the four cases (k = 2 x t_valid + s_valid)
    reg  [NB*CASE_N*LINE_W-1:0] line_case; // 4 x 4 x 58
    reg  [NB*CASE_N-1:0] line_sel_oh;  // 4 x 4: one-hot case per bank
    wire [NB-1:0]        t_want_nb;    // 4: new stream offer's bank (on)
    wire [NB-1:0]        t_want_hb;    // 4: held stream offer's bank (on)

    // ======================================================================
    // this cycle's offer of each prefetcher: a new candidate first
    // ======================================================================
    assign s_off      = s_valid | s_hold_vld;
    assign s_off_bank = s_valid ? s_bank : (s_hold_bank & {NB{s_hold_vld}});
    assign s_off_uniq = s_valid_dp[NB] ? s_unique : s_hold_uniq;
    assign s_off_cnt  = s_valid ? {HOLD_W{1'b0}} : s_hold_cnt;
    assign t_off_bank = t_valid ? t_bank : (t_hold_bank & {NB{t_hold_vld}});
    assign t_off_uniq = t_valid_dp[NB] ? t_unique : t_hold_uniq;
    assign t_off_cnt  = t_valid ? {HOLD_W{1'b0}} : t_hold_cnt;
    assign m_off_line = m_valid_dp ? m_line : m_hold_line;
    assign m_off_cnt  = m_valid_dp ? {HOLD_W{1'b0}} : m_hold_cnt;

    // ======================================================================
    // per bank: rank chain stride > stream > SMS (a priority on purpose)
    // ======================================================================
    assign s_want = s_off_bank & {NB{pf_enable}};
    assign t_want = t_off_bank & {NB{pf_enable}};
    assign t_want_nb = t_bank & {NB{pf_enable}};
    assign t_want_hb = t_hold_bank & {NB{t_hold_vld & pf_enable}};
    assign s_win  = s_want & pf_req_ready;
    assign s_taken = |s_win;
    assign s_want_n = s_bank & {NB{pf_enable}};
    assign s_want_h = s_hold_bank & {NB{s_hold_vld & pf_enable}};

    // the 58-bit line compares, all from inputs and flops (none waits for
    // a late valid bit). The stream and SMS lines are each a 2:1 select
    // by a flop; so each compare with them is made for both possible
    // lines (all flops, or the SMS load), and the select flop's own copy
    // picks the result (rule T-9): the select and its 58-bit fanout are
    // no longer in front of the compare trees. The pick is plain logic,
    // not a kept l1d_pf3_mux2: a kept block's output looks early to the
    // synthesis of the decisions after it (run top48)
    assign meq_scan   = {(t_line_direct == m_line_scan),
                         (t_line_scan   == m_line_scan),
                         t_hold_vld & (t_hold_line == m_line_scan),
                         (s_hold_line   == m_line_scan),
                         (s_line        == m_line_scan)};
    assign meq_direct = {(t_line_direct == m_line_direct),
                         (t_line_scan   == m_line_direct),
                         t_hold_vld & (t_hold_line == m_line_direct),
                         (s_hold_line   == m_line_direct),
                         (s_line        == m_line_direct)};
    assign meq_pick = m_pick_scan ? meq_scan : meq_direct;
    assign sn_eq_mn = meq_pick[0];
    assign sh_eq_mn = meq_pick[1];
    assign th_mn_eq = meq_pick[2];
    assign ts_mn_eq = meq_pick[3];
    assign td_mn_eq = meq_pick[4];
    assign teq_scan   = {ts_mn_eq,
                         (t_line_scan   == m_hold_line),
                         (s_hold_line   == t_line_scan),
                         (s_line        == t_line_scan)};
    assign teq_direct = {td_mn_eq,
                         (t_line_direct == m_hold_line),
                         (s_hold_line   == t_line_direct),
                         (s_line        == t_line_direct)};
    assign teq_pick = t_pick_scan ? teq_scan : teq_direct;
    assign sn_eq_tn = teq_pick[0];
    assign sh_eq_tn = teq_pick[1];
    assign tn_mh_eq = teq_pick[2];
    assign tn_mn_eq = teq_pick[3];
    assign sn_eq_th = (s_line == t_hold_line);
    assign sn_eq_mh = (s_line == m_hold_line);
    assign sh_eq_th = (s_hold_line == t_hold_line);
    assign sh_eq_mh = (s_hold_line == m_hold_line);
    assign th_mh_eq = t_hold_vld & (t_hold_line == m_hold_line);

    // the four cases (stride offer new / held) x (stream offer new or
    // not), all built with both late bits fixed; s_valid and t_valid (both
    // late) are decoded to one-hot selects and pick through one kept
    // AND-OR (rule T-9: one select level on every path)
    assign low_sel_oh = {s_valid & t_valid, ~s_valid & t_valid,
                         s_valid & ~t_valid, ~s_valid & ~t_valid};
    genvar tcase_g; // generate index: 1 = t_valid case, 0 = no new stream
    generate
        for (tcase_g = 0; tcase_g < 2; tcase_g = tcase_g + 1) begin : g_tcase
            l1d_pf3_rank_low #(.NB (NB), .HOLD_MAX (HOLD_MAX)) u_low_new (
                .pf_enable    (pf_enable),    .pf_req_ready (pf_req_ready),
                .credit_ok    (credit_ok_q),
                .s_want       (s_want_n),     .s_off        (1'b1),
                .s_eq_tn      (sn_eq_tn),     .s_eq_th      (sn_eq_th),
                .s_eq_mn      (sn_eq_mn),     .s_eq_mh      (sn_eq_mh),
                .t_valid      (tcase_g == 1), .t_bank       (t_bank),
                .t_hold_vld   (t_hold_vld),   .t_hold_bank  (t_hold_bank),
                .t_hold_cnt   (t_hold_cnt),
                .m_valid      (m_valid),      .m_bank       (m_bank),
                .m_hold_vld   (m_hold_vld),   .m_hold_bank  (m_hold_bank),
                .m_hold_cnt   (m_hold_cnt),
                .tn_mn_eq     (tn_mn_eq),     .th_mn_eq     (th_mn_eq),
                .tn_mh_eq     (tn_mh_eq),     .th_mh_eq     (th_mh_eq),
                .mn_waits     (mn_waits),     .mh_waits     (mh_waits),
                .t_win        (low_case[(2*tcase_g+1)*LOW_W + NB +: NB]),
                .m_win        (low_case[(2*tcase_g+1)*LOW_W +: NB]),
                .t_base       (keep_case[(2*tcase_g+1)*KB_W + 5]),
                .t_dup        (keep_case[(2*tcase_g+1)*KB_W + 4]),
                .mn_base      (keep_case[(2*tcase_g+1)*KB_W + 3]),
                .mn_dup       (keep_case[(2*tcase_g+1)*KB_W + 2]),
                .mh_base      (keep_case[(2*tcase_g+1)*KB_W + 1]),
                .mh_dup       (keep_case[(2*tcase_g+1)*KB_W])
            );
            l1d_pf3_rank_low #(.NB (NB), .HOLD_MAX (HOLD_MAX)) u_low_held (
                .pf_enable    (pf_enable),    .pf_req_ready (pf_req_ready),
                .credit_ok    (credit_ok_q),
                .s_want       (s_want_h),     .s_off        (s_hold_vld),
                .s_eq_tn      (sh_eq_tn),     .s_eq_th      (sh_eq_th),
                .s_eq_mn      (sh_eq_mn),     .s_eq_mh      (sh_eq_mh),
                .t_valid      (tcase_g == 1), .t_bank       (t_bank),
                .t_hold_vld   (t_hold_vld),   .t_hold_bank  (t_hold_bank),
                .t_hold_cnt   (t_hold_cnt),
                .m_valid      (m_valid),      .m_bank       (m_bank),
                .m_hold_vld   (m_hold_vld),   .m_hold_bank  (m_hold_bank),
                .m_hold_cnt   (m_hold_cnt),
                .tn_mn_eq     (tn_mn_eq),     .th_mn_eq     (th_mn_eq),
                .tn_mh_eq     (tn_mh_eq),     .th_mh_eq     (th_mh_eq),
                .mn_waits     (mn_waits),     .mh_waits     (mh_waits),
                .t_win        (low_case[(2*tcase_g)*LOW_W + NB +: NB]),
                .m_win        (low_case[(2*tcase_g)*LOW_W +: NB]),
                .t_base       (keep_case[(2*tcase_g)*KB_W + 5]),
                .t_dup        (keep_case[(2*tcase_g)*KB_W + 4]),
                .mn_base      (keep_case[(2*tcase_g)*KB_W + 3]),
                .mn_dup       (keep_case[(2*tcase_g)*KB_W + 2]),
                .mh_base      (keep_case[(2*tcase_g)*KB_W + 1]),
                .mh_dup       (keep_case[(2*tcase_g)*KB_W])
            );
        end // for tcase_g
    endgenerate
    // case k of low_case: k = 2 * t + s (s = 1: stride offer new), so the
    // one-hot bit {s & t, ~s & t, s & ~t, ~s & ~t} = case 3, 2, 1, 0
    l1d_pf3_mux4oh #(.W (LOW_W), .SEL_N (CASE_N)) u_low_sel (
        .sel_oh (low_sel_oh),
        .in3    (low_case[3*LOW_W +: LOW_W]),
        .in2    (low_case[2*LOW_W +: LOW_W]),
        .in1    (low_case[1*LOW_W +: LOW_W]),
        .in0    (low_case[0 +: LOW_W]),
        .out    ({t_win, m_win})
    );

    // the keeps: per case, "would keep" ANDed with the case select (and
    // m_valid for the SMS), then the duplicate bits (late: the line
    // compares) enter last in a small kept AND-NOT-OR in front of the
    // hold valid flops (l1d_pf3_keep_or)
    always @* begin : keep_go_proc
        integer case_idx; // walks the four cases
        for (case_idx = 0; case_idx < CASE_N; case_idx = case_idx + 1) begin
            t_keep_go[case_idx]   = low_sel_oh[case_idx]
                                    & keep_case[case_idx*KB_W + 5];
            t_keep_drop[case_idx] = keep_case[case_idx*KB_W + 4];
            m_keep_go[2*case_idx + 1]   = low_sel_oh[case_idx] & m_valid
                                          & keep_case[case_idx*KB_W + 3];
            m_keep_drop[2*case_idx + 1] = keep_case[case_idx*KB_W + 2];
            m_keep_go[2*case_idx]       = low_sel_oh[case_idx] & ~m_valid
                                          & keep_case[case_idx*KB_W + 1];
            m_keep_drop[2*case_idx]     = keep_case[case_idx*KB_W];
        end // for case_idx
    end // always keep_go_proc
    l1d_pf3_keep_or #(.N (CASE_N)) u_t_keep (
        .go   (t_keep_go),
        .drop (t_keep_drop),
        .out  (t_keep)
    );
    l1d_pf3_keep_or #(.N (2*CASE_N)) u_m_keep (
        .go   (m_keep_go),
        .drop (m_keep_drop),
        .out  (m_keep)
    );

    always @* begin : out_next_proc
        integer bank_idx; // walks the banks
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
            valid_next[bank_idx] = s_win[bank_idx] | t_win[bank_idx]
                                   | m_win[bank_idx];
            // the data needs no ready: valid qualifies it. The line for
            // the four cases (stride new / held, stream new / not); the
            // bank copies of s_valid and t_valid pick through one kept
            // one-hot AND-OR (below)
            tm_new_t = t_want_nb[bank_idx] ? t_line : m_off_line;
            tm_old_t = t_want_hb[bank_idx] ? t_hold_line : m_off_line;
            line_case[(bank_idx*CASE_N + 3)*LINE_W +: LINE_W] =
                s_want_n[bank_idx] ? s_line : tm_new_t;
            line_case[(bank_idx*CASE_N + 2)*LINE_W +: LINE_W] =
                s_want_h[bank_idx] ? s_hold_line : tm_new_t;
            line_case[(bank_idx*CASE_N + 1)*LINE_W +: LINE_W] =
                s_want_n[bank_idx] ? s_line : tm_old_t;
            line_case[(bank_idx*CASE_N)*LINE_W +: LINE_W] =
                s_want_h[bank_idx] ? s_hold_line : tm_old_t;
            line_sel_oh[bank_idx*CASE_N +: CASE_N] = {
                s_valid_dp[bank_idx] & t_valid_dp[bank_idx],
                ~s_valid_dp[bank_idx] & t_valid_dp[bank_idx],
                s_valid_dp[bank_idx] & ~t_valid_dp[bank_idx],
                ~s_valid_dp[bank_idx] & ~t_valid_dp[bank_idx]};
            unique_next[bank_idx] = s_want[bank_idx] ? s_off_uniq
                                    : (t_want[bank_idx] & t_off_uniq);
        end // for bank_idx
    end // always out_next_proc

    // per bank, s_valid (its own copy) picks the line last
    genvar lbank_g; // generate index: bank of the line select
    generate
        for (lbank_g = 0; lbank_g < NB; lbank_g = lbank_g + 1) begin : g_line_sel
            l1d_pf3_mux4oh #(.W (LINE_W), .SEL_N (CASE_N)) u_line_sel (
                .sel_oh (line_sel_oh[lbank_g*CASE_N +: CASE_N]),
                .in3    (line_case[(lbank_g*CASE_N + 3)*LINE_W +: LINE_W]),
                .in2    (line_case[(lbank_g*CASE_N + 2)*LINE_W +: LINE_W]),
                .in1    (line_case[(lbank_g*CASE_N + 1)*LINE_W +: LINE_W]),
                .in0    (line_case[(lbank_g*CASE_N)*LINE_W +: LINE_W]),
                .out    (line_pick[lbank_g*LINE_W +: LINE_W])
            );
        end // for lbank_g
    endgenerate

    always @(posedge clk) begin : out_reg_proc
        if (rst) begin
            pf_req_valid  <= {NB{1'b0}};
            pf_req_unique <= {NB{1'b0}};
        end
        else begin
            pf_req_valid  <= valid_next;
            pf_req_unique <= unique_next;
        end // else (rst)
        pf_req_line <= line_pick;  // wide data: pf_req_valid qualifies it
    end // always out_reg_proc

    // ======================================================================
    // hold, duplicate drop, newest first (only the valid bits are late)
    // ======================================================================
    // an offer stays while it has decision cycles left (HOLD_MAX); the
    // stream / SMS keeps come from l1d_pf3_rank_low above
    assign s_keep = s_off & ~s_taken & (s_off_cnt != HOLD_LAST);

    always @(posedge clk) begin : hold_ctl_proc
        if (rst || !pf_enable) begin
            s_hold_vld <= 1'b0;
            t_hold_vld <= 1'b0;
            m_hold_vld <= 1'b0;
        end
        else begin
            s_hold_vld <= s_keep;
            t_hold_vld <= t_keep;
            m_hold_vld <= m_keep;
        end // else (run)
        s_hold_cnt <= s_off_cnt + HOLD_ONE;
        t_hold_cnt <= t_off_cnt + HOLD_ONE;
        m_hold_cnt <= m_waits ? m_off_cnt : (m_off_cnt + HOLD_ONE);
    end // always hold_ctl_proc

    // held data loads from a new candidate (stream / SMS enables are
    // flops; the stride enable is its filter result, about 5 cells)
    always @(posedge clk) begin : hold_data_proc
        if (s_valid_dp[NB]) begin
            s_hold_line <= s_line;
            s_hold_bank <= s_bank;
            s_hold_uniq <= s_unique;
        end
        if (t_valid_dp[NB]) begin
            t_hold_line <= t_line;
            t_hold_bank <= t_bank;
            t_hold_uniq <= t_unique;
        end
        if (m_valid_dp) begin
            m_hold_line <= m_line;
            m_hold_bank <= m_bank;
        end
    end // always hold_data_proc

    assign t_held = t_hold_vld;
    assign m_held = m_hold_vld;

    // ======================================================================
    // SMS bank credits: +1 when an SMS hint is sent, -1 on feedback
    // ======================================================================
    assign mn_waits  = ~(|(m_bank & credit_ok_q)) & pf_enable;
    assign mh_waits  = m_hold_vld & ~(|(m_hold_bank & credit_ok_q)) & pf_enable;
    assign m_waits   = m_valid ? mn_waits : mh_waits;
    assign m_timeout = m_waits & (wait_q == WAIT_LAST);

    // both outcomes are worked out from flops (count, feedback, timeout)
    // and m_win, the late bit, picks one of them last (rule T-9)
    always @* begin : credit_next_proc
        integer bank_idx; // walks the banks
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
            one_credit = credit_q[bank_idx*CRED_W +: CRED_W];
            // sent: +1, unless a feedback returns one in the same cycle
            credit_win  = (fb_valid[bank_idx] && (one_credit != {CRED_W{1'b0}}))
                          ? one_credit : (one_credit + 1'b1);
            // not sent: -1 on a feedback
            credit_stay = (fb_valid[bank_idx] && (one_credit != {CRED_W{1'b0}}))
                          ? (one_credit - 1'b1) : one_credit;
            // both outcomes with their terminal compare (kept as a flop,
            // T-8); the kept 2:1 below picks by m_win, and the late
            // m_timeout (all credits back) clears after it (count 0, ok 1)
            cred_win_all[bank_idx*(CRED_W+1) +: CRED_W+1] =
                {(credit_win < CRED_MAX), credit_win};
            cred_stay_all[bank_idx*(CRED_W+1) +: CRED_W+1] =
                {(credit_stay < CRED_MAX), credit_stay};
        end // for bank_idx
    end // always credit_next_proc

    // m_win comes late (stream and SMS valids, stride case select): one
    // kept 2:1 per bank, so synthesis keeps it the last gate (T-9)
    genvar cbank_g; // generate index: bank of the credit select
    generate
        for (cbank_g = 0; cbank_g < NB; cbank_g = cbank_g + 1) begin : g_cred_sel
            l1d_pf3_mux2 #(.W (CRED_W + 1)) u_cred_sel (
                .sel (m_win[cbank_g]),
                .in1 (cred_win_all[cbank_g*(CRED_W+1) +: CRED_W+1]),
                .in0 (cred_stay_all[cbank_g*(CRED_W+1) +: CRED_W+1]),
                .out (cred_pick[cbank_g*(CRED_W+1) +: CRED_W+1])
            );
            always @* begin : cred_split_proc
                credit_next[cbank_g*CRED_W +: CRED_W] =
                    cred_pick[cbank_g*(CRED_W+1) +: CRED_W]
                    & {CRED_W{~m_timeout}};
                ok_next[cbank_g] = cred_pick[cbank_g*(CRED_W+1) + CRED_W]
                                   | m_timeout;
            end // always cred_split_proc
        end // for cbank_g
    endgenerate

    always @(posedge clk) begin : credit_proc
        if (rst) begin
            credit_q    <= {(NB*CRED_W){1'b0}};
            credit_ok_q <= {NB{1'b1}};
            wait_q      <= {WAIT_W{1'b0}};
        end
        else begin
            credit_q    <= credit_next;
            credit_ok_q <= ok_next;
            if (m_waits && !m_timeout) begin
                wait_q <= wait_q + 1'b1;
            end
            else begin
                wait_q <= {WAIT_W{1'b0}};
            end // else (no wait)
        end // else (run)
    end // always credit_proc

`ifdef L1D_DEBUG
    // ---- watch-dogs (V-9.3), simulation only ----
    always @(posedge clk) begin : debug_proc
        if (!rst) begin
            if (t_valid && t_hold_vld) begin
                $display("[%0t] L1D PF3_RANK ERROR stream offer while held",
                         $time);
            end
            if (m_valid && m_hold_vld) begin
                $display("[%0t] L1D PF3_RANK ERROR SMS offer while held",
                         $time);
            end
            // the split compares assume line = pick ? scan : direct
            if (t_valid && t_line !=
                (t_pick_scan ? t_line_scan : t_line_direct)) begin
                $display("[%0t] L1D PF3_RANK ERROR stream line split",
                         $time);
            end
            if (m_valid && m_line !=
                (m_pick_scan ? m_line_scan : m_line_direct)) begin
                $display("[%0t] L1D PF3_RANK ERROR SMS line split",
                         $time);
            end
            if ((s_win & t_win) != {NB{1'b0}} || (s_win & m_win) != {NB{1'b0}}
                || (t_win & m_win) != {NB{1'b0}}) begin
                $display("[%0t] L1D PF3_RANK ERROR two winners for a bank",
                         $time);
            end
        end // if (!rst)
    end // always debug_proc
`endif

endmodule

`default_nettype wire
`endif // L1D_PF3_RANK_V
