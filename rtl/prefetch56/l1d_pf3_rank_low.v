// ==========================================================================
// l1d_pf3_rank_low.v - the stream and SMS decisions of l1d_pf3_rank for
//                      one given stride offer (one copy per stride case)
// ==========================================================================
// IN SIMPLE WORDS : the rank sends, per bank, the stride offer first, then
//   the stream offer, then the SMS offer. Whether the stream and SMS
//   offers are sent, kept (held) or dropped as duplicates depends on the
//   stride offer. The stride's "I have a new offer" bit (s_valid) comes
//   late in the cycle, so the rank works these decisions out twice, once
//   as if the stride offer is new and once as if it is the held one, and
//   s_valid only picks one of the two results at the end. This module is
//   that decision for one stride case; the rank has two copies of it.
//
// STAGE / CYCLE   : S3, combinational, inside l1d_pf3_rank.
// INSTANTIATED BY : l1d_pf3_rank, u_low_new (stride offer = the new
//   candidate) and u_low_held (stride offer = the held one), 2 copies.
//
// PARAMETERS:
//   name      default unit    meaning
//   NB        4       banks   cache banks
//   HOLD_MAX  2       cycles  decision cycles of an offer (>= 1)
//
// PORTS (default widths; "this case" = the stride offer this copy assumes):
//   pf_enable              in  1    prefetch on
//   pf_req_ready           in  4    bank b takes a hint (flop)
//   credit_ok              in  4    bank b has an SMS credit (flop)
//   s_want                 in  4    this case: the stride offer's bank
//   s_off                  in  1    this case: the stride offers
//   s_eq_tn, s_eq_th       in  1    this case: stride line == new / held
//                                   stream line
//   s_eq_mn, s_eq_mh       in  1    this case: stride line == new / held
//                                   SMS line
//   t_valid, t_bank        in  1, 4 new stream offer (late valid) and bank
//   t_hold_vld, t_hold_bank, t_hold_cnt  in 1, 4, 2  held stream offer
//   m_valid, m_bank        in  1, 4 new SMS offer (late valid) and bank
//   m_hold_vld, m_hold_bank, m_hold_cnt  in 1, 4, 2  held SMS offer
//   tn_mn_eq, th_mn_eq     in  1    new / held stream line == new SMS line
//                                   (th_*: 0 when no stream offer is held)
//   tn_mh_eq, th_mh_eq     in  1    the same with the held SMS line
//   mn_waits, mh_waits     in  1    new / held SMS offer has no credit
//   t_win, m_win           out 4    stream / SMS offer sent to bank b
//   t_base, t_dup          out 1    stream offer of this case (new when
//                                   t_valid, else held): would be kept
//                                   (not sent, cycles left) / is a
//                                   duplicate of the stride line; it is
//                                   held when t_base AND NOT t_dup
//   mn_base, mn_dup        out 1    the same for the new SMS offer
//   mh_base, mh_dup        out 1    the same for the held SMS offer (the
//                                   rank's m_valid picks, in l1d_pf3_keep_or)
//
// WORKED EXAMPLE: this case s_want = 0001, s_eq_tn = 1, t_valid = 1, t_bank
//   = 0001: the stream offer is the stride's line -> dropped (t_dup = 1,
//   t_win = 0000). With s_eq_tn = 0, bank 0 ready: the stride takes bank
//   0, the stream offer waits: t_base = 1, t_dup = 0 (HOLD_MAX = 2).
//
// LOGIC LEVELS (T-9, estimate): bank AND / rank chain 2, any-sent OR 2,
//   base AND 1 = about 5, all from flops and the stride case inputs; the
//   duplicate bits are 1 to 2 levels after the line compares (the keep
//   AND-NOT is in l1d_pf3_keep_or, last before the hold flops).
//
// ASSUMPTIONS (V-9.2): A1 t_valid only when t_hold_vld is 0, m_valid only
//   when m_hold_vld is 0 (as l1d_pf3_rank A1).
// ==========================================================================
`ifndef L1D_PF3_RANK_LOW_V
`define L1D_PF3_RANK_LOW_V
`default_nettype none

module l1d_pf3_rank_low #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter NB       = 4, // default 4 banks
    parameter HOLD_MAX = 2  // default 2 decision cycles
) (
    // ---- control ----
    input  wire                         pf_enable,    // 1 = on
    input  wire [NB-1:0]                pf_req_ready, // 4: bank takes a hint
    input  wire [NB-1:0]                credit_ok,    // 4: SMS credit left
    // ---- the stride offer of this case ----
    input  wire [NB-1:0]                s_want,       // 4: its bank
    input  wire                         s_off,        // it offers
    input  wire                         s_eq_tn,      // = new stream line
    input  wire                         s_eq_th,      // = held stream line
    input  wire                         s_eq_mn,      // = new SMS line
    input  wire                         s_eq_mh,      // = held SMS line
    // ---- stream offer ----
    input  wire                         t_valid,      // new offer (late)
    input  wire [NB-1:0]                t_bank,       // 4: its bank
    input  wire                         t_hold_vld,   // an offer is held
    input  wire [NB-1:0]                t_hold_bank,  // 4: its bank
    input  wire [$clog2(HOLD_MAX+1)-1:0] t_hold_cnt,  // HOLD_W = 2 bits
    // ---- SMS offer ----
    input  wire                         m_valid,      // new offer (late)
    input  wire [NB-1:0]                m_bank,       // 4: its bank
    input  wire                         m_hold_vld,   // an offer is held
    input  wire [NB-1:0]                m_hold_bank,  // 4: its bank
    input  wire [$clog2(HOLD_MAX+1)-1:0] m_hold_cnt,  // HOLD_W = 2 bits
    // ---- stream / SMS line compares and SMS credit waits ----
    input  wire                         tn_mn_eq,     // new stream = new SMS
    input  wire                         th_mn_eq,     // held stream = new SMS
    input  wire                         tn_mh_eq,     // new stream = held SMS
    input  wire                         th_mh_eq,     // held stream = held SMS
    input  wire                         mn_waits,     // new SMS: no credit
    input  wire                         mh_waits,     // held SMS: no credit
    // ---- decisions ----
    output wire [NB-1:0]                t_win,        // 4: stream sent to b
    output wire [NB-1:0]                m_win,        // 4: SMS sent to b
    output wire                         t_base,       // stream: would keep
    output wire                         t_dup,        // stream: duplicate
    output wire                         mn_base,      // new SMS: would keep
    output wire                         mn_dup,       // new SMS: duplicate
    output wire                         mh_base,      // held SMS: would keep
    output wire                         mh_dup        // held SMS: duplicate
);

    // ---- local constants (value at the default) ----
    localparam HOLD_W = $clog2(HOLD_MAX + 1);          // 2 bits
    localparam [HOLD_W-1:0] HOLD_LAST = HOLD_MAX - 1;  // last chance count
    localparam [HOLD_W-1:0] HOLD_NEW  = 0;             // count of a new offer

    // ---- signals ----
    wire [NB-1:0] t_want_n;   // 4: new stream offer's bank (on)
    wire [NB-1:0] t_want_h;   // 4: held stream offer's bank (on, held)
    wire [NB-1:0] t_want;     // 4: the stream offer's bank
    wire          tn_dup;     // new stream offer = stride offer
    wire          th_dup;     // held stream offer = stride offer
    wire          tn_base;    // new stream offer would stay held
    wire          th_base;    // held stream offer would stay held
    wire [NB-1:0] m_ok;       // 4: SMS may use bank b (credit, ready, on,
                              //   no stride offer there)
    wire [NB-1:0] mn_win;     // 4: new SMS offer sent to bank b
    wire [NB-1:0] mh_win;     // 4: held SMS offer sent to bank b

    // ======================================================================
    // stream: sent below the stride, dropped when it is the stride's line,
    // else held while it has decision cycles left
    // ======================================================================
    assign t_want_n = t_bank & {NB{pf_enable}};
    assign t_want_h = t_hold_bank & {NB{t_hold_vld & pf_enable}};
    assign t_want   = t_valid ? t_want_n : t_want_h;
    assign t_win    = t_want & pf_req_ready & ~s_want;
    assign tn_dup   = s_off & s_eq_tn;
    assign th_dup   = s_off & t_hold_vld & s_eq_th;
    assign tn_base  = ~(|(t_want_n & pf_req_ready & ~s_want))
                      & (HOLD_NEW != HOLD_LAST);
    assign th_base  = t_hold_vld & ~(|(t_want_h & pf_req_ready & ~s_want))
                      & (t_hold_cnt != HOLD_LAST);
    assign t_base   = t_valid ? tn_base : th_base;
    assign t_dup    = t_valid ? tn_dup : th_dup;

    // ======================================================================
    // SMS: sent below stride and stream with a credit, dropped when it is a
    // higher offer's line, else held (no ageing while it waits for credit)
    // ======================================================================
    assign m_ok    = credit_ok & {NB{pf_enable}} & pf_req_ready & ~s_want
                     & ~t_want;
    assign mn_win  = m_bank & m_ok;
    assign mh_win  = m_hold_bank & {NB{m_hold_vld}} & m_ok;
    assign m_win   = m_valid ? mn_win : mh_win;
    assign mn_dup  = (s_off & s_eq_mn) | (t_valid ? tn_mn_eq : th_mn_eq);
    assign mh_dup  = m_hold_vld
                     & ((s_off & s_eq_mh) | (t_valid ? tn_mh_eq : th_mh_eq));
    assign mn_base = ~(|mn_win) & (mn_waits | (HOLD_NEW != HOLD_LAST));
    assign mh_base = m_hold_vld & ~(|mh_win)
                     & (mh_waits | (m_hold_cnt != HOLD_LAST));

endmodule

`default_nettype wire
`endif // L1D_PF3_RANK_LOW_V
