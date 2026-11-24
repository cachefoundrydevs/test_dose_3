`ifndef L1D_PF3_RANK_LOW_V
`define L1D_PF3_RANK_LOW_V
`default_nettype none
module l1d_pf3_rank_low #(
    parameter NB       = 4,
    parameter HOLD_MAX = 2
) (
    input  wire                         pf_enable,
    input  wire [NB-1:0]                pf_req_ready,
    input  wire [NB-1:0]                credit_ok,
    input  wire [NB-1:0]                s_want,
    input  wire                         s_off,
    input  wire                         s_eq_tn,
    input  wire                         s_eq_th,
    input  wire                         s_eq_mn,
    input  wire                         s_eq_mh,
    input  wire                         t_valid,
    input  wire [NB-1:0]                t_bank,
    input  wire                         t_hold_vld,
    input  wire [NB-1:0]                t_hold_bank,
    input  wire [$clog2(HOLD_MAX+1)-1:0] t_hold_cnt,
    input  wire                         m_valid,
    input  wire [NB-1:0]                m_bank,
    input  wire                         m_hold_vld,
    input  wire [NB-1:0]                m_hold_bank,
    input  wire [$clog2(HOLD_MAX+1)-1:0] m_hold_cnt,
    input  wire                         tn_mn_eq,
    input  wire                         th_mn_eq,
    input  wire                         tn_mh_eq,
    input  wire                         th_mh_eq,
    input  wire                         mn_waits,
    input  wire                         mh_waits,
    output wire [NB-1:0]                t_win,
    output wire [NB-1:0]                m_win,
    output wire                         t_base,
    output wire                         t_dup,
    output wire                         mn_base,
    output wire                         mn_dup,
    output wire                         mh_base,
    output wire                         mh_dup
);
    localparam HOLD_W = $clog2(HOLD_MAX + 1);
    localparam [HOLD_W-1:0] HOLD_LAST = HOLD_MAX - 1;
    localparam [HOLD_W-1:0] HOLD_NEW  = 0;
    wire [NB-1:0] t_want_n;
    wire [NB-1:0] t_want_h;
    wire [NB-1:0] t_want;
    wire          tn_dup;
    wire          th_dup;
    wire          tn_base;
    wire          th_base;
    wire [NB-1:0] m_ok;
    wire [NB-1:0] mn_win;
    wire [NB-1:0] mh_win;
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
`endif
