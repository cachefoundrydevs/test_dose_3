`ifndef L1D_PF3_RANK_V
`define L1D_PF3_RANK_V
`default_nettype none
module l1d_pf3_rank #(
    parameter NB              = 4,
    parameter LINE_W          = 58,
    parameter HOLD_MAX        = 2,
    parameter CREDIT_PER_BANK = 2,
    parameter CREDIT_WAIT_MAX = 64
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 pf_enable,
    input  wire [NB-1:0]        pf_req_ready,
    input  wire                 s_valid,
    input  wire [NB:0]          s_valid_dp,
    input  wire [LINE_W-1:0]    s_line,
    input  wire [NB-1:0]        s_bank,
    input  wire                 s_unique,
    input  wire                 t_valid,
    input  wire [NB:0]          t_valid_dp,
    input  wire [LINE_W-1:0]    t_line,
    input  wire [LINE_W-1:0]    t_line_scan,
    input  wire [LINE_W-1:0]    t_line_direct,
    input  wire                 t_pick_scan,
    input  wire [NB-1:0]        t_bank,
    input  wire                 t_unique,
    input  wire                 m_valid,
    input  wire                 m_valid_dp,
    input  wire [LINE_W-1:0]    m_line,
    input  wire [LINE_W-1:0]    m_line_scan,
    input  wire [LINE_W-1:0]    m_line_direct,
    input  wire                 m_pick_scan,
    input  wire [NB-1:0]        m_bank,
    input  wire [NB-1:0]        fb_valid,
    output wire                 t_held,
    output wire                 m_held,
    output reg  [NB-1:0]        pf_req_valid,
    output reg  [NB*LINE_W-1:0] pf_req_line,
    output reg  [NB-1:0]        pf_req_unique
);
    localparam HOLD_W = $clog2(HOLD_MAX + 1);
    localparam CRED_W = $clog2(CREDIT_PER_BANK + 1);
    localparam WAIT_W = $clog2(CREDIT_WAIT_MAX + 1);
    localparam [HOLD_W-1:0] HOLD_ONE  = 1;
    localparam [HOLD_W-1:0] HOLD_LAST = HOLD_MAX - 1;
    localparam [CRED_W-1:0] CRED_MAX  = CREDIT_PER_BANK;
    localparam LOW_W  = 2*NB;
    localparam KB_W   = 6;
    localparam CASE_N = 4;
    localparam [WAIT_W-1:0] WAIT_LAST = CREDIT_WAIT_MAX;
    localparam MEQ_N  = 5;
    localparam TEQ_N  = 4;
    reg                  s_hold_vld;
    reg  [LINE_W-1:0]    s_hold_line;
    reg  [NB-1:0]        s_hold_bank;
    reg                  s_hold_uniq;
    reg  [HOLD_W-1:0]    s_hold_cnt;
    reg                  t_hold_vld;
    reg  [LINE_W-1:0]    t_hold_line;
    reg  [NB-1:0]        t_hold_bank;
    reg                  t_hold_uniq;
    reg  [HOLD_W-1:0]    t_hold_cnt;
    reg                  m_hold_vld;
    reg  [LINE_W-1:0]    m_hold_line;
    reg  [NB-1:0]        m_hold_bank;
    reg  [HOLD_W-1:0]    m_hold_cnt;
    reg  [NB*CRED_W-1:0] credit_q;
    reg  [NB-1:0]        credit_ok_q;
    reg  [WAIT_W-1:0]    wait_q;
    reg  [NB*CRED_W-1:0] credit_next;
    reg  [CRED_W-1:0]    one_credit;
    reg  [CRED_W-1:0]    credit_win;
    reg  [CRED_W-1:0]    credit_stay;
    reg  [NB-1:0]        ok_next;
    reg  [NB*(CRED_W+1)-1:0] cred_win_all;
    reg  [NB*(CRED_W+1)-1:0] cred_stay_all;
    wire [NB*(CRED_W+1)-1:0] cred_pick;
    wire                 s_off;
    wire [NB-1:0]        s_off_bank;
    wire                 s_off_uniq;
    wire [HOLD_W-1:0]    s_off_cnt;
    wire [NB-1:0]        t_off_bank;
    wire                 t_off_uniq;
    wire [HOLD_W-1:0]    t_off_cnt;
    wire [LINE_W-1:0]    m_off_line;
    wire [HOLD_W-1:0]    m_off_cnt;
    wire [NB-1:0]        s_want;
    wire [NB-1:0]        t_want;
    wire [NB-1:0]        s_win;
    wire [NB-1:0]        t_win;
    wire [NB-1:0]        m_win;
    wire                 s_taken;
    wire                 s_keep;
    wire                 t_keep;
    wire                 m_keep;
    wire                 m_waits;
    wire [NB-1:0]        s_want_n;
    wire [NB-1:0]        s_want_h;
    wire                 sn_eq_tn;
    wire                 sn_eq_th;
    wire                 sn_eq_mn;
    wire                 sn_eq_mh;
    wire                 sh_eq_tn;
    wire                 sh_eq_th;
    wire                 sh_eq_mn;
    wire                 sh_eq_mh;
    wire                 tn_mn_eq;
    wire                 th_mn_eq;
    wire                 tn_mh_eq;
    wire                 th_mh_eq;
    wire [MEQ_N-1:0]     meq_scan;
    wire [MEQ_N-1:0]     meq_direct;
    wire [MEQ_N-1:0]     meq_pick;
    wire [TEQ_N-1:0]     teq_scan;
    wire [TEQ_N-1:0]     teq_direct;
    wire [TEQ_N-1:0]     teq_pick;
    wire                 ts_mn_eq;
    wire                 td_mn_eq;
    wire                 mn_waits;
    wire                 mh_waits;
    wire [CASE_N*LOW_W-1:0] low_case;
    wire [CASE_N*KB_W-1:0] keep_case;
    reg  [CASE_N-1:0]    t_keep_go;
    reg  [CASE_N-1:0]    t_keep_drop;
    reg  [2*CASE_N-1:0]  m_keep_go;
    reg  [2*CASE_N-1:0]  m_keep_drop;
    wire [CASE_N-1:0]    low_sel_oh;
    wire                 m_timeout;
    reg  [NB-1:0]        valid_next;
    wire [NB*LINE_W-1:0] line_pick;
    reg  [NB-1:0]        unique_next;
    reg  [LINE_W-1:0]    tm_new_t;
    reg  [LINE_W-1:0]    tm_old_t;
    reg  [NB*CASE_N*LINE_W-1:0] line_case;
    reg  [NB*CASE_N-1:0] line_sel_oh;
    wire [NB-1:0]        t_want_nb;
    wire [NB-1:0]        t_want_hb;
    assign s_off      = s_valid | s_hold_vld;
    assign s_off_bank = s_valid ? s_bank : (s_hold_bank & {NB{s_hold_vld}});
    assign s_off_uniq = s_valid_dp[NB] ? s_unique : s_hold_uniq;
    assign s_off_cnt  = s_valid ? {HOLD_W{1'b0}} : s_hold_cnt;
    assign t_off_bank = t_valid ? t_bank : (t_hold_bank & {NB{t_hold_vld}});
    assign t_off_uniq = t_valid_dp[NB] ? t_unique : t_hold_uniq;
    assign t_off_cnt  = t_valid ? {HOLD_W{1'b0}} : t_hold_cnt;
    assign m_off_line = m_valid_dp ? m_line : m_hold_line;
    assign m_off_cnt  = m_valid_dp ? {HOLD_W{1'b0}} : m_hold_cnt;
    assign s_want = s_off_bank & {NB{pf_enable}};
    assign t_want = t_off_bank & {NB{pf_enable}};
    assign t_want_nb = t_bank & {NB{pf_enable}};
    assign t_want_hb = t_hold_bank & {NB{t_hold_vld & pf_enable}};
    assign s_win  = s_want & pf_req_ready;
    assign s_taken = |s_win;
    assign s_want_n = s_bank & {NB{pf_enable}};
    assign s_want_h = s_hold_bank & {NB{s_hold_vld & pf_enable}};
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
    assign low_sel_oh = {s_valid & t_valid, ~s_valid & t_valid,
                         s_valid & ~t_valid, ~s_valid & ~t_valid};
    genvar tcase_g;
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
        end
    endgenerate
    l1d_pf3_mux4oh #(.W (LOW_W), .SEL_N (CASE_N)) u_low_sel (
        .sel_oh (low_sel_oh),
        .in3    (low_case[3*LOW_W +: LOW_W]),
        .in2    (low_case[2*LOW_W +: LOW_W]),
        .in1    (low_case[1*LOW_W +: LOW_W]),
        .in0    (low_case[0 +: LOW_W]),
        .out    ({t_win, m_win})
    );
    always @* begin : keep_go_proc
        integer case_idx;
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
        end
    end
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
        integer bank_idx;
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
            valid_next[bank_idx] = s_win[bank_idx] | t_win[bank_idx]
                                   | m_win[bank_idx];
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
        end
    end
    genvar lbank_g;
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
        end
    endgenerate
    always @(posedge clk) begin : out_reg_proc
        if (rst) begin
            pf_req_valid  <= {NB{1'b0}};
            pf_req_unique <= {NB{1'b0}};
        end
        else begin
            pf_req_valid  <= valid_next;
            pf_req_unique <= unique_next;
        end
        pf_req_line <= line_pick;
    end
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
        end
        s_hold_cnt <= s_off_cnt + HOLD_ONE;
        t_hold_cnt <= t_off_cnt + HOLD_ONE;
        m_hold_cnt <= m_waits ? m_off_cnt : (m_off_cnt + HOLD_ONE);
    end
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
    end
    assign t_held = t_hold_vld;
    assign m_held = m_hold_vld;
    assign mn_waits  = ~(|(m_bank & credit_ok_q)) & pf_enable;
    assign mh_waits  = m_hold_vld & ~(|(m_hold_bank & credit_ok_q)) & pf_enable;
    assign m_waits   = m_valid ? mn_waits : mh_waits;
    assign m_timeout = m_waits & (wait_q == WAIT_LAST);
    always @* begin : credit_next_proc
        integer bank_idx;
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
            one_credit = credit_q[bank_idx*CRED_W +: CRED_W];
            credit_win  = (fb_valid[bank_idx] && (one_credit != {CRED_W{1'b0}}))
                          ? one_credit : (one_credit + 1'b1);
            credit_stay = (fb_valid[bank_idx] && (one_credit != {CRED_W{1'b0}}))
                          ? (one_credit - 1'b1) : one_credit;
            cred_win_all[bank_idx*(CRED_W+1) +: CRED_W+1] =
                {(credit_win < CRED_MAX), credit_win};
            cred_stay_all[bank_idx*(CRED_W+1) +: CRED_W+1] =
                {(credit_stay < CRED_MAX), credit_stay};
        end
    end
    genvar cbank_g;
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
            end
        end
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
            end
        end
    end
`ifdef L1D_DEBUG
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
        end
    end
`endif
endmodule
`default_nettype wire
`endif
