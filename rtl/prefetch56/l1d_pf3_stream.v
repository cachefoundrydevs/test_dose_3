`ifndef L1D_PF3_STREAM_V
`define L1D_PF3_STREAM_V
`default_nettype none
module l1d_pf3_stream #(
    parameter NB          = 4,
    parameter ADDR_W      = 64,
    parameter PA_W        = 56,
    parameter LINE_BYTES  = 64,
    parameter PAGE_BYTES  = 4096,
    parameter STR_ENTRIES = 16,
    parameter TAG_W       = 12,
    parameter DEPTH       = 4,
    parameter CONF_ON     = 2,
    parameter CONF_MAX    = 3,
    parameter RECENT_N    = 4
) (
    input  wire                      clk,
    input  wire                      rst,
    input  wire                      pf_enable,
    input  wire [NB-1:0]             train_valid,
    input  wire [NB-1:0]             train_no_train,
    input  wire [NB*ADDR_W-1:0]      train_addr,
    input  wire [NB-1:0]             train_is_store,
    input  wire [NB-1:0]             train_miss,
    input  wire [NB-1:0]             train_pf_hit,
    input  wire                      held,
    output wire                      cand_valid,
    output wire [NB:0]               cand_valid_dp,
    output wire [ADDR_W-$clog2(LINE_BYTES)-1:0] cand_line,
    output wire [ADDR_W-$clog2(LINE_BYTES)-1:0] cand_line_scan,
    output wire [ADDR_W-$clog2(LINE_BYTES)-1:0] cand_line_direct,
    output wire                      cand_pick_scan,
    output reg  [NB-1:0]             cand_bank,
    output wire                      cand_unique
);
    localparam NC         = STR_ENTRIES;
    localparam OFFSET_W   = $clog2(LINE_BYTES);
    localparam PAGE_W     = $clog2(PAGE_BYTES);
    localparam LIP_W      = PAGE_W - OFFSET_W;
    localparam PAGE_NUM_W = PA_W - PAGE_W;
    localparam PAD_W      = ADDR_W - PA_W;
    localparam CONF_W     = $clog2(CONF_MAX + 1);
    localparam NCLS       = CONF_MAX + 2;
    localparam BANK_W     = $clog2(NB);
    localparam WIDE_W     = LIP_W + 2;
    localparam [LIP_W-1:0] LIP_TWO = 2;
    localparam EC_N       = 4;
    localparam OV_N       = NB + 7;
    localparam SG_N       = 5;
    localparam CPC        = NC / EC_N;
    localparam EC_W       = 5 * LIP_W + 4;
    localparam [WIDE_W-1:0] DEPTH_WIDE = DEPTH;
    localparam [LIP_W-1:0]  DEPTH_LIP  = DEPTH;
    localparam [WIDE_W-1:0] LAST_WIDE  = (1 << LIP_W) - 1;
    localparam [LIP_W-1:0]  LAST_LINE  = (1 << LIP_W) - 1;
    localparam [LIP_W-1:0]  LIP_ONE    = 1;
    reg  [NB-1:0]            lane_pick;
    reg                      lane_seen;
    reg  [NB*TAG_W-1:0]      lane_tag;
    wire [NB*LIP_W-1:0]      lane_line;
    wire [NB*(LIP_W+1)-1:0]  lane_line_p1;
    reg  [NB*LIP_W-1:0]      lane_tgt_up;
    reg  [NB*LIP_W-1:0]      lane_tgt_dn;
    reg  [WIDE_W-1:0]        line_wide;
    reg  [WIDE_W-1:0]        up_sum;
    reg  [LIP_W-1:0]         dn_diff;
    reg                      pick_any;
    reg  [TAG_W-1:0]         pick_tag;
    reg  [PAGE_NUM_W-1:0]    pick_page;
    reg  [LIP_W-1:0]         pick_line;
    reg  [LIP_W-1:0]         pick_tgt_up;
    reg  [LIP_W-1:0]         pick_tgt_dn;
    reg                      pick_trig;
    reg                      pick_miss;
    reg                      pick_store;
    reg                      pick_same_tag;
    reg                      pick_same;
    reg                      pick_up;
    reg                      pick_dn;
    reg                      e1_vld;
    reg  [TAG_W-1:0]         e1_tag;
    reg  [PAGE_NUM_W-1:0]    e1_page;
    reg  [LIP_W-1:0]         e1_line;
    reg  [LIP_W:0]           e1_line_p1w;
    reg  [LIP_W-1:0]         e1_line_p1;
    reg  [LIP_W-1:0]         e1_line_m1;
    reg  [LIP_W-1:0]         e1_line_p2;
    reg  [LIP_W-1:0]         e1_line_m2;
    reg                      e1_top;
    reg                      e1_bot;
    reg  [LIP_W-1:0]         e1_tgt_dn;
    wire [EC_N*EC_W-1:0]     ec_word;
    wire [NC:0]              offer_rows;
    wire [EC_N-1:0]          cand_cp;
    wire [OV_N-1:0]          offer_cp;
    wire [EC_N-1:0]          slot_go_cp;
    reg                      e1_trig;
    reg                      e1_miss;
    reg                      e1_store;
    reg  [NC-1:0]            e2_card;
    wire [NC-1:0]            card_hit;
    wire [NC-1:0]            card_pend;
    wire [NC*NCLS-1:0]       card_class;
    wire [NC-1:0]            card_dvld;
    wire [NC-1:0]            card_devp;
    wire [NC-1:0]            card_dup;
    wire [NC*LIP_W-1:0]      card_dline;
    wire [NC*LIP_W-1:0]      card_dlp1;
    wire [NC*LIP_W-1:0]      card_dlm1;
    wire [NC-1:0]            card_ddup;
    wire [NC-1:0]            card_dstore;
    wire [NC-1:0]            card_sgo;
    wire [NC*LIP_W-1:0]      card_sline;
    wire [NC*LIP_W-1:0]      card_slp1;
    wire [NC*LIP_W-1:0]      card_slm1;
    wire [NC-1:0]            card_sdup;
    wire [NC*PAGE_NUM_W-1:0] card_page;
    wire [NC*TAG_W-1:0]      card_tag;
    wire [NC-1:0]            card_store;
    wire [NC-1:0]            card_alloc;
    wire [NC-1:0]            prev_card;
    wire                     any_hit;
    wire                     alloc;
    reg  [NCLS*NC-1:0]       class_cards;
    reg  [NCLS-1:0]          class_lower;
    wire [NCLS*NC-1:0]       class_first;
    wire [NCLS-1:0]          class_any;
    reg  [NC-1:0]            victim;
    reg  [RECENT_N-1:0]       recent_vld;
    reg  [RECENT_N*TAG_W-1:0] recent_tag;
    reg  [RECENT_N*LIP_W-1:0] recent_line;
    reg  [RECENT_N*LIP_W-1:0] recent_lp1;
    reg  [RECENT_N*LIP_W-1:0] recent_lm1;
    wire [RECENT_N-1:0]       win_tag_eq;
    wire [RECENT_N-1:0]       win_ev_p1;
    wire [RECENT_N-1:0]       win_ev_m1;
    wire                      ev_dup_p1;
    wire                      ev_dup_m1;
    reg  [NC-1:0]            rr_mask_q;
    reg  [NC-1:0]            sel_q;
    reg                      sel_vld_q;
    reg  [NC-1:0]            pick_high;
    reg  [NC-1:0]            pick_base;
    reg  [NC-1:0]            pick_one;
    reg  [NC-1:0]            pick_above;
    reg                      pick_seen;
    wire                     slot_free;
    wire                     scan_take;
    reg                      s_go;
    reg  [LIP_W-1:0]         s_line;
    reg                      s_dup;
    reg  [PAGE_NUM_W-1:0]    s_page;
    reg  [TAG_W-1:0]         s_tag;
    reg                      s_store;
    reg  [LIP_W-1:0]         d_line;
    reg                      d_evp;
    reg                      d_up;
    reg  [LIP_W-1:0]         d_kp;
    reg  [LIP_W-1:0]         d_kp_p1;
    reg  [LIP_W-1:0]         d_kp_m1;
    reg  [LIP_W-1:0]         d_lp1;
    reg  [LIP_W-1:0]         d_lm1;
    reg  [LIP_W-1:0]         s_lp1;
    reg  [LIP_W-1:0]         s_lm1;
    reg  [LIP_W-1:0]         d_lp1_q;
    reg  [LIP_W-1:0]         d_lm1_q;
    reg  [LIP_W-1:0]         s_lp1_q;
    reg  [LIP_W-1:0]         s_lm1_q;
    reg                      d_store;
    reg  [NC-1:0]            dvld_q;
    reg  [NC-1:0]            ddup_q;
    wire [NC-1:0]            doffer;
    reg  [LIP_W-1:0]         d_line_q;
    reg                      d_store_q;
    reg  [TAG_W-1:0]         e2_tag;
    reg  [PAGE_NUM_W-1:0]    e2_page;
    reg  [LIP_W-1:0]         e2_line;
    reg  [LIP_W:0]           e2_line_p1w;
    reg  [LIP_W-1:0]         e2_tgt_dn;
    reg                      e2_store;
    reg                      sdup_q;
    wire                     soffer;
    wire                     soffer_b;
    wire [SG_N-1:0]          sgo_cp;
    wire [NC:0]              offer_rows_b;
    reg  [LIP_W-1:0]         s_line_q;
    reg  [PAGE_NUM_W-1:0]    s_page_q;
    reg  [TAG_W-1:0]         s_tag_q;
    reg                      s_store_q;
    wire [TAG_W-1:0]         offer_tag;
    wire [LIP_W-1:0]         offer_line;
    wire [LIP_W-1:0]         offer_lp1;
    wire [LIP_W-1:0]         offer_lm1;
    genvar lane_g;
    generate
        for (lane_g = 0; lane_g < NB; lane_g = lane_g + 1) begin : g_lane
            assign lane_line[lane_g*LIP_W +: LIP_W] =
                train_addr[lane_g*ADDR_W + OFFSET_W +: LIP_W];
            assign lane_line_p1[lane_g*(LIP_W+1) +: LIP_W+1] =
                {1'b0, train_addr[lane_g*ADDR_W + OFFSET_W +: LIP_W]}
                + {{LIP_W{1'b0}}, 1'b1};
        end
    endgenerate
    always @* begin : lane_proc
        integer lane_idx;
        integer bit_idx;
        lane_tag = {(NB*TAG_W){1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            for (bit_idx = 0; bit_idx < PAGE_NUM_W; bit_idx = bit_idx + 1) begin
                lane_tag[lane_idx*TAG_W + (bit_idx % TAG_W)] =
                    lane_tag[lane_idx*TAG_W + (bit_idx % TAG_W)]
                    ^ train_addr[lane_idx*ADDR_W + PAGE_W + bit_idx];
            end
            line_wide = {{(WIDE_W-LIP_W){1'b0}},
                         lane_line[lane_idx*LIP_W +: LIP_W]};
            up_sum    = line_wide + DEPTH_WIDE;
            lane_tgt_up[lane_idx*LIP_W +: LIP_W] =
                (up_sum > LAST_WIDE) ? LAST_LINE : up_sum[LIP_W-1:0];
            dn_diff   = lane_line[lane_idx*LIP_W +: LIP_W] - DEPTH_LIP;
            lane_tgt_dn[lane_idx*LIP_W +: LIP_W] =
                (line_wide >= DEPTH_WIDE) ? dn_diff
                                          : {LIP_W{1'b0}};
        end
    end
    always @* begin : pick_proc
        integer lane_idx;
        lane_seen = 1'b0;
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_pick[lane_idx] = train_valid[lane_idx]
                                  & ~train_no_train[lane_idx] & ~lane_seen;
            lane_seen = lane_seen
                        | (train_valid[lane_idx] & ~train_no_train[lane_idx]);
        end
        pick_any      = 1'b0;
        pick_tag      = {TAG_W{1'b0}};
        pick_page     = {PAGE_NUM_W{1'b0}};
        pick_line     = {LIP_W{1'b0}};
        pick_tgt_up   = {LIP_W{1'b0}};
        pick_tgt_dn   = {LIP_W{1'b0}};
        pick_trig     = 1'b0;
        pick_miss     = 1'b0;
        pick_store    = 1'b0;
        pick_same_tag = 1'b0;
        pick_same     = 1'b0;
        pick_up       = 1'b0;
        pick_dn       = 1'b0;
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            pick_any    = pick_any | lane_pick[lane_idx];
            pick_tag    = pick_tag | ({TAG_W{lane_pick[lane_idx]}}
                          & lane_tag[lane_idx*TAG_W +: TAG_W]);
            pick_page   = pick_page | ({PAGE_NUM_W{lane_pick[lane_idx]}}
                          & train_addr[lane_idx*ADDR_W + PAGE_W +: PAGE_NUM_W]);
            pick_line   = pick_line | ({LIP_W{lane_pick[lane_idx]}}
                          & lane_line[lane_idx*LIP_W +: LIP_W]);
            pick_tgt_up = pick_tgt_up | ({LIP_W{lane_pick[lane_idx]}}
                          & lane_tgt_up[lane_idx*LIP_W +: LIP_W]);
            pick_tgt_dn = pick_tgt_dn | ({LIP_W{lane_pick[lane_idx]}}
                          & lane_tgt_dn[lane_idx*LIP_W +: LIP_W]);
            pick_trig   = pick_trig | (lane_pick[lane_idx]
                          & (train_miss[lane_idx] | train_pf_hit[lane_idx]));
            pick_miss   = pick_miss | (lane_pick[lane_idx] & train_miss[lane_idx]);
            pick_store  = pick_store | (lane_pick[lane_idx]
                          & train_is_store[lane_idx]);
            pick_same_tag = pick_same_tag | (lane_pick[lane_idx]
                & (lane_tag[lane_idx*TAG_W +: TAG_W] == e1_tag));
            pick_same = pick_same | (lane_pick[lane_idx]
                & (lane_line[lane_idx*LIP_W +: LIP_W] == e1_line));
            pick_up   = pick_up | (lane_pick[lane_idx]
                & ({1'b0, lane_line[lane_idx*LIP_W +: LIP_W]} == e1_line_p1w));
            pick_dn   = pick_dn | (lane_pick[lane_idx]
                & (lane_line_p1[lane_idx*(LIP_W+1) +: LIP_W+1]
                   == {1'b0, e1_line}));
        end
    end
    always @(posedge clk) begin : s1_reg_proc
        if (rst) begin
            e1_vld <= 1'b0;
        end
        else begin
            e1_vld <= pick_any & pf_enable;
        end
        e1_tag      <= pick_tag;
        e1_page     <= pick_page;
        e1_line     <= pick_line;
        e1_line_p1w <= {1'b0, pick_line} + {{LIP_W{1'b0}}, 1'b1};
        e1_line_p1  <= pick_line + LIP_ONE;
        e1_line_m1  <= pick_line - LIP_ONE;
        e1_line_p2  <= pick_line + LIP_TWO;
        e1_line_m2  <= pick_line - LIP_TWO;
        e1_top      <= (pick_line == LAST_LINE);
        e1_bot      <= (pick_line == {LIP_W{1'b0}});
        e1_tgt_dn   <= pick_tgt_dn;
        e1_trig     <= pick_trig;
        e1_miss     <= pick_miss;
        e1_store    <= pick_store;
    end
    assign offer_tag  = sgo_cp[3] ? s_tag_q  : e2_tag;
    assign offer_line = sgo_cp[3] ? s_line_q : d_line_q;
    assign offer_lp1 = sgo_cp[3] ? s_lp1_q : d_lp1_q;
    assign offer_lm1 = sgo_cp[3] ? s_lm1_q : d_lm1_q;
    genvar key_g;
    generate
        for (key_g = 0; key_g < RECENT_N; key_g = key_g + 1) begin : g_win_eq
            assign win_tag_eq[key_g] = (recent_tag[key_g*TAG_W +: TAG_W] == e1_tag);
            assign win_ev_p1[key_g] = recent_vld[key_g] & win_tag_eq[key_g]
                & (recent_line[key_g*LIP_W +: LIP_W] == e1_line_p1);
            assign win_ev_m1[key_g] = recent_vld[key_g] & win_tag_eq[key_g]
                & (recent_line[key_g*LIP_W +: LIP_W] == e1_line_m1);
        end
    endgenerate
    assign ev_dup_p1 = |win_ev_p1;
    assign ev_dup_m1 = |win_ev_m1;
    genvar ecopy_g;
    generate
        for (ecopy_g = 0; ecopy_g < EC_N; ecopy_g = ecopy_g + 1) begin : g_ecopy
            l1d_pf3_or_rows #(.W (1), .N (NC + 1)) u_cand_or (
                .rows   (offer_rows),
                .or_out (cand_cp[ecopy_g])
            );
            assign slot_go_cp[ecopy_g] = ~cand_cp[ecopy_g] & ~held & pf_enable;
            l1d_pf3_rep #(.W (EC_W), .HAS_RST (0), .HAS_EN (0)) u_ec (
                .clk (clk), .rst (1'b0), .en (1'b1),
                .d   ({pick_same_tag, pick_same, pick_up, pick_dn,
                       pick_line, pick_line + LIP_ONE, pick_line - LIP_ONE,
                       pick_tgt_up, pick_tgt_dn}),
                .q   (ec_word[ecopy_g*EC_W +: EC_W])
            );
        end
    endgenerate
    genvar card_g;
    generate
        for (card_g = 0; card_g < NC; card_g = card_g + 1) begin : g_card
            assign prev_card[card_g]  = e2_card[card_g];
            assign card_alloc[card_g] = alloc & victim[card_g];
            l1d_pf3_stream_card #(
                .NB         (NB),
                .TAG_W      (TAG_W),
                .PAGE_NUM_W (PAGE_NUM_W),
                .LIP_W      (LIP_W),
                .CONF_W     (CONF_W),
                .CONF_ON    (CONF_ON),
                .CONF_MAX   (CONF_MAX),
                .RECENT_N   (RECENT_N),
                .DEPTH      (DEPTH)
            ) u_card (
                .clk           (clk),
                .rst           (rst),
                .flush         (~pf_enable),
                .lane_pick     (lane_pick),
                .lane_tag      (lane_tag),
                .lane_line     (lane_line),
                .lane_line_p1  (lane_line_p1),
                .ev_vld        (e1_vld),
                .ev_page       (e1_page),
                .ev_line       (ec_word[(card_g/CPC)*EC_W + 4*LIP_W +: LIP_W]),
                .ev_line_p1    (ec_word[(card_g/CPC)*EC_W + 3*LIP_W +: LIP_W]),
                .ev_line_m1    (ec_word[(card_g/CPC)*EC_W + 2*LIP_W +: LIP_W]),
                .ev_top        (e1_top),
                .ev_bot        (e1_bot),
                .ev_tgt_up     (ec_word[(card_g/CPC)*EC_W + LIP_W +: LIP_W]),
                .ev_tgt_dn     (ec_word[(card_g/CPC)*EC_W +: LIP_W]),
                .ev_trig       (e1_trig),
                .prev_card     (prev_card[card_g]),
                .prev_same_tag (ec_word[(card_g/CPC)*EC_W + 5*LIP_W + 3]),
                .prev_same     (ec_word[(card_g/CPC)*EC_W + 5*LIP_W + 2]),
                .prev_up       (ec_word[(card_g/CPC)*EC_W + 5*LIP_W + 1]),
                .prev_dn       (ec_word[(card_g/CPC)*EC_W + 5*LIP_W]),
                .alloc_en      (card_alloc[card_g]),
                .p_tag         (e2_tag),
                .p_page        (e2_page),
                .p_line        (e2_line),
                .p_line_p1w    (e2_line_p1w),
                .p_tgt_dn      (e2_tgt_dn),
                .p_store       (e2_store),
                .scan_idle     (~sel_vld_q),
                .scan_sel      (sel_q[card_g]),
                .slot_go       (slot_go_cp[card_g/CPC]),
                .recent_tag_eq (win_tag_eq),
                .ev_dup_p1     (ev_dup_p1),
                .ev_dup_m1     (ev_dup_m1),
                .recent_lp1    (recent_lp1),
                .recent_lm1    (recent_lm1),
                .recent_tag    (recent_tag),
                .recent_vld    (recent_vld),
                .hit           (card_hit[card_g]),
                .pend_out      (card_pend[card_g]),
                .class_out     (card_class[card_g*NCLS +: NCLS]),
                .dir_vld       (card_dvld[card_g]),
                .dir_evp       (card_devp[card_g]),
                .dir_up        (card_dup[card_g]),
                .dir_kp        (card_dline[card_g*LIP_W +: LIP_W]),
                .dir_kp_p1     (card_dlp1[card_g*LIP_W +: LIP_W]),
                .dir_kp_m1     (card_dlm1[card_g*LIP_W +: LIP_W]),
                .dir_dup       (card_ddup[card_g]),
                .dir_store     (card_dstore[card_g]),
                .scan_go       (card_sgo[card_g]),
                .scan_line     (card_sline[card_g*LIP_W +: LIP_W]),
                .scan_lp1      (card_slp1[card_g*LIP_W +: LIP_W]),
                .scan_lm1      (card_slm1[card_g*LIP_W +: LIP_W]),
                .scan_dup      (card_sdup[card_g]),
                .page_out      (card_page[card_g*PAGE_NUM_W +: PAGE_NUM_W]),
                .tag_out       (card_tag[card_g*TAG_W +: TAG_W]),
                .store_out     (card_store[card_g])
            );
        end
    endgenerate
    assign any_hit = |card_hit;
    assign alloc   = e1_vld & ~any_hit & e1_miss;
    always @* begin : class_cards_proc
        integer card_idx;
        integer class_idx;
        for (class_idx = 0; class_idx < NCLS; class_idx = class_idx + 1) begin
            for (card_idx = 0; card_idx < NC; card_idx = card_idx + 1) begin
                class_cards[class_idx*NC + card_idx] =
                    card_class[card_idx*NCLS + class_idx];
            end
        end
    end
    genvar class_g;
    generate
        for (class_g = 0; class_g < NCLS; class_g = class_g + 1) begin : g_class
            l1d_pf3_ffs #(.W (NC)) u_class_first (
                .in_vec  (class_cards[class_g*NC +: NC]),
                .low_oh  (class_first[class_g*NC +: NC]),
                .low2_oh (),
                .any_set (class_any[class_g]),
                .two_set ()
            );
        end
    endgenerate
    always @* begin : victim_proc
        integer class_idx;
        class_lower[0] = 1'b0;
        for (class_idx = 1; class_idx < NCLS; class_idx = class_idx + 1) begin
            class_lower[class_idx] = class_lower[class_idx-1]
                | class_any[class_idx-1];
        end
        victim = {NC{1'b0}};
        for (class_idx = 0; class_idx < NCLS; class_idx = class_idx + 1) begin
            victim = victim | (class_first[class_idx*NC +: NC]
                               & {NC{~class_lower[class_idx]}});
        end
    end
    assign slot_free = ~offer_cp[NB+2] & ~held;
    assign scan_take = slot_free & pf_enable;
    always @* begin : offer_sel_proc
        integer card_idx;
        s_go    = 1'b0;
        s_line  = {LIP_W{1'b0}};
        s_dup   = 1'b0;
        s_page  = {PAGE_NUM_W{1'b0}};
        s_tag   = {TAG_W{1'b0}};
        s_store = 1'b0;
        d_evp   = 1'b0;
        d_up    = 1'b0;
        d_kp    = {LIP_W{1'b0}};
        d_kp_p1 = {LIP_W{1'b0}};
        d_kp_m1 = {LIP_W{1'b0}};
        s_lp1   = {LIP_W{1'b0}};
        s_lm1   = {LIP_W{1'b0}};
        d_store = 1'b0;
        for (card_idx = 0; card_idx < NC; card_idx = card_idx + 1) begin
            s_go    = s_go | card_sgo[card_idx];
            s_line  = s_line | ({LIP_W{sel_q[card_idx]}}
                      & card_sline[card_idx*LIP_W +: LIP_W]);
            s_lp1   = s_lp1 | ({LIP_W{sel_q[card_idx]}}
                      & card_slp1[card_idx*LIP_W +: LIP_W]);
            s_lm1   = s_lm1 | ({LIP_W{sel_q[card_idx]}}
                      & card_slm1[card_idx*LIP_W +: LIP_W]);
            s_dup   = s_dup | (sel_q[card_idx] & card_sdup[card_idx]);
            s_page  = s_page | ({PAGE_NUM_W{sel_q[card_idx]}}
                      & card_page[card_idx*PAGE_NUM_W +: PAGE_NUM_W]);
            s_tag   = s_tag | ({TAG_W{sel_q[card_idx]}}
                      & card_tag[card_idx*TAG_W +: TAG_W]);
            s_store = s_store | (sel_q[card_idx] & card_store[card_idx]);
            d_evp   = d_evp | (card_hit[card_idx] & card_devp[card_idx]);
            d_up    = d_up | (card_hit[card_idx] & card_dup[card_idx]);
            d_kp    = d_kp | ({LIP_W{card_hit[card_idx]}}
                      & card_dline[card_idx*LIP_W +: LIP_W]);
            d_kp_p1 = d_kp_p1 | ({LIP_W{card_hit[card_idx]}}
                      & card_dlp1[card_idx*LIP_W +: LIP_W]);
            d_kp_m1 = d_kp_m1 | ({LIP_W{card_hit[card_idx]}}
                      & card_dlm1[card_idx*LIP_W +: LIP_W]);
            d_store = d_store | (card_hit[card_idx] & card_dstore[card_idx]);
        end
        if (d_evp) begin
            d_line = d_up ? e1_line_p1 : e1_line_m1;
            d_lp1  = d_up ? e1_line_p2 : e1_line;
            d_lm1  = d_up ? e1_line    : e1_line_m2;
        end
        else begin
            d_line = d_kp;
            d_lp1  = d_kp_p1;
            d_lm1  = d_kp_m1;
        end
    end
    always @* begin : scan_pick_proc
        integer card_idx;
        pick_high = card_pend & rr_mask_q;
        pick_base = (|pick_high) ? pick_high : card_pend;
        pick_one  = {NC{1'b0}};
        pick_seen = 1'b0;
        for (card_idx = 0; card_idx < NC; card_idx = card_idx + 1) begin
            if (pick_base[card_idx] && !pick_seen) begin
                pick_one[card_idx] = 1'b1;
                pick_seen          = 1'b1;
            end
        end
        pick_above = {NC{1'b0}};
        pick_seen  = 1'b0;
        for (card_idx = 0; card_idx < NC; card_idx = card_idx + 1) begin
            pick_above[card_idx] = pick_seen;
            pick_seen            = pick_seen | pick_one[card_idx];
        end
    end
    always @(posedge clk) begin : s2_reg_proc
        if (rst || !pf_enable) begin
            sel_vld_q <= 1'b0;
            sel_q     <= {NC{1'b0}};
            rr_mask_q <= {NC{1'b1}};
            dvld_q    <= {NC{1'b0}};
            ddup_q    <= {NC{1'b0}};
            sdup_q    <= 1'b0;
            e2_card   <= {NC{1'b0}};
        end
        else begin
            if (!sel_vld_q || scan_take) begin
                sel_vld_q <= |card_pend;
                sel_q     <= pick_one;
                if (|card_pend) begin
                    rr_mask_q <= pick_above;
                end
            end
            sdup_q <= s_dup;
            dvld_q <= card_dvld;
            ddup_q <= card_ddup;
            e2_card  <= card_hit | card_alloc;
        end
        d_line_q  <= d_line;
        d_lp1_q   <= d_lp1;
        d_lm1_q   <= d_lm1;
        s_lp1_q   <= s_lp1;
        s_lm1_q   <= s_lm1;
        d_store_q <= d_store;
        e2_tag    <= e1_tag;
        e2_page   <= e1_page;
        e2_line   <= e1_line;
        e2_line_p1w <= e1_line_p1w;
        e2_tgt_dn <= e1_tgt_dn;
        e2_store  <= e1_store;
        s_line_q  <= s_line;
        s_page_q  <= s_page;
        s_tag_q   <= s_tag;
        s_store_q <= s_store;
    end
    l1d_pf3_rep #(.W(SG_N), .HAS_RST(1), .HAS_EN(0)) u_sgo (
        .clk (clk),
        .rst (rst | ~pf_enable),
        .en  (1'b1),
        .d   ({SG_N{scan_take & s_go}}),
        .q   (sgo_cp)
    );
    assign soffer       = sgo_cp[0] & ~sdup_q;
    assign soffer_b     = sgo_cp[1] & ~sdup_q;
    assign doffer       = dvld_q & ~ddup_q;
    assign offer_rows   = {soffer, doffer};
    assign offer_rows_b = {soffer_b, doffer};
    genvar ocopy_g;
    generate
        for (ocopy_g = 0; ocopy_g < OV_N; ocopy_g = ocopy_g + 1) begin : g_offer
            l1d_pf3_or_rows #(.W (1), .N (NC + 1)) u_offer_or (
                .rows   ((ocopy_g <= NB) ? offer_rows : offer_rows_b),
                .or_out (offer_cp[ocopy_g])
            );
        end
    endgenerate
    assign cand_valid    = offer_cp[0];
    assign cand_valid_dp = offer_cp[NB+1:1];
    assign cand_line   = {{PAD_W{1'b0}}, (sgo_cp[2] ? s_page_q : e2_page),
                          offer_line};
    assign cand_unique = sgo_cp[2] ? s_store_q : d_store_q;
    assign cand_line_scan   = {{PAD_W{1'b0}}, s_page_q, s_line_q};
    assign cand_line_direct = {{PAD_W{1'b0}}, e2_page, d_line_q};
    assign cand_pick_scan   = sgo_cp[4];
    always @* begin : bank_proc
        integer bank_idx;
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
            cand_bank[bank_idx] = (offer_line[BANK_W-1:0] == bank_idx[BANK_W-1:0]);
        end
    end
    always @(posedge clk) begin : recent_proc
        if (rst || !pf_enable) begin
            recent_vld <= {RECENT_N{1'b0}};
        end
        else if (offer_cp[NB+3]) begin
            recent_vld <= {recent_vld[RECENT_N-2:0], 1'b1};
        end
        if (offer_cp[NB+3]) begin
            recent_tag  <= {recent_tag[(RECENT_N-1)*TAG_W-1:0], offer_tag};
        end
        if (offer_cp[NB+4]) begin
            recent_line <= {recent_line[(RECENT_N-1)*LIP_W-1:0], offer_line};
        end
        if (offer_cp[NB+5]) begin
            recent_lp1  <= {recent_lp1[(RECENT_N-1)*LIP_W-1:0], offer_lp1};
        end
        if (offer_cp[NB+6]) begin
            recent_lm1  <= {recent_lm1[(RECENT_N-1)*LIP_W-1:0], offer_lm1};
        end
    end
`ifdef L1D_DEBUG
    always @(posedge clk) begin : debug_proc
        if (!rst) begin
            if ((card_hit & (card_hit - 1'b1)) != {NC{1'b0}}) begin
                $display("[%0t] L1D PF3_STREAM ERROR two cards hit", $time);
            end
            if (alloc && ((victim & (victim - 1'b1)) != {NC{1'b0}})) begin
                $display("[%0t] L1D PF3_STREAM ERROR victim not one-hot",
                         $time);
            end
            if (sgo_cp[0] && (dvld_q != {NC{1'b0}})) begin
                $display("[%0t] L1D PF3_STREAM ERROR scan and direct offer",
                         $time);
            end
        end
    end
`endif
endmodule
`default_nettype wire
`endif
