`ifndef L1D_PF3_STREAM_CARD_V
`define L1D_PF3_STREAM_CARD_V
`default_nettype none
module l1d_pf3_stream_card #(
    parameter NB         = 4,
    parameter TAG_W      = 12,
    parameter PAGE_NUM_W = 44,
    parameter LIP_W      = 6,
    parameter CONF_W     = 2,
    parameter CONF_ON    = 2,
    parameter CONF_MAX   = 3,
    parameter RECENT_N   = 4,
    parameter DEPTH      = 4
) (
    input  wire                      clk,
    input  wire                      rst,
    input  wire                      flush,
    input  wire [NB-1:0]             lane_pick,
    input  wire [NB*TAG_W-1:0]       lane_tag,
    input  wire [NB*LIP_W-1:0]       lane_line,
    input  wire [NB*(LIP_W+1)-1:0]   lane_line_p1,
    input  wire                      ev_vld,
    input  wire [PAGE_NUM_W-1:0]     ev_page,
    input  wire [LIP_W-1:0]          ev_line,
    input  wire [LIP_W-1:0]          ev_line_p1,
    input  wire [LIP_W-1:0]          ev_line_m1,
    input  wire                      ev_top,
    input  wire                      ev_bot,
    input  wire [LIP_W-1:0]          ev_tgt_up,
    input  wire [LIP_W-1:0]          ev_tgt_dn,
    input  wire                      ev_trig,
    input  wire                      prev_card,
    input  wire                      prev_same_tag,
    input  wire                      prev_same,
    input  wire                      prev_up,
    input  wire                      prev_dn,
    input  wire                      alloc_en,
    input  wire [TAG_W-1:0]          p_tag,
    input  wire [PAGE_NUM_W-1:0]     p_page,
    input  wire [LIP_W-1:0]          p_line,
    input  wire [LIP_W:0]            p_line_p1w,
    input  wire [LIP_W-1:0]          p_tgt_dn,
    input  wire                      p_store,
    input  wire                      scan_idle,
    input  wire                      scan_sel,
    input  wire                      slot_go,
    input  wire [RECENT_N-1:0]       recent_tag_eq,
    input  wire                      ev_dup_p1,
    input  wire                      ev_dup_m1,
    input  wire [RECENT_N*LIP_W-1:0] recent_lp1,
    input  wire [RECENT_N*LIP_W-1:0] recent_lm1,
    input  wire [RECENT_N*TAG_W-1:0] recent_tag,
    input  wire [RECENT_N-1:0]       recent_vld,
    output wire                      hit,
    output wire                      pend_out,
    output wire [CONF_MAX+1:0]       class_out,
    output wire                      dir_vld,
    output wire                      dir_evp,
    output wire                      dir_up,
    output wire [LIP_W-1:0]          dir_kp,
    output wire [LIP_W-1:0]          dir_kp_p1,
    output wire [LIP_W-1:0]          dir_kp_m1,
    output wire                      dir_dup,
    output wire                      dir_store,
    output wire                      scan_go,
    output wire [LIP_W-1:0]          scan_line,
    output wire [LIP_W-1:0]          scan_lp1,
    output wire [LIP_W-1:0]          scan_lm1,
    output wire                      scan_dup,
    output wire [PAGE_NUM_W-1:0]     page_out,
    output wire [TAG_W-1:0]          tag_out,
    output wire                      store_out
);
    localparam [CONF_W-1:0] CONF_ON_V  = CONF_ON;
    localparam [CONF_W-1:0] CONF_MAX_V = CONF_MAX;
    localparam [CONF_W-1:0] CONF_ONE   = 1;
    localparam [LIP_W-1:0]  LIP_ONE    = 1;
    localparam [LIP_W-1:0]  LIP_TWO    = 2;
    localparam AQ_N = 3;
    localparam NCLS = CONF_MAX + 2;
    localparam [NCLS-1:0] CLASS_FREE = 1;
    localparam [NCLS-1:0] CLASS_NEW  = 2;
    localparam WIDE_W = LIP_W + 2;
    localparam [WIDE_W-1:0] STEP_LEAD = DEPTH + 1;
    localparam [WIDE_W-1:0] LAST_WIDE = (1 << LIP_W) - 1;
    localparam [LIP_W-1:0]  LAST_LINE = (1 << LIP_W) - 1;
    reg                   valid_q;
    reg  [TAG_W-1:0]      tag_q;
    reg  [PAGE_NUM_W-1:0] page_q;
    reg  [LIP_W-1:0]      last_q;
    reg  [LIP_W:0]        last_p1_q;
    reg                   dir_q;
    reg  [CONF_W-1:0]     conf_q;
    reg  [NCLS-1:0]       class_q;
    reg  [NCLS-1:0]       class_nx;
    reg  [CONF_W-1:0]     conf_nx;
    reg  [CONF_W-1:0]     class_conf;
    reg  [LIP_W-1:0]      ahead_q;
    reg  [LIP_W-1:0]      tgt_q;
    reg  [LIP_W-1:0]      tup_q;
    reg  [LIP_W-1:0]      tdn_q;
    reg  [LIP_W-1:0]      ev_tup;
    reg  [LIP_W-1:0]      ev_tdn;
    reg  [LIP_W-1:0]      p_tup;
    reg  [LIP_W-1:0]      p_tdn;
    wire [LIP_W-1:0]      v_tup;
    wire [LIP_W-1:0]      v_tdn;
    reg                   store_q;
    reg                   pend_q;
    wire [AQ_N-1:0]       alloc_cp;
    wire                  alloc_d;
    wire                  v_valid;
    wire [TAG_W-1:0]      v_tag;
    wire [PAGE_NUM_W-1:0] v_page;
    wire [LIP_W-1:0]      v_last;
    wire [LIP_W:0]        v_last_p1;
    wire                  v_dir;
    wire [CONF_W-1:0]     v_conf;
    wire [LIP_W-1:0]      v_ahead;
    wire [LIP_W-1:0]      v_tgt;
    wire                  v_store;
    wire                  v_pend;
    reg                   s1_hit_q;
    reg                   s1_same_q;
    reg                   s1_up_q;
    reg                   s1_dn_q;
    reg  [NB-1:0]         lane_hit;
    reg  [NB-1:0]         lane_same;
    reg  [NB-1:0]         lane_up;
    reg  [NB-1:0]         lane_dn;
    wire                  c_same;
    wire                  c_up;
    wire                  c_dn;
    wire                  behind_up;
    wire                  behind_dn;
    wire                  conf_unset;
    wire                  step_up;
    wire                  step_dn;
    wire                  step_ok;
    wire                  flip_ok;
    wire                  ahead_behind;
    reg  [CONF_W-1:0]     new_conf;
    reg                   new_dir;
    reg  [LIP_W-1:0]      new_tgt;
    wire                  new_pend;
    wire [LIP_W-1:0]      ahead_p1;
    wire [LIP_W-1:0]      ahead_m1;
    wire [LIP_W-1:0]      ahead_p2;
    wire [LIP_W-1:0]      ahead_m2;
    wire [LIP_W-1:0]      ev_next;
    wire                  ev_done;
    wire                  ev_dup;
    wire                  kept_dir;
    wire [LIP_W-1:0]      kp_next;
    wire                  kp_done;
    wire                  kp_dup;
    wire [LIP_W-1:0]      kp_lp1;
    wire [LIP_W-1:0]      kp_lm1;
    wire                  pend_go;
    wire                  mk_ev;
    wire                  mk_kp;
    wire                  d_make;
    wire [LIP_W:0]        go_pick;
    wire                  ev_hit;
    wire [LIP_W-1:0]      go_ev;
    wire                  pend_ev;
    wire [LIP_W-1:0]      go_oth;
    wire                  pend_oth;
    wire [LIP_W-1:0]      oth_go;
    wire                  poth_go;
    wire                  poth_stop;
    wire                  cur_done;
    wire                  scan_done;
    wire [LIP_W-1:0]      cur_next;
    wire                  ev_path;
    wire                  kept_done_up;
    wire                  kept_done_dn;
    reg                   ah_dup_p1;
    reg                   ah_dup_m1;
    reg                   own_dup_p1;
    reg                   own_dup_m1;
    reg                   s_dup_r;
    assign alloc_d = alloc_en & ~flush;
    genvar copy_g;
    generate
        for (copy_g = 0; copy_g < AQ_N; copy_g = copy_g + 1) begin : g_aq
            l1d_pf3_rep #(.W (1), .HAS_RST (1), .HAS_EN (0)) u_alloc_q (
                .clk (clk), .rst (rst), .en (1'b1),
                .d   (alloc_d), .q (alloc_cp[copy_g])
            );
        end
    endgenerate
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
    always @* begin : step_tgt_proc
        reg [WIDE_W-1:0] ev_wide;
        reg [WIDE_W-1:0] p_wide;
        reg [WIDE_W-1:0] ev_sum;
        reg [WIDE_W-1:0] p_sum;
        reg [LIP_W-1:0]  ev_dif;
        reg [LIP_W-1:0]  p_dif;
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
    end
    always @* begin : s1_cmp_proc
        integer lane_idx;
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_hit[lane_idx]  = v_valid
                & (v_tag == lane_tag[lane_idx*TAG_W +: TAG_W]);
            lane_same[lane_idx] = (v_last == lane_line[lane_idx*LIP_W +: LIP_W]);
            lane_up[lane_idx]   = ({1'b0, lane_line[lane_idx*LIP_W +: LIP_W]}
                                   == v_last_p1);
            lane_dn[lane_idx]   = (lane_line_p1[lane_idx*(LIP_W+1) +: LIP_W+1]
                                   == {1'b0, v_last});
        end
    end
    always @(posedge clk) begin : s1_reg_proc
        s1_hit_q  <= |(lane_pick & lane_hit);
        s1_same_q <= |(lane_pick & lane_same);
        s1_up_q   <= |(lane_pick & lane_up);
        s1_dn_q   <= |(lane_pick & lane_dn);
    end
    assign hit    = ev_vld & (prev_card ? prev_same_tag : s1_hit_q);
    assign c_same = prev_card ? prev_same : s1_same_q;
    assign c_up   = prev_card ? prev_up   : s1_up_q;
    assign c_dn   = prev_card ? prev_dn   : s1_dn_q;
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
            new_conf  = v_conf;
            new_dir   = v_dir;
            new_tgt   = v_tgt;
        end
        else if (step_ok) begin
            new_conf  = (v_conf == CONF_MAX_V) ? v_conf : (v_conf + CONF_ONE);
            new_dir   = c_up;
            new_tgt   = c_up ? ev_tgt_up : ev_tgt_dn;
        end
        else if (flip_ok) begin
            new_conf  = {CONF_W{1'b0}};
            new_dir   = c_up;
            new_tgt   = c_up ? ev_tgt_up : ev_tgt_dn;
        end
        else begin
            new_conf  = {CONF_W{1'b0}};
            new_dir   = 1'b0;
            new_tgt   = ev_tgt_dn;
        end
    end
    assign new_pend = (new_conf >= CONF_ON_V) & (ev_trig | v_pend);
    assign ahead_p1 = ahead_q + LIP_ONE;
    assign ahead_m1 = ahead_q - LIP_ONE;
    assign ahead_p2 = ahead_q + LIP_TWO;
    assign ahead_m2 = ahead_q - LIP_TWO;
    always @* begin : dup_proc
        integer key_idx;
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
        end
    end
    assign ev_path = ~c_same & (~step_ok | (v_conf < CONF_ON_V) | ahead_behind);
    assign kept_done_up = (ahead_q >= tup_q);
    assign kept_done_dn = (ahead_q <= tdn_q);
    assign ev_next = c_up ? ev_line_p1 : ev_line_m1;
    assign ev_done = c_up ? ev_top : ev_bot;
    assign ev_dup  = c_up ? ev_dup_p1 : ev_dup_m1;
    assign kept_dir = c_same ? dir_q : c_up;
    assign kp_next  = kept_dir ? ahead_p1 : ahead_m1;
    assign kp_done  = c_same ? cur_done : (c_up ? kept_done_up : kept_done_dn);
    assign kp_dup   = kept_dir ? ah_dup_p1 : ah_dup_m1;
    assign kp_lp1   = kept_dir ? ahead_p2 : ahead_q;
    assign kp_lm1   = kept_dir ? ahead_q : ahead_m2;
    assign pend_go  = hit & new_pend & scan_idle;
    assign mk_ev    = pend_go & ~ev_done;
    assign mk_kp    = pend_go & ~kp_done;
    assign d_make   = ev_path ? mk_ev : mk_kp;
    assign dir_vld   = slot_go & d_make;
    assign dir_evp   = ev_path;
    assign dir_up    = c_up;
    assign dir_kp    = kp_next;
    assign dir_kp_p1 = kp_lp1;
    assign dir_kp_m1 = kp_lm1;
    assign dir_dup   = ev_path ? ev_dup  : kp_dup;
    assign dir_store = v_store;
    assign cur_done = dir_q ? (ahead_q >= tgt_q) : (ahead_q <= tgt_q);
    assign cur_next = dir_q ? ahead_p1 : ahead_m1;
    always @* s_dup_r = dir_q ? own_dup_p1 : own_dup_m1;
    assign scan_go   = scan_sel & v_pend & ~cur_done;
    assign scan_done = scan_sel & v_pend & cur_done;
    assign scan_line = cur_next;
    assign scan_lp1  = dir_q ? ahead_p2 : ahead_q;
    assign scan_lm1  = dir_q ? ahead_q : ahead_m2;
    assign scan_dup  = s_dup_r;
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
    always @* begin : class_next_proc
        integer class_idx;
        conf_nx    = hit ? new_conf : v_conf;
        class_conf = {CONF_W{1'b0}};
        class_nx   = {NCLS{1'b0}};
        class_nx[0] = ~v_valid;
        for (class_idx = 1; class_idx < NCLS; class_idx = class_idx + 1) begin
            class_nx[class_idx] = v_valid & (conf_nx == class_conf);
            class_conf = class_conf + CONF_ONE;
        end
    end
    always @(posedge clk) begin : class_proc
        if (rst) begin
            class_q <= CLASS_FREE;
        end
        else if (alloc_d) begin
            class_q <= CLASS_NEW;
        end
        else begin
            class_q <= class_nx;
        end
    end
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
            end
        end
    end
    always @(posedge clk) begin : card_data_proc
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
        end
        ahead_q <= go_pick[LIP_W-1:0];
    end
    assign pend_out  = v_pend;
    assign class_out = class_q;
    assign page_out  = v_page;
    assign tag_out   = v_tag;
    assign store_out = v_store;
`ifdef L1D_DEBUG
    reg [NCLS-1:0]   class_view;
    reg [CONF_W-1:0] view_conf;
    always @* begin : class_view_proc
        integer class_idx;
        view_conf     = {CONF_W{1'b0}};
        class_view    = {NCLS{1'b0}};
        class_view[0] = ~v_valid;
        for (class_idx = 1; class_idx < NCLS; class_idx = class_idx + 1) begin
            class_view[class_idx] = v_valid & (v_conf == view_conf);
            view_conf = view_conf + CONF_ONE;
        end
    end
    wire chk_step = hit & ~c_same & step_ok & (v_conf >= CONF_ON_V)
                    & ~alloc_cp[0] & ~alloc_cp[1];
    wire chk_behind = c_up ? (ahead_q < ev_line) : (ahead_q > ev_line);
    wire chk_kdone  = c_up ? (ahead_q >= ev_tgt_up) : (ahead_q <= ev_tgt_dn);
    wire [LIP_W-1:0] ahead_go;
    wire [LIP_W-1:0] ahead_stop;
    wire             pend_go_nx;
    wire             pend_stop_nx;
    wire             d_finish;
    assign d_finish = hit & new_pend & (ev_path ? ev_done : kp_done);
    assign ahead_go   = hit ? (ev_path ? (mk_ev ? ev_next : ev_line)
                                       : (mk_kp ? kp_next : v_ahead))
                            : (scan_go ? cur_next : v_ahead);
    assign ahead_stop = (hit && ev_path) ? ev_line : v_ahead;
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
    end
`endif
endmodule
`default_nettype wire
`endif
