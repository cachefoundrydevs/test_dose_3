`ifndef L1D_PF3_STRIDE_ENTRY_V
`define L1D_PF3_STRIDE_ENTRY_V
`default_nettype none
module l1d_pf3_stride_entry #(
    parameter TAG_W      = 12,
    parameter PAGE_W     = 12,
    parameter OFFSET_W   = 6,
    parameter PH_W       = 10,
    parameter FILT_DEPTH = 4,
    parameter NL         = 4
) (
    input  wire                                 clk,
    input  wire                                 rst,
    input  wire                                 clear,
    input  wire                                 ev_set_on,
    input  wire [NL*TAG_W-1:0]                  s1_tag,
    input  wire [NL-1:0]                        s1_st,
    input  wire [NL-1:0]                        ev_lane,
    input  wire [NL-1:0]                        ev_p2_key,
    input  wire [NL*PH_W-1:0]                   s1_ph,
    input  wire [NL-1:0]                        ev_ph_ev,
    input  wire [NL-1:0]                        ev_ph_p,
    input  wire [TAG_W-1:0]                     ev_tag,
    input  wire                                 ev_st,
    input  wire [PAGE_W-1:0]                    ev_lo,
    input  wire [PAGE_W+1:0]                    ev_lo3,
    input  wire [PH_W-1:0]                      ev_ph,
    input  wire [FILT_DEPTH-1:0]                filt_pm,
    input  wire [FILT_DEPTH*(PAGE_W-OFFSET_W)-1:0] filt_lip,
    input  wire                                 prev_pm,
    input  wire [PAGE_W-OFFSET_W-1:0]           prev_lip,
    input  wire [TAG_W-1:0]                     p_tag,
    input  wire                                 p_st,
    input  wire [PAGE_W-1:0]                    p_lo,
    input  wire [PAGE_W+1:0]                    p_lo3,
    input  wire [PH_W-1:0]                      p_ph,
    input  wire                                 a_key,
    input  wire                                 a_correct,
    input  wire [PAGE_W:0]                      a_stride,
    input  wire [PAGE_W:0]                      a_exp,
    input  wire [PAGE_W+2:0]                    a_s3,
    input  wire [PAGE_W+2:0]                    a_tgt,
    input  wire                                 a_nz,
    input  wire                                 alloc_en,
    output wire                                 hit,
    output reg  [2:0]                           class_rank,
    output wire                                 fire_new,
    output wire                                 fire_dup,
    output wire [PAGE_W-OFFSET_W-1:0]           tgt_lip
);
    localparam STRIDE_W = PAGE_W + 1;
    localparam TGT_W    = PAGE_W + 3;
    localparam LO3_W    = PAGE_W + 2;
    localparam LIP_W    = PAGE_W - OFFSET_W;
    localparam OH_N     = 6;
    localparam GRP_N    = OH_N - 2;
    localparam AQ_N     = 5;
    localparam [1:0] S_INIT   = 2'd0;
    localparam [1:0] S_TRANS  = 2'd1;
    localparam [1:0] S_STEADY = 2'd2;
    localparam [1:0] S_NOPRED = 2'd3;
    reg                  valid_q;
    reg  [NL-1:0]        lane_eq_q;
    reg                  wrote_q;
    reg  [NL-1:0]        lane_eq;
    wire [NL-1:0]        key_rows;
    reg  [NL-1:0]        lane_pheq_q;
    reg  [NL-1:0]        lane_pheq;
    reg                  ph_ev_q;
    reg                  ph_p_q;
    wire [NL-1:0]        ph_rows;
    wire [OH_N-1:0]      key_cp;
    wire [OH_N-1:0]      own_cp;
    reg                  st_q;
    reg  [TAG_W-1:0]     tag_q;
    reg  [1:0]           state_q;
    reg  [STRIDE_W-1:0]  stride_q;
    reg                  nz_q;
    reg  [STRIDE_W-1:0]  exp_q;
    reg  [TGT_W-1:0]     s3_q;
    reg  [TGT_W-1:0]     tgt_q;
    reg  [PAGE_W-1:0]    lo_q;
    reg  [LO3_W-1:0]     lo3_q;
    reg  [PH_W-1:0]      ph_q;
    wire [AQ_N-1:0]      alloc_cp;
    wire                 alloc_clr;
    wire                 v_valid;
    wire [1:0]           v_state;
    wire [LIP_W-1:0]     v_tgt_lip;
    wire                 a_hit;
    reg  [1:0]           a_state;
    reg  [STRIDE_W-1:0]  al_stride;
    reg  [STRIDE_W-1:0]  al_exp;
    reg  [TGT_W-1:0]     al_s3;
    reg  [TGT_W-1:0]     al_tgt;
    reg                  al_nz;
    wire                 own_hit;
    wire                 same_page;
    wire                 exp_ok;
    wire                 correct;
    wire                 steady;
    wire                 armed;
    wire                 in_page;
    wire                 own_line;
    reg                  filt_main;
    wire                 filt_last;
    wire                 dup_prev;
    wire                 fire;
    wire [STRIDE_W-1:0]  lo_ext;
    wire [STRIDE_W-1:0]  ev_lo_ext;
    wire [STRIDE_W-1:0]  ev_lo2_ext;
    wire [TGT_W-1:0]     ev_lo_t;
    wire [TGT_W-1:0]     ev_lo4_t;
    wire [TGT_W-1:0]     ev_lo3_t;
    wire [TGT_W-1:0]     lo3_t;
    wire [STRIDE_W-1:0]  exp_keep;
    wire [TGT_W-1:0]     tgt_keep;
    wire [STRIDE_W-1:0]  delta;
    wire [STRIDE_W-1:0]  exp_new;
    wire [TGT_W-1:0]     s3_new;
    wire [TGT_W-1:0]     tgt_new;
    wire                 nz_new;
    reg  [1:0]           state_next;
    wire [GRP_N-1:0]     sel_keep;
    wire [GRP_N-1:0]     sel_new;
    wire [GRP_N-1:0]     sel_zero;
    wire [GRP_N-1:0]     sel_hold;
    wire                 sel_alloc;
    assign alloc_clr = rst | clear;
    genvar copy_g;
    generate
        for (copy_g = 0; copy_g < AQ_N; copy_g = copy_g + 1) begin : g_aq
            l1d_pf3_rep #(.W (1), .HAS_RST (1), .HAS_EN (0)) u_alloc_q (
                .clk (clk), .rst (alloc_clr), .en (1'b1),
                .d   (alloc_en), .q (alloc_cp[copy_g])
            );
        end
    endgenerate
    assign v_valid  = valid_q | alloc_cp[0];
    assign v_tgt_lip = alloc_cp[2] ? p_lo[PAGE_W-1:OFFSET_W]
                                   : tgt_q[PAGE_W-1:OFFSET_W];
    assign v_state  = alloc_cp[4] ? S_INIT : state_q;
    assign a_hit     = ev_set_on & a_key;
    assign key_rows  = ev_lane & (wrote_q ? ev_p2_key : lane_eq_q);
    genvar oh_g;
    generate
        for (oh_g = 0; oh_g < OH_N; oh_g = oh_g + 1) begin : g_own
            l1d_pf3_or_rows #(.W (1), .N (NL)) u_key_or (
                .rows   (key_rows),
                .or_out (key_cp[oh_g])
            );
            assign own_cp[oh_g] = ev_set_on & valid_q & key_cp[oh_g];
        end
    endgenerate
    assign own_hit   = own_cp[OH_N-1];
    assign hit       = alloc_cp[0] ? a_hit : own_hit;
    assign ph_rows   = ev_lane & (ph_ev_q ? ev_ph_ev
                                          : (ph_p_q ? ev_ph_p : lane_pheq_q));
    assign same_page = |ph_rows;
    assign lo_ext    = {1'b0, lo_q};
    assign ev_lo_ext = {1'b0, ev_lo};
    assign exp_ok    = (exp_q == ev_lo_ext);
    assign correct   = same_page & exp_ok;
    assign steady    = (state_q == S_STEADY);
    assign armed     = (state_q != S_NOPRED) & nz_q;
    assign tgt_lip   = v_tgt_lip;
    assign in_page   = (tgt_q[TGT_W-1:PAGE_W] == {(TGT_W-PAGE_W){1'b0}});
    assign own_line  = (tgt_q[PAGE_W-1:OFFSET_W] == ev_lo[PAGE_W-1:OFFSET_W]);
    always @* begin : filter_proc
        integer slot_idx;
        filt_main = 1'b0;
        for (slot_idx = 0; slot_idx < FILT_DEPTH - 1; slot_idx = slot_idx + 1) begin
            if (filt_pm[slot_idx]
                && (filt_lip[slot_idx*LIP_W +: LIP_W] == tgt_q[PAGE_W-1:OFFSET_W])) begin
                filt_main = 1'b1;
            end
        end
    end
    assign filt_last = filt_pm[FILT_DEPTH-1]
                       & (filt_lip[(FILT_DEPTH-1)*LIP_W +: LIP_W] == tgt_q[PAGE_W-1:OFFSET_W]);
    assign dup_prev  = prev_pm & (prev_lip == tgt_q[PAGE_W-1:OFFSET_W]);
    assign fire     = ~alloc_cp[0] & own_hit & correct & armed & in_page
                      & ~own_line & ~filt_main;
    assign fire_new = fire & ~filt_last;
    assign fire_dup = fire & ~dup_prev;
    assign ev_lo2_ext = {ev_lo, 1'b0};
    assign ev_lo_t    = {{(TGT_W-PAGE_W){1'b0}}, ev_lo};
    assign ev_lo4_t   = {1'b0, ev_lo, 2'b00};
    assign ev_lo3_t   = {1'b0, ev_lo3};
    assign lo3_t      = {1'b0, lo3_q};
    assign nz_new     = (ev_lo != lo_q);
    l1d_pf3_ks_add #(.W(STRIDE_W), .SUB(0)) u_exp_keep (
        .a (ev_lo_ext), .b (stride_q), .sum (exp_keep)
    );
    l1d_pf3_ks_add #(.W(TGT_W), .SUB(0)) u_tgt_keep (
        .a (ev_lo_t), .b (s3_q), .sum (tgt_keep)
    );
    l1d_pf3_ks_add #(.W(STRIDE_W), .SUB(1)) u_delta (
        .a (ev_lo_ext), .b (lo_ext), .sum (delta)
    );
    l1d_pf3_ks_add #(.W(STRIDE_W), .SUB(1)) u_exp_new (
        .a (ev_lo2_ext), .b (lo_ext), .sum (exp_new)
    );
    l1d_pf3_ks_add #(.W(TGT_W), .SUB(1)) u_s3_new (
        .a (ev_lo3_t), .b (lo3_t), .sum (s3_new)
    );
    l1d_pf3_ks_add #(.W(TGT_W), .SUB(1)) u_tgt_new (
        .a (ev_lo4_t), .b (lo3_t), .sum (tgt_new)
    );
    always @* begin : next_proc
        case (state_q)
            S_INIT:   state_next = correct ? S_STEADY : S_TRANS;
            S_TRANS:  state_next = correct ? S_STEADY : S_NOPRED;
            S_STEADY: state_next = correct ? S_STEADY : S_INIT;
            default:  state_next = correct ? S_TRANS  : S_NOPRED;
        endcase
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
        end
    end
    assign sel_alloc = alloc_cp[3];
    assign sel_hold  = {GRP_N{~alloc_cp[3]}} & ~own_cp[GRP_N-1:0];
    assign sel_keep  = {GRP_N{~alloc_cp[3] & steady}} & own_cp[GRP_N-1:0];
    assign sel_new   = {GRP_N{~alloc_cp[3] & ~steady & same_page}}
                       & own_cp[GRP_N-1:0];
    assign sel_zero  = {GRP_N{~alloc_cp[3] & ~steady & ~same_page}}
                       & own_cp[GRP_N-1:0];
    always @(posedge clk) begin : card_ctl_proc
        if (rst || clear) begin
            valid_q <= 1'b0;
            state_q <= S_INIT;
        end
        else begin
            valid_q <= v_valid;
            if (alloc_cp[4]) begin
                state_q <= a_hit ? a_state : S_INIT;
            end
            else if (own_cp[4]) begin
                state_q <= state_next;
            end
        end
    end
    always @(posedge clk) begin : card_data_proc
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
        lane_eq_q <= lane_eq;
    end
    always @* begin : s1_ph_proc
        integer lane_idx;
        for (lane_idx = 0; lane_idx < NL; lane_idx = lane_idx + 1) begin
            lane_pheq[lane_idx] = (s1_ph[lane_idx*PH_W +: PH_W] == ph_q);
        end
    end
    always @(posedge clk) begin : ph_wrote_proc
        if (rst) begin
            ph_ev_q <= 1'b0;
            ph_p_q  <= 1'b0;
        end
        else begin
            ph_ev_q <= alloc_cp[1] ? a_hit : own_cp[4];
            ph_p_q  <= alloc_cp[1] & ~a_hit;
        end
        lane_pheq_q <= lane_pheq;
    end
    always @* begin : s1_key_proc
        integer lane_idx;
        for (lane_idx = 0; lane_idx < NL; lane_idx = lane_idx + 1) begin
            lane_eq[lane_idx] = (s1_tag[lane_idx*TAG_W +: TAG_W] == tag_q)
                                & (s1_st[lane_idx] == st_q);
        end
    end
    always @(posedge clk) begin : wrote_proc
        if (rst) begin
            wrote_q <= 1'b0;
        end
        else begin
            wrote_q <= alloc_cp[4];
        end
    end
    always @* begin : class_proc
        if (!v_valid) begin
            class_rank = 3'd0;
        end
        else begin
            case (v_state)
                S_NOPRED: class_rank = 3'd1;
                S_INIT:   class_rank = 3'd2;
                S_TRANS:  class_rank = 3'd3;
                default:  class_rank = 3'd4;
            endcase
        end
    end
`ifdef L1D_DEBUG
    wire [TGT_W-1:0] dbg_stride_t = {{(TGT_W-STRIDE_W){stride_q[STRIDE_W-1]}},
                                     stride_q};
    always @(posedge clk) begin : debug_proc
        if (!rst && valid_q && !alloc_cp[0]) begin
            if ((exp_q != ({1'b0, lo_q} + stride_q))
                || (s3_q != (dbg_stride_t + {dbg_stride_t[TGT_W-2:0], 1'b0}))
                || (tgt_q != ({{(TGT_W-PAGE_W){1'b0}}, lo_q} + s3_q))
                || (nz_q != (stride_q != {STRIDE_W{1'b0}}))) begin
                $display("[%0t] L1D PF3_STRIDE_ENTRY ERROR card invariant", $time);
            end
        end
        if (!rst && (own_hit != (ev_set_on && valid_q && (tag_q == ev_tag)
                                 && (st_q == ev_st)))) begin
            $display("[%0t] L1D PF3_STRIDE_ENTRY ERROR own hit != key compare",
                     $time);
        end
        if (!rst && ev_set_on && (same_page != (ph_q == ev_ph))) begin
            $display("[%0t] L1D PF3_STRIDE_ENTRY ERROR same page != ph compare",
                     $time);
        end
    end
`endif
endmodule
`default_nettype wire
`endif
