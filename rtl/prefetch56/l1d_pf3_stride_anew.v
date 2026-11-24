`ifndef L1D_PF3_STRIDE_ANEW_V
`define L1D_PF3_STRIDE_ANEW_V
`default_nettype none
module l1d_pf3_stride_anew #(
    parameter TAG_W  = 12,
    parameter PAGE_W = 12,
    parameter PH_W   = 10
) (
    input  wire [TAG_W-1:0]   ev_tag,
    input  wire               ev_st,
    input  wire [PAGE_W-1:0]  ev_lo,
    input  wire [PAGE_W+1:0]  ev_lo3,
    input  wire [PH_W-1:0]    ev_ph,
    input  wire [TAG_W-1:0]   p_tag,
    input  wire               p_st,
    input  wire [PAGE_W-1:0]  p_lo,
    input  wire [PAGE_W+1:0]  p_lo3,
    input  wire [PH_W-1:0]    p_ph,
    output wire               a_key,
    output wire               a_correct,
    output wire [PAGE_W:0]    a_stride,
    output wire [PAGE_W:0]    a_exp,
    output wire [PAGE_W+2:0]  a_s3,
    output wire [PAGE_W+2:0]  a_tgt,
    output wire               a_nz
);
    localparam STRIDE_W = PAGE_W + 1;
    localparam TGT_W    = PAGE_W + 3;
    wire                same_page;
    wire [STRIDE_W-1:0] lo_ext;
    wire [STRIDE_W-1:0] ev_lo_ext;
    wire [STRIDE_W-1:0] ev_lo2_ext;
    wire [TGT_W-1:0]    ev_lo_t;
    wire [TGT_W-1:0]    ev_lo3_t;
    wire [TGT_W-1:0]    ev_lo4_t;
    wire [TGT_W-1:0]    lo3_t;
    wire [STRIDE_W-1:0] delta;
    wire [STRIDE_W-1:0] exp_new;
    wire [TGT_W-1:0]    s3_new;
    wire [TGT_W-1:0]    tgt_new;
    assign a_key      = (p_tag == ev_tag) & (p_st == ev_st);
    assign same_page  = (p_ph == ev_ph);
    assign a_correct  = same_page & (p_lo == ev_lo);
    assign lo_ext     = {1'b0, p_lo};
    assign ev_lo_ext  = {1'b0, ev_lo};
    assign ev_lo2_ext = {ev_lo, 1'b0};
    assign ev_lo_t    = {{(TGT_W-PAGE_W){1'b0}}, ev_lo};
    assign ev_lo3_t   = {1'b0, ev_lo3};
    assign ev_lo4_t   = {1'b0, ev_lo, 2'b00};
    assign lo3_t      = {1'b0, p_lo3};
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
    assign a_stride = same_page ? delta   : {STRIDE_W{1'b0}};
    assign a_exp    = same_page ? exp_new : ev_lo_ext;
    assign a_s3     = same_page ? s3_new  : {TGT_W{1'b0}};
    assign a_tgt    = same_page ? tgt_new : ev_lo_t;
    assign a_nz     = same_page & (ev_lo != p_lo);
endmodule
`default_nettype wire
`endif
