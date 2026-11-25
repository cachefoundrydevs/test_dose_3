// ==========================================================================
// l1d_pf3_stride_anew.v - next contents of a stride card that the event
//                         before has just allocated, when this event hits it
// ==========================================================================
// IN SIMPLE WORDS : a card written by the event before (E-1) holds only E-1:
//   Initial, stride 0, offset = E-1's offset. If the next event E has the
//   same PC, the card's new contents depend on E and E-1 only, not on the
//   card, so they are the same for every card. They are worked out here
//   once per copy of the event register (E1_COPIES copies, each for 8
//   cards) instead of in every card behind a "view" select. The card then
//   picks these values or its own (from its flops) with one late 2:1
//   select by its alloc_q flop (rule T-9: the select is the last level,
//   not the first).
//
// STAGE / CYCLE   : S2 (all inputs are flops: the event copy and the copy of
//   the event before).
// INSTANTIATED BY : l1d_pf3_stride, generate g_copy, u_anew, E1_COPIES = 8.
//
// PARAMETERS:
//   name    default unit  meaning
//   TAG_W   12      bits  folded PC tag
//   PAGE_W  12      bits  page offset
//   PH_W    10      bits  page hash
//
// PORTS (default widths):
//   ev_tag, ev_st     in  12, 1     the event E
//   ev_lo, ev_lo3     in  12, 14    its offset and 3 x offset
//   ev_ph             in  10        its page hash
//   p_tag, p_st       in  12, 1     the event before E-1
//   p_lo, p_lo3, p_ph in  12,14,10
//   a_key             out 1   E has E-1's PC tag and store bit (a hit on
//                             the card E-1 allocated, if same set)
//   a_correct         out 1   that hit is "correct" (same page, offset
//                             equal to E-1's: stride 0 kept)
//   a_stride          out 13  new stride   (same page ? E - E-1 : 0)
//   a_exp             out 13  new expected (same page ? 2E - E-1 : E)
//   a_s3              out 15  new 3 x stride
//   a_tgt             out 15  new target   (same page ? 4E - 3 E-1 : E)
//   a_nz              out 1   new stride != 0
//
// WORKED EXAMPLE: E-1 offset 0x100, E offset 0x180, same page and PC ->
//   a_key 1, a_correct 0, a_stride 0x80, a_exp 0x200, a_s3 0x180,
//   a_tgt 0x300, a_nz 1 (the card goes Transient with stride 128).
//
// LOGIC LEVELS (T-9): tag compare 4; adders 6 (Kogge-Stone, parallel);
//   same-page select 1 = 7.
// ASSUMPTIONS (V-9.2): none (the keep case of the card rule gives the same
//   values here: a correct hit on stride 0 has E == E-1).
// ==========================================================================
`ifndef L1D_PF3_STRIDE_ANEW_V
`define L1D_PF3_STRIDE_ANEW_V
`default_nettype none

module l1d_pf3_stride_anew #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter TAG_W  = 12, // default 12 bits: folded PC tag
    parameter PAGE_W = 12, // default 12 bits: page offset
    parameter PH_W   = 10  // default 10 bits: page hash
) (
    // ---- the event E (S2 copy) ----
    input  wire [TAG_W-1:0]   ev_tag,    // TAG_W = 12 bits
    input  wire               ev_st,     // store event
    input  wire [PAGE_W-1:0]  ev_lo,     // PAGE_W = 12 bits
    input  wire [PAGE_W+1:0]  ev_lo3,    // 14 bits: 3 x ev_lo
    input  wire [PH_W-1:0]    ev_ph,     // PH_W = 10 bits
    // ---- the event before E-1 (copy) ----
    input  wire [TAG_W-1:0]   p_tag,     // 12 bits
    input  wire               p_st,      // store bit
    input  wire [PAGE_W-1:0]  p_lo,      // 12 bits
    input  wire [PAGE_W+1:0]  p_lo3,     // 14 bits: 3 x p_lo
    input  wire [PH_W-1:0]    p_ph,      // 10 bits
    // ---- the new contents of the card E-1 allocated, if E hits it ----
    output wire               a_key,     // same PC tag and store bit
    output wire               a_correct, // same page and same offset
    output wire [PAGE_W:0]    a_stride,  // 13 bits
    output wire [PAGE_W:0]    a_exp,     // 13 bits
    output wire [PAGE_W+2:0]  a_s3,      // 15 bits
    output wire [PAGE_W+2:0]  a_tgt,     // 15 bits
    output wire               a_nz       // new stride != 0
);

    // ---- derived sizes (value at the default) ----
    localparam STRIDE_W = PAGE_W + 1;              // 13 bits
    localparam TGT_W    = PAGE_W + 3;              // 15 bits

    wire                same_page; // page hash equal
    wire [STRIDE_W-1:0] lo_ext;    // 13 bits: {0, p_lo}
    wire [STRIDE_W-1:0] ev_lo_ext; // 13 bits: {0, ev_lo}
    wire [STRIDE_W-1:0] ev_lo2_ext;// 13 bits: 2 x ev_lo
    wire [TGT_W-1:0]    ev_lo_t;   // 15 bits: ev_lo
    wire [TGT_W-1:0]    ev_lo3_t;  // 15 bits: 3 x ev_lo
    wire [TGT_W-1:0]    ev_lo4_t;  // 15 bits: 4 x ev_lo
    wire [TGT_W-1:0]    lo3_t;     // 15 bits: 3 x p_lo
    wire [STRIDE_W-1:0] delta;     // 13 bits: ev_lo - p_lo
    wire [STRIDE_W-1:0] exp_new;   // 13 bits: 2 ev_lo - p_lo
    wire [TGT_W-1:0]    s3_new;    // 15 bits: 3 ev_lo - 3 p_lo
    wire [TGT_W-1:0]    tgt_new;   // 15 bits: 4 ev_lo - 3 p_lo

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
        .a (ev_lo_ext), .b (lo_ext), .sum (delta)        // ev_lo - p_lo
    );
    l1d_pf3_ks_add #(.W(STRIDE_W), .SUB(1)) u_exp_new (
        .a (ev_lo2_ext), .b (lo_ext), .sum (exp_new)     // 2 ev_lo - p_lo
    );
    l1d_pf3_ks_add #(.W(TGT_W), .SUB(1)) u_s3_new (
        .a (ev_lo3_t), .b (lo3_t), .sum (s3_new)         // 3 ev_lo - 3 p_lo
    );
    l1d_pf3_ks_add #(.W(TGT_W), .SUB(1)) u_tgt_new (
        .a (ev_lo4_t), .b (lo3_t), .sum (tgt_new)        // 4 ev_lo - 3 p_lo
    );

    // other page: restart at E (stride 0, expected = target = E)
    assign a_stride = same_page ? delta   : {STRIDE_W{1'b0}};
    assign a_exp    = same_page ? exp_new : ev_lo_ext;
    assign a_s3     = same_page ? s3_new  : {TGT_W{1'b0}};
    assign a_tgt    = same_page ? tgt_new : ev_lo_t;
    assign a_nz     = same_page & (ev_lo != p_lo);

endmodule

`default_nettype wire
`endif // L1D_PF3_STRIDE_ANEW_V
