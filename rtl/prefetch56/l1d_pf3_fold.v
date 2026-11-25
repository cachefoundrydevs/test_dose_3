// ==========================================================================
// l1d_pf3_fold.v - XOR fold of IN_W bits into OUT_W bits (one copy of a
//                  replicated hash, fanout split)
// ==========================================================================
// IN SIMPLE WORDS : the input bits are dealt round-robin into OUT_W piles
//   (bit i goes to pile i % OUT_W) and each pile is XORed into one output
//   bit. A block that needs the folded value in many places keeps several
//   copies of this fold, each feeding a few places, so no copy's output
//   has to drive every load through a long buffer tree (rule T-6). Kept as
//   its own level of hierarchy so synthesis does not merge the copies
//   (same inputs -> same logic would otherwise be shared).
//
// STAGE / CYCLE   : combinational, inside a stage of the parent.
// INSTANTIATED BY : l1d_pf3_stride (u_tag_fold: the S1 PC tag of every
//   lane, one copy per group of cards, compared with each card's key).
//
// PARAMETERS:
//   name    default unit  meaning
//   IN_W    59      bits  input width
//   OUT_W   12      bits  output width (folded)
//
// PORTS:
//   din   in  IN_W   the bits to fold
//   dout  out OUT_W  dout[k] = XOR of din[i] for all i with i % OUT_W = k
//
// WORKED EXAMPLE: IN_W = 6, OUT_W = 3, din = 6'b101_011 ->
//   dout[0] = din[0]^din[3] = 1^1 = 0, dout[1] = din[1]^din[4] = 1^0 = 1,
//   dout[2] = din[2]^din[5] = 0^1 = 1 -> dout = 3'b110.
//
// LOGIC LEVELS (T-9): ceil(log2(ceil(IN_W / OUT_W))) XOR2 levels (3 for
//   the default: 5 bits per output).
// ASSUMPTIONS (V-9.2): none.
// ==========================================================================
`ifndef L1D_PF3_FOLD_V
`define L1D_PF3_FOLD_V
`default_nettype none

(* keep_hierarchy *)
module l1d_pf3_fold #(
    parameter IN_W  = 59, // default 59 bits: input width
    parameter OUT_W = 12  // default 12 bits: output width
) (
    input  wire [IN_W-1:0]  din,  // IN_W = 59 bits: bits to fold
    output reg  [OUT_W-1:0] dout  // OUT_W = 12 bits: folded value
);

    always @* begin : fold_proc
        integer bit_idx; // walks the input bits
        dout = {OUT_W{1'b0}};
        for (bit_idx = 0; bit_idx < IN_W; bit_idx = bit_idx + 1) begin
            dout[bit_idx % OUT_W] = dout[bit_idx % OUT_W] ^ din[bit_idx];
        end // for bit_idx
    end // always fold_proc

endmodule

`default_nettype wire
`endif // L1D_PF3_FOLD_V
