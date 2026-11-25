// ==========================================================================
// l1d_pf3_mux4oh.v - a W-bit 4:1 select with one-hot selects, kept as its
//                    own level of hierarchy (rule T-9, two late bits)
// ==========================================================================
// IN SIMPLE WORDS : out = the input whose select bit is 1 (an AND-OR).
//   When two late decisions (a and b) pick among four results worked out
//   ahead of time, two 2:1 selects in series put two select levels on the
//   path of every result. Here the parent decodes the two late bits into
//   four one-hot selects ({a & b, ~a & b, a & ~b, ~a & ~b}) and this
//   module picks with one AND-OR level, so a result and a late bit each
//   pass one select only. Kept as its own hierarchy so synthesis does not
//   fold the late bits back into the logic before it.
//
// STAGE / CYCLE   : combinational, the last level of the parent's stage.
// INSTANTIATED BY : l1d_pf3_rank (stream / SMS decisions and the output
//   line per bank, by s_valid and t_valid); one instance per select.
//
// PARAMETERS:
//   name   default unit  meaning
//   W      32      bits  width
//   SEL_N  4       -     inputs (fixed at 4: in3 .. in0)
//
// PORTS:
//   sel_oh  in  4  one-hot selects: bit k picks in_k (all 0: out = 0)
//   in3     in  W  the result for sel_oh[3]
//   in2     in  W  the result for sel_oh[2]
//   in1     in  W  the result for sel_oh[1]
//   in0     in  W  the result for sel_oh[0]
//   out     out W  the chosen result
//
// WORKED EXAMPLE: W = 4, in3 = 4'b1000, in2 = 4'b0100, in1 = 4'b0010,
//   in0 = 4'b0001, sel_oh = 4'b0100 -> out = 4'b0100.
//
// LOGIC LEVELS (T-9): 2 (AO22 x 2 + OR2 per bit, or AO222 + AO21).
// ASSUMPTIONS (V-9.2): A1 sel_oh has at most one bit set (the parent
//   decodes it from two bits, so it is one-hot by construction).
// ==========================================================================
`ifndef L1D_PF3_MUX4OH_V
`define L1D_PF3_MUX4OH_V
`default_nettype none

(* keep_hierarchy *)
module l1d_pf3_mux4oh #(
    parameter W     = 32, // default 32 bits: width
    parameter SEL_N = 4   // default 4 inputs (fixed: in3 .. in0)
) (
    input  wire [SEL_N-1:0] sel_oh, // SEL_N = 4: one-hot selects
    input  wire [W-1:0] in3,    // W = 32 bits: result if sel_oh[3]
    input  wire [W-1:0] in2,    // W = 32 bits: result if sel_oh[2]
    input  wire [W-1:0] in1,    // W = 32 bits: result if sel_oh[1]
    input  wire [W-1:0] in0,    // W = 32 bits: result if sel_oh[0]
    output wire [W-1:0] out     // W = 32 bits: the chosen result
);

    // ---- the select: one AND-OR level ----
    assign out = ({W{sel_oh[3]}} & in3) | ({W{sel_oh[2]}} & in2)
               | ({W{sel_oh[1]}} & in1) | ({W{sel_oh[0]}} & in0);

endmodule

`default_nettype wire
`endif // L1D_PF3_MUX4OH_V
