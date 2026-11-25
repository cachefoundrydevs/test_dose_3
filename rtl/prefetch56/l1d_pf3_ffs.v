// ==========================================================================
// l1d_pf3_ffs.v - find first set: the lowest 1 of a vector (and the second
//                 lowest), as one-hots, built as a parallel prefix (log
//                 depth, rule T-9)
// ==========================================================================
// IN SIMPLE WORDS : a row of W boxes, some ticked. The output keeps only the
//   lowest ticked box. Written as a loop ("seen so far") the logic becomes
//   a chain of W gates one after the other; here every box instead learns
//   "is any lower box ticked?" from a prefix OR tree, which doubles the
//   distance it looks back at each level, so only log2(W) levels are needed
//   (5 for 32 boxes). The same idea as a Kogge-Stone adder's carry. A
//   second tree, "are two or more lower boxes ticked?", gives the second
//   lowest ticked box the same way (pairs of counts saturating at 2).
//
// STAGE / CYCLE   : combinational, used inside a stage of the parent.
// INSTANTIATED BY : l1d_pf3_sms_core (PHT write: first / second line, FT /
//   AT free slot), l1d_pf3_sms_expand (next line of a slot).
//
// PARAMETERS:
//   name  default unit  meaning
//   W     32      bits  vector width (any width >= 2)
//
// PORTS:
//   in_vec   in  W  the vector
//   low_oh   out W  its lowest 1 as a one-hot (all 0 if none)
//   low2_oh  out W  its second lowest 1 as a one-hot (0 if fewer than 2)
//   any_set  out 1  in_vec has a 1
//   two_set  out 1  in_vec has two 1s or more
//
// WORKED EXAMPLE: W = 8, in_vec = 8'b0110_1000 -> low_oh = 8'b0000_1000,
//   low2_oh = 8'b0010_0000, any_set = 1, two_set = 1. in_vec = 0 -> all 0.
//
// LOGIC LEVELS (T-9): prefix clog2(W) levels (5 for 32, an AO21 each for
//   the two-or-more tree) + AND 1. Unused outputs are removed by synthesis.
// ASSUMPTIONS (V-9.2): none.
// SYNTHESIS: kept as its own level of hierarchy (keep_hierarchy), so the
//   mapper keeps the log-depth prefix tree instead of folding it into the
//   surrounding logic as a chain of gates (seen in the whole-unit runs).
// ==========================================================================
`ifndef L1D_PF3_FFS_V
`define L1D_PF3_FFS_V
`default_nettype none

(* keep_hierarchy *)
module l1d_pf3_ffs #(
    parameter W = 32 // default 32 bits: vector width
) (
    input  wire [W-1:0] in_vec,  // W = 32 bits: the vector
    output wire [W-1:0] low_oh,  // W = 32 bits: lowest 1, one-hot
    output wire [W-1:0] low2_oh, // W = 32 bits: second lowest 1, one-hot
    output wire         any_set, // the vector has a 1
    output wire         two_set  // the vector has two 1s or more
);

    // ---- local constants (value at the default) ----
    localparam LEVELS = $clog2(W);                  // 5 levels

    // prefix[l] bit i = OR of in_vec[i - 2^l + 1 .. i] (inclusive prefix
    // after level l); level 0 is the input itself
    wire [(LEVELS+1)*W-1:0] prefix;                 // 6 x 32 bits
    // prefix2[l] bit i = two or more 1s in the same window
    wire [(LEVELS+1)*W-1:0] prefix2;                // 6 x 32 bits
    wire [W-1:0]            lower;                  // 32: a lower bit is 1
    wire [W-1:0]            lower2;                 // 32: two lower bits are 1

    assign prefix[W-1:0]  = in_vec;
    assign prefix2[W-1:0] = {W{1'b0}};

    genvar level_g; // generate index: prefix level
    genvar bit_g;   // generate index: bit
    generate
        for (level_g = 0; level_g < LEVELS; level_g = level_g + 1) begin : g_level
            for (bit_g = 0; bit_g < W; bit_g = bit_g + 1) begin : g_bit
                if (bit_g >= (1 << level_g)) begin : g_or
                    // look back 2^level bits
                    assign prefix[(level_g+1)*W + bit_g] =
                        prefix[level_g*W + bit_g]
                        | prefix[level_g*W + bit_g - (1 << level_g)];
                    // two or more: in either half, or one in each half
                    assign prefix2[(level_g+1)*W + bit_g] =
                        prefix2[level_g*W + bit_g]
                        | prefix2[level_g*W + bit_g - (1 << level_g)]
                        | (prefix[level_g*W + bit_g]
                           & prefix[level_g*W + bit_g - (1 << level_g)]);
                end
                else begin : g_pass
                    assign prefix[(level_g+1)*W + bit_g]  = prefix[level_g*W + bit_g];
                    assign prefix2[(level_g+1)*W + bit_g] = prefix2[level_g*W + bit_g];
                end // else (nothing further back)
            end // for bit_g
        end // for level_g
    endgenerate

    // a lower bit is 1: the inclusive prefix of the bit below
    assign lower   = {prefix[LEVELS*W +: W-1], 1'b0};
    assign lower2  = {prefix2[LEVELS*W +: W-1], 1'b0};
    assign low_oh  = in_vec & ~lower;
    assign low2_oh = in_vec & lower & ~lower2;
    assign any_set = prefix[LEVELS*W + W-1];
    assign two_set = prefix2[LEVELS*W + W-1];

endmodule

`default_nettype wire
`endif // L1D_PF3_FFS_V
