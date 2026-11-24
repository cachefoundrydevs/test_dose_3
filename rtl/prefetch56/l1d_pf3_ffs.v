`ifndef L1D_PF3_FFS_V
`define L1D_PF3_FFS_V
`default_nettype none
(* keep_hierarchy *)
module l1d_pf3_ffs #(
    parameter W = 32
) (
    input  wire [W-1:0] in_vec,
    output wire [W-1:0] low_oh,
    output wire [W-1:0] low2_oh,
    output wire         any_set,
    output wire         two_set
);
    localparam LEVELS = $clog2(W);
    wire [(LEVELS+1)*W-1:0] prefix;
    wire [(LEVELS+1)*W-1:0] prefix2;
    wire [W-1:0]            lower;
    wire [W-1:0]            lower2;
    assign prefix[W-1:0]  = in_vec;
    assign prefix2[W-1:0] = {W{1'b0}};
    genvar level_g;
    genvar bit_g;
    generate
        for (level_g = 0; level_g < LEVELS; level_g = level_g + 1) begin : g_level
            for (bit_g = 0; bit_g < W; bit_g = bit_g + 1) begin : g_bit
                if (bit_g >= (1 << level_g)) begin : g_or
                    assign prefix[(level_g+1)*W + bit_g] =
                        prefix[level_g*W + bit_g]
                        | prefix[level_g*W + bit_g - (1 << level_g)];
                    assign prefix2[(level_g+1)*W + bit_g] =
                        prefix2[level_g*W + bit_g]
                        | prefix2[level_g*W + bit_g - (1 << level_g)]
                        | (prefix[level_g*W + bit_g]
                           & prefix[level_g*W + bit_g - (1 << level_g)]);
                end
                else begin : g_pass
                    assign prefix[(level_g+1)*W + bit_g]  = prefix[level_g*W + bit_g];
                    assign prefix2[(level_g+1)*W + bit_g] = prefix2[level_g*W + bit_g];
                end
            end
        end
    endgenerate
    assign lower   = {prefix[LEVELS*W +: W-1], 1'b0};
    assign lower2  = {prefix2[LEVELS*W +: W-1], 1'b0};
    assign low_oh  = in_vec & ~lower;
    assign low2_oh = in_vec & lower & ~lower2;
    assign any_set = prefix[LEVELS*W + W-1];
    assign two_set = prefix2[LEVELS*W + W-1];
endmodule
`default_nettype wire
`endif
