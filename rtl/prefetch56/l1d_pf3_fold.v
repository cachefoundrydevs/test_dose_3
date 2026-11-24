`ifndef L1D_PF3_FOLD_V
`define L1D_PF3_FOLD_V
`default_nettype none
(* keep_hierarchy *)
module l1d_pf3_fold #(
    parameter IN_W  = 59,
    parameter OUT_W = 12
) (
    input  wire [IN_W-1:0]  din,
    output reg  [OUT_W-1:0] dout
);
    always @* begin : fold_proc
        integer bit_idx;
        dout = {OUT_W{1'b0}};
        for (bit_idx = 0; bit_idx < IN_W; bit_idx = bit_idx + 1) begin
            dout[bit_idx % OUT_W] = dout[bit_idx % OUT_W] ^ din[bit_idx];
        end
    end
endmodule
`default_nettype wire
`endif
