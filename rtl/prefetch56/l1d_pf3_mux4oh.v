`ifndef L1D_PF3_MUX4OH_V
`define L1D_PF3_MUX4OH_V
`default_nettype none
(* keep_hierarchy *)
module l1d_pf3_mux4oh #(
    parameter W     = 32,
    parameter SEL_N = 4
) (
    input  wire [SEL_N-1:0] sel_oh,
    input  wire [W-1:0] in3,
    input  wire [W-1:0] in2,
    input  wire [W-1:0] in1,
    input  wire [W-1:0] in0,
    output wire [W-1:0] out
);
    assign out = ({W{sel_oh[3]}} & in3) | ({W{sel_oh[2]}} & in2)
               | ({W{sel_oh[1]}} & in1) | ({W{sel_oh[0]}} & in0);
endmodule
`default_nettype wire
`endif
