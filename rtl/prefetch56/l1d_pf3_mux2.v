`ifndef L1D_PF3_MUX2_V
`define L1D_PF3_MUX2_V
`default_nettype none
(* keep_hierarchy *)
module l1d_pf3_mux2 #(
    parameter W = 32
) (
    input  wire         sel,
    input  wire [W-1:0] in1,
    input  wire [W-1:0] in0,
    output wire [W-1:0] out
);
    assign out = sel ? in1 : in0;
endmodule
`default_nettype wire
`endif
