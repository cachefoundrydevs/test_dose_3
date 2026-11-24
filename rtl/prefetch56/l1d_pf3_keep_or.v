`ifndef L1D_PF3_KEEP_OR_V
`define L1D_PF3_KEEP_OR_V
`default_nettype none
(* keep_hierarchy *)
module l1d_pf3_keep_or #(
    parameter N = 8
) (
    input  wire [N-1:0] go,
    input  wire [N-1:0] drop,
    output wire         out
);
    assign out = |(go & ~drop);
endmodule
`default_nettype wire
`endif
