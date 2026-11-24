`ifndef L1D_PF3_REP_V
`define L1D_PF3_REP_V
`default_nettype none
(* keep_hierarchy *)
module l1d_pf3_rep #(
    parameter W       = 8,
    parameter HAS_RST = 0,
    parameter HAS_EN  = 0
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         en,
    input  wire [W-1:0] d,
    output reg  [W-1:0] q
);
    always @(posedge clk) begin : copy_proc
        if ((HAS_RST != 0) && rst) begin
            q <= {W{1'b0}};
        end
        else if ((HAS_EN == 0) || en) begin
            q <= d;
        end
    end
endmodule
`default_nettype wire
`endif
