`ifndef L1D_PF3_OR_ROWS_V
`define L1D_PF3_OR_ROWS_V
`default_nettype none
(* keep_hierarchy *)
module l1d_pf3_or_rows #(
    parameter W = 6,
    parameter N = 16
) (
    input  wire [N*W-1:0] rows,
    output reg  [W-1:0]   or_out
);
    always @* begin : or_proc
        integer row_idx;
        or_out = {W{1'b0}};
        for (row_idx = 0; row_idx < N; row_idx = row_idx + 1) begin
            or_out = or_out | rows[row_idx*W +: W];
        end
    end
endmodule
`default_nettype wire
`endif
