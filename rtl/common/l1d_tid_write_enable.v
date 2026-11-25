`ifndef L1D_TID_WRITE_ENABLE_V
`define L1D_TID_WRITE_ENABLE_V
`default_nettype none
(* keep_hierarchy *)
module l1d_tid_write_enable #(
    parameter N = 3
) (
    input  wire [N-1:0] block_flags,
    output wire         write_enable
);
    assign write_enable = ~|block_flags;
endmodule
`default_nettype wire
`endif
