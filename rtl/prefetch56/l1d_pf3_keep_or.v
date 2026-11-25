// ==========================================================================
// l1d_pf3_keep_or.v - "any go without a drop": out = OR of (go AND NOT
//                     drop), kept as its own level of hierarchy
// ==========================================================================
// IN SIMPLE WORDS : the rank keeps (holds) a stream or SMS offer when its
//   case is chosen, the offer was not sent ("go"), and it is not the line
//   of a higher offer ("drop", from the 58-bit line compares, the late
//   part). Written flat, synthesis put the compare results at the start
//   of a long decision chain (run top54: compare done at 213 ps, flop D at
//   318 ps). In this small kept block every input is one or two gates
//   from the output, so a late drop bit passes about 3 levels.
//
// STAGE / CYCLE   : combinational, S3 of l1d_pf3_rank, in front of the
//   hold valid flops t_hold_vld / m_hold_vld.
// INSTANTIATED BY : l1d_pf3_rank, u_t_keep (N = 4) and u_m_keep (N = 8).
//
// PARAMETERS:
//   name  default unit   meaning
//   N     8       terms  number of (go, drop) pairs
//
// PORTS:
//   go    in  N  term k would keep the offer
//   drop  in  N  term k's offer is a duplicate (dropped)
//   out   out 1  some term keeps: OR over k of (go[k] AND NOT drop[k])
//
// WORKED EXAMPLE: N = 2, go = 10, drop = 10 -> out = 0 (term 1 is a
//   duplicate); drop = 00 -> out = 1.
//
// LOGIC LEVELS (T-9): AND-NOT 1 + OR tree log4(N) (2 for 8) = 3.
// ASSUMPTIONS (V-9.2): none.
// ==========================================================================
`ifndef L1D_PF3_KEEP_OR_V
`define L1D_PF3_KEEP_OR_V
`default_nettype none

(* keep_hierarchy *)
module l1d_pf3_keep_or #(
    parameter N = 8  // default 8 terms
) (
    input  wire [N-1:0] go,   // N = 8: term would keep
    input  wire [N-1:0] drop, // N = 8: term is a duplicate
    output wire         out   // 1: some term keeps
);

    assign out = |(go & ~drop);

endmodule

`default_nettype wire
`endif // L1D_PF3_KEEP_OR_V
