// l1d_ks_add_q.v
// Registered-input Kogge-Stone adder: a_q, b_q <= a_d, b_d on every
// clock; sum = a_q + b_q (SUB = 0) or a_q - b_q (SUB = 1), modulo 2^W,
// combinational from the flops.
// Used for: l1d_pf_stride (the deltas and the prefetch target). The
// operand flops sit inside the kept hierarchy so ABC maps flop, operand
// inversion and prefix tree together; with the flops outside, the
// flop's Q inverter was left unsized at the module pins (one INVx1
// driving 13 loads, 40 ps, measured 2026-11-04).
// Assumptions (V-9.2): none (any W >= 1).
// Combinational outputs (V-3.11): sum (from the internal flops).

`ifndef L1D_KS_ADD_Q_V
`define L1D_KS_ADD_Q_V
`default_nettype none

(* keep_hierarchy *)
module l1d_ks_add_q #(
   parameter W   = 13,                    // 13 bits (pf_stride delta)
   parameter SUB = 1                      // 1: a - b, 0: a + b
) (
   clk,
   a_d,
   b_d,
   sum
);

   input  wire          clk;              // clock
   input  wire [W-1:0]  a_d;              // W = 13 bits, next a
   input  wire [W-1:0]  b_d;              // W = 13 bits, next b
   output wire [W-1:0]  sum;              // W = 13 bits

   reg  [W-1:0]         a_q;              // W = 13 bits, operand flops
   reg  [W-1:0]         b_q;              // W = 13 bits (no reset: data)
   wire [W-1:0]         b_x;              // W = 13 bits, b or ~b
   wire                 cin;              // carry in: 1 for a - b

   always @(posedge clk) begin : op_reg_proc
      a_q <= a_d;
      b_q <= b_d;
   end // always op_reg_proc

   assign b_x = (SUB != 0) ? ~b_q : b_q;
   assign cin = (SUB != 0) ? 1'b1 : 1'b0;

   l1d_ks_add #(.W(W)) u_ks_add (
      .cin (cin),
      .a   (a_q),
      .b   (b_x),
      .sum (sum)
   );

endmodule

`default_nettype wire
`endif // L1D_KS_ADD_Q_V
