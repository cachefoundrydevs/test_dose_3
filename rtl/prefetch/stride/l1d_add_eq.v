// l1d_add_eq.v
// Carry-free sum compare: eq = (a + b == c), modulo 2^W, without an adder.
// Used for: l1d_pf_stride D stage ("is this delta the stored stride":
// lo == last_lo + stride). a + b == c  <=>  a + b + ~c == all ones
// <=>  S ^ (C << 1) == all ones, with the carry-save sum S = ~(a^b^c)
// and carry C = maj(a, b, ~c). Depth: 1 XOR3 / MAJ, 1 XOR, a W-input AND
// (no carry chain).
// Assumptions (V-9.2): none (any W >= 2).
// Combinational outputs (V-3.11): eq (a compare block, no flops).

`ifndef L1D_ADD_EQ_V
`define L1D_ADD_EQ_V
`default_nettype none

(* keep_hierarchy *)
module l1d_add_eq #(
   parameter W = 13                       // 13 bits (pf_stride delta)
) (
   a,
   b,
   c,
   eq
);

   input  wire [W-1:0]  a;                // W = 13 bits
   input  wire [W-1:0]  b;                // W = 13 bits
   input  wire [W-1:0]  c;                // W = 13 bits
   output reg           eq;               // a + b == c (mod 2^W)

   reg  [W-1:0]         csa_s;            // W = 13 bits, carry-save sum
   reg  [W-2:0]         csa_c;            // W-1 = 12 bits, carry-save carry
                                          // (the top carry leaves 2^W)

   always @* begin : eq_proc
      csa_s = ~(a ^ b ^ c);
      csa_c = (a[W-2:0] & b[W-2:0]) | (a[W-2:0] & ~c[W-2:0]) |
              (b[W-2:0] & ~c[W-2:0]);
      eq    = &(csa_s ^ {csa_c, 1'b0});
   end // always eq_proc

endmodule

`default_nettype wire
`endif // L1D_ADD_EQ_V
