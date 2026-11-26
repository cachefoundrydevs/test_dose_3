// l1d_ks_add.v
// Kogge-Stone (parallel prefix) adder: sum = a + b + cin, modulo 2^W.
// Used for: l1d_pf_stride (deltas and the prefetch target). Written out
// so the adder is log2(W) deep in every flow: the repo's Yosys + ABC
// flow maps a plain "+" to a ripple carry (13 bits = 9 MAJ cells, about
// 215 ps in ASAP7, measured 2026-11-04). l1d_pf_stride uses it through
// l1d_ks_add_q (operand flops and adder in one kept hierarchy).
// Assumptions (V-9.2): none (any W >= 1).
// Combinational outputs (V-3.11): sum (an arithmetic block, no flops).

`ifndef L1D_KS_ADD_V
`define L1D_KS_ADD_V
`default_nettype none

module l1d_ks_add #(
   parameter W = 13                       // 13 bits (pf_stride delta)
) (
   cin,
   a,
   b,
   sum
);

   localparam LV = (W > 1) ? $clog2(W) : 1;   // 4 prefix levels at 13

   input  wire          cin;              // carry in (1 for a - b)
   input  wire [W-1:0]  a;                // W = 13 bits
   input  wire [W-1:0]  b;                // W = 13 bits
   output reg  [W-1:0]  sum;              // W = 13 bits

   reg  [W-1:0]         p0;               // W = 13 bits, propagate
   reg  [W-1:0]         gen;              // W = 13 bits, group generate
   reg  [W-1:0]         prp;              // W = 13 bits, group propagate
   reg  [W-1:0]         gen_n;            // W = 13 bits, next level
   reg  [W-1:0]         prp_n;            // W = 13 bits, next level

   always @* begin : add_proc
      integer i;                          // bit index
      integer k;                          // prefix level
      p0  = a ^ b;
      gen = a & b;
      gen[0] = gen[0] | (p0[0] & cin);    // cin folded into bit 0
      prp = p0;
      for (k = 0; k < LV; k = k + 1) begin
         gen_n = gen;
         prp_n = prp;
         for (i = 0; i < W; i = i + 1) begin
            if (i >= (1 << k)) begin
               gen_n[i] = gen[i] | (prp[i] & gen[i - (1 << k)]);
               prp_n[i] = prp[i] & prp[i - (1 << k)];
            end
         end
         gen = gen_n;
         prp = prp_n;
      end
      // carry into bit i is the group generate of bits i-1 .. 0
      sum[0] = p0[0] ^ cin;
      for (i = 1; i < W; i = i + 1) begin
         sum[i] = p0[i] ^ gen[i - 1];
      end
   end // always add_proc

endmodule

`default_nettype wire
`endif // L1D_KS_ADD_V
