// ==========================================================================
// l1d_pf3_ks_add.v - Kogge-Stone adder / subtractor (combinational)
// ==========================================================================
// IN SIMPLE WORDS : adds (or subtracts) two W-bit numbers with a carry
//   tree of log2(W) levels instead of a ripple chain. The flow maps a
//   plain "+" of 13 bits into a 9-cell ripple chain (measured, stride
//   DESIGN.md section 8), so every adder of the prefetch system that sits
//   on a timed path uses this block.
// STAGE / CYCLE   : combinational, inside the stage of its parent.
// INSTANTIATED BY : l1d_pf3_stride_entry (4 copies per entry),
//   l1d_pf3_stride (virtual entry, S1 lane math), l1d_pf3_stream.
//
// PARAMETERS:
//   name  default  unit  meaning
//   W     13       bits  operand and result width
//   SUB   0        flag  0: sum = a + b, 1: sum = a - b (two's complement)
//
// PORTS (default widths):
//   a    in  13  first operand
//   b    in  13  second operand
//   sum  out 13  a + b or a - b, modulo 2**W (combinational)
//
// LOGIC LEVELS (T-9, estimate): generate/propagate 1, prefix
//   ceil(log2(W)) AO21 levels (4 at W = 13 to 16), sum XOR 1: 6 cells.
//
// WORKED EXAMPLE : W = 13, SUB = 1, a = 5, b = 7 -> sum = 8190 (= -2).
//
// ASSUMPTIONS (V-9.2): A1 W >= 2. Nothing else: pure combinational logic.
// SYNTHESIS: kept as its own level of hierarchy (keep_hierarchy), so the
//   mapper keeps the Kogge-Stone carry tree instead of turning it into a
//   ripple (MAJ) chain inside the surrounding logic (seen in the
//   whole-unit runs).
// ==========================================================================
`ifndef L1D_PF3_KS_ADD_V
`define L1D_PF3_KS_ADD_V
`default_nettype none

(* keep_hierarchy *)
module l1d_pf3_ks_add #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter W   = 13, // default 13 bits: operand width
    parameter SUB = 0   // default 0: 0 = add, 1 = subtract
) (
    // ---- operands in ----
    input  wire [W-1:0] a,   // W = 13 bits, first operand
    input  wire [W-1:0] b,   // W = 13 bits, second operand
    // ---- result out (combinational) ----
    output wire [W-1:0] sum  // W = 13 bits, a + b or a - b
);

    // ---- local constants (value at the default) ----
    localparam LEVELS = $clog2(W);                 // 4 prefix levels

    // ---- declarations ----
    wire [W-1:0] b_eff;          // W = 13 bits: b, or ~b for a subtract
    wire         carry_in;       // 1 for a subtract (two's complement)
    reg  [W-1:0] gen;            // W = 13 bits: carry generate per bit
    reg  [W-1:0] prop;           // W = 13 bits: carry propagate per bit
    reg  [W-1:0] gen_level;      // W = 13 bits: group generate, running
    reg  [W-1:0] prop_level;     // W = 13 bits: group propagate, running
    reg  [W-1:0] gen_next;       // W = 13 bits: group generate, next level
    reg  [W-1:0] prop_next;      // W = 13 bits: group propagate, next
    reg  [W-1:0] carry;          // W = 13 bits: carry into each bit
    reg  [W-1:0] sum_bits;       // W = 13 bits: the result

    assign b_eff    = (SUB != 0) ? ~b : b;
    assign carry_in = (SUB != 0) ? 1'b1 : 1'b0;

    // ======================================================================
    // carry tree: generate / propagate, then log2(W) prefix levels
    // ======================================================================
    always @* begin : prefix_proc
        integer level_idx; // walks the prefix levels
        integer bit_idx; // walks the bits
        integer span; // distance of the prefix combine
        // bit level: the carry-in is folded into bit 0's generate
        gen  = a & b_eff;
        prop = a ^ b_eff;
        gen_level  = gen;
        prop_level = prop;
        gen_level[0] = gen[0] | (prop[0] & carry_in);
        // Kogge-Stone: at level k every bit combines with the bit 2**k
        // below it, so after LEVELS levels each bit holds its full group
        span = 1;
        for (level_idx = 0; level_idx < LEVELS; level_idx = level_idx + 1) begin
            for (bit_idx = 0; bit_idx < W; bit_idx = bit_idx + 1) begin
                if (bit_idx >= span) begin
                    gen_next[bit_idx]  = gen_level[bit_idx]
                        | (prop_level[bit_idx] & gen_level[bit_idx - span]);
                    prop_next[bit_idx] = prop_level[bit_idx]
                        & prop_level[bit_idx - span];
                end
                else begin
                    gen_next[bit_idx]  = gen_level[bit_idx];
                    prop_next[bit_idx] = prop_level[bit_idx];
                end // else (bit below the span)
            end // for bit_idx
            gen_level  = gen_next;
            prop_level = prop_next;
            span       = span * 2;
        end // for level_idx
        // carry into bit i is the group generate of bits i-1 .. 0
        carry[0] = carry_in;
        for (bit_idx = 1; bit_idx < W; bit_idx = bit_idx + 1) begin
            carry[bit_idx] = gen_level[bit_idx - 1];
        end // for bit_idx
        sum_bits = prop ^ carry;
    end // always prefix_proc

    assign sum = sum_bits;

endmodule

`default_nettype wire
`endif // L1D_PF3_KS_ADD_V
