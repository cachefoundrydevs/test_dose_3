// ==========================================================================
// l1d_pf3_rep.v - one copy of a replicated register (fanout split)
// ==========================================================================
// IN SIMPLE WORDS : a plain row of flops that copies its input every clock.
//   A block that must send one registered value to many places (the S1
//   event register of a table with 64 entries, for example) keeps several
//   copies of it, each copy driving only a few places, so no long buffer
//   chain is needed (rule T-6). The module is kept as its own level of
//   hierarchy so synthesis does not merge the copies back into one
//   (same input -> same register would otherwise be shared).
//
// STAGE / CYCLE   : any; the copy is one clock later than its input, like
//   any register of the stage that writes it.
// INSTANTIATED BY : l1d_pf3_stride (event copies, filter copies) and other
//   prefetch blocks, inside generate loops, one instance per copy.
//
// PARAMETERS:
//   name     default unit  meaning
//   W        8       bits  width of the register
//   HAS_RST  0       flag  1: synchronous reset to 0 by rst
//   HAS_EN   0       flag  1: the register loads only when en is 1
//   (chosen by parameter, not by a constant on a port, because a kept
//   level of hierarchy does not see constants on its ports)
//
// PORTS:
//   clk  in  1   clock
//   rst  in  1   synchronous reset, high (used when HAS_RST = 1)
//   en   in  1   load enable (used when HAS_EN = 1)
//   d    in  W   value to copy (from the stage logic)
//   q    out W   registered copy (flop outputs)
//
// WORKED EXAMPLE: W = 8, d = 0x5A in cycle 3 -> q = 0x5A in cycle 4.
//
// LOGIC LEVELS (T-9): none (HAS_EN: one 2:1 before the flop, HAS_RST:
//   one AND before the flop).
// ASSUMPTIONS (V-9.2): none.
// ==========================================================================
`ifndef L1D_PF3_REP_V
`define L1D_PF3_REP_V
`default_nettype none

(* keep_hierarchy *)
module l1d_pf3_rep #(
    parameter W       = 8, // default 8 bits: register width
    parameter HAS_RST = 0, // default 0: 1 = reset to 0 by rst
    parameter HAS_EN  = 0  // default 0: 1 = load only when en
) (
    input  wire         clk, // clock
    input  wire         rst, // sync reset, high (HAS_RST = 1)
    input  wire         en,  // load enable (HAS_EN = 1)
    input  wire [W-1:0] d,   // W = 8 bits: value to copy
    output reg  [W-1:0] q    // W = 8 bits: the copy, one clock later
);

    // ---- the copy ----
    always @(posedge clk) begin : copy_proc
        if ((HAS_RST != 0) && rst) begin
            q <= {W{1'b0}};       // reset to zero
        end
        else if ((HAS_EN == 0) || en) begin
            q <= d;               // copy the stage value
        end // rst / en
    end // always copy_proc

endmodule

`default_nettype wire
`endif // L1D_PF3_REP_V
