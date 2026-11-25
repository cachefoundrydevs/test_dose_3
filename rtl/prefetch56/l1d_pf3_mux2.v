// ==========================================================================
// l1d_pf3_mux2.v - a W-bit 2:1 select kept as its own level of hierarchy
//                  (the "late select last" of rule T-9, kept by synthesis)
// ==========================================================================
// IN SIMPLE WORDS : out = sel ? in1 : in0. The prefetch blocks work out the
//   result of a late decision both ways ahead of time and let the late bit
//   choose at the very end. Synthesis (ABC) sees the whole stage at once
//   and may fold the late bit back into the logic before the select; this
//   module is kept as its own level of hierarchy, so the select stays the
//   last gate before the flops and both inputs are built without the late
//   bit.
//
// STAGE / CYCLE   : combinational, the last level of the parent's stage.
// INSTANTIATED BY : l1d_pf3_sms_expand (slot pending / next line by
//   direct_go), l1d_pf3_rank (stream / SMS decisions by s_valid; SMS credit
//   per bank by m_win),
//   l1d_pf3_stream_card ({pending, ahead} by slot_go), l1d_pf3_sms_core
//   (S1 event fields by take_evt); one instance per late select.
//
// PARAMETERS:
//   name  default unit  meaning
//   W     32      bits  width
//
// PORTS:
//   sel   in  1  the late decision
//   in1   in  W  the result when sel = 1
//   in0   in  W  the result when sel = 0
//   out   out W  the chosen result
//
// WORKED EXAMPLE: W = 4, in1 = 4'b1010, in0 = 4'b0001, sel = 1 -> out =
//   4'b1010; sel = 0 -> out = 4'b0001.
//
// LOGIC LEVELS (T-9): 1 (one AO22 / MUX2 per bit).
// ASSUMPTIONS (V-9.2): none.
// ==========================================================================
`ifndef L1D_PF3_MUX2_V
`define L1D_PF3_MUX2_V
`default_nettype none

(* keep_hierarchy *)
module l1d_pf3_mux2 #(
    parameter W = 32  // default 32 bits: width
) (
    input  wire         sel, // the late decision
    input  wire [W-1:0] in1, // W = 32 bits: result if sel = 1
    input  wire [W-1:0] in0, // W = 32 bits: result if sel = 0
    output wire [W-1:0] out  // W = 32 bits: the chosen result
);

    // ---- the select ----
    assign out = sel ? in1 : in0;

endmodule

`default_nettype wire
`endif // L1D_PF3_MUX2_V
