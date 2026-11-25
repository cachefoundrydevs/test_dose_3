// ==========================================================================
// l1d_pf3_or_rows.v - OR of N rows of W bits (one copy of a replicated
//                     reduction, fanout split)
// ==========================================================================
// IN SIMPLE WORDS : N registers each hold a W-bit value, at most one of them
//   non-zero (a one-hot pick that was registered per row); the output is
//   the OR of all rows, i.e. the picked value. A block that needs that
//   value in many places keeps several copies of this OR, each feeding a
//   few places (rule T-6). The module is kept as its own level of
//   hierarchy so synthesis does not merge the copies into one (same
//   inputs -> same logic would otherwise be shared).
//
// STAGE / CYCLE   : combinational, inside a stage of the parent.
// INSTANTIATED BY : l1d_pf3_stride (u_pick: the S1 picked event word per
//   group of event registers; u_prev_lip: the previous event's target per
//   copy; the S3 candidate line), l1d_pf3_stream (u_cand_or: "the offer
//   place is taken" per copy of the slot-free signal), l1d_pf3_sms_core
//   (u_tr_or / u_ev_or: "a record / an eviction now" per take_evt copy).
//
// PARAMETERS:
//   name  default unit  meaning
//   W     6       bits  row width
//   N     16      rows  number of rows
//
// PORTS:
//   rows    in  N x W  the rows, row r at [r*W +: W]
//   or_out  out W      OR of the rows
//
// WORKED EXAMPLE: W = 6, N = 16, row 5 = 6'd9, all others 0 -> or_out = 9.
//
// LOGIC LEVELS (T-9): log4(N) NOR/NAND levels (2 for 16).
// ASSUMPTIONS (V-9.2): none (any number of rows may be non-zero; the
//   output is then their OR).
// ==========================================================================
`ifndef L1D_PF3_OR_ROWS_V
`define L1D_PF3_OR_ROWS_V
`default_nettype none

(* keep_hierarchy *)
module l1d_pf3_or_rows #(
    parameter W = 6,  // default 6 bits: row width
    parameter N = 16  // default 16 rows
) (
    input  wire [N*W-1:0] rows,   // N x W = 16 x 6 bits
    output reg  [W-1:0]   or_out  // W = 6 bits: OR of the rows
);

    always @* begin : or_proc
        integer row_idx; // walks the rows
        or_out = {W{1'b0}};
        for (row_idx = 0; row_idx < N; row_idx = row_idx + 1) begin
            or_out = or_out | rows[row_idx*W +: W];
        end // for row_idx
    end // always or_proc

endmodule

`default_nettype wire
`endif // L1D_PF3_OR_ROWS_V
