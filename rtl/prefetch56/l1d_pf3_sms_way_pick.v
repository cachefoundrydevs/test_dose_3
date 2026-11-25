// ==========================================================================
// l1d_pf3_sms_way_pick.v - the PHT way a pattern is written to, for one
//                          set (kept as its own level of hierarchy, so the
//                          SMS core can use several copies of it)
// ==========================================================================
// IN SIMPLE WORDS : a footprint is written to one way of its PHT set: the
//   way that already holds its tag; else the first empty way; else the way
//   the set's ring pointer shows (and the ring then moves on). Every way
//   write enable drives many data flops (tag, lines, pattern), so the SMS
//   core uses three copies of this pick per set, each for one group of
//   flops; the copies are kept apart, so synthesis does not merge them
//   back into one net with a deep buffer tree (rule T-6).
//
// STAGE / CYCLE   : W2, combinational, from flops only.
// INSTANTIATED BY : l1d_pf3_sms_core, g_way_set[set].g_way_copy[copy].u_pick,
//   PHT_SETS x WP_COPIES = 32 x 3 copies.
//
// PARAMETERS:
//   name      default unit  meaning
//   WAYS      4       ways  ways of a PHT set (power of two, >= 2)
//
// PORTS (default widths):
//   set_go     in  1  this set is written now (registered x_vld AND set)
//   tag_match  in  4  ways that hold the footprint's tag (at most one, A5
//                     of l1d_pf3_sms_core)
//   way_valid  in  4  ways that hold a pattern
//   ring_way   in  2  the set's ring pointer
//   way_write  out 4  the way written now, one-hot (0 when set_go is 0)
//   ring_adv   out 1  the ring pointer moves (no match, no empty way)
//
// WORKED EXAMPLE: set_go = 1, tag_match = 0000, way_valid = 1011,
//   ring_way = 1 -> the first empty way is way 2: way_write = 0100,
//   ring_adv = 0. With way_valid = 1111: way_write = 0010 (way 1), ring_adv
//   = 1. With tag_match = 1000: way_write = 1000, ring_adv = 0.
//
// LOGIC LEVELS (T-9): first empty way 2 + ring decode 1 (in parallel),
//   match OR 1, then one AND-OR per way 2 = about 4, from flops.
// ASSUMPTIONS (V-9.2): A1 tag_match has at most one bit set (A5 of the
//   SMS core; checked there under L1D_DEBUG).
// ==========================================================================
`ifndef L1D_PF3_SMS_WAY_PICK_V
`define L1D_PF3_SMS_WAY_PICK_V
`default_nettype none

(* keep_hierarchy *)
module l1d_pf3_sms_way_pick #(
    parameter WAYS = 4  // default 4 ways
) (
    input  wire                    set_go,    // this set is written now
    input  wire [WAYS-1:0]         tag_match, // WAYS = 4: ways with the tag
    input  wire [WAYS-1:0]         way_valid, // WAYS = 4: ways in use
    input  wire [$clog2(WAYS)-1:0] ring_way,  // WAY_W = 2 bits: ring way
    output wire [WAYS-1:0]         way_write, // WAYS = 4: way written now
    output wire                    ring_adv   // the ring pointer moves
);

    // ---- signals ----
    reg  [WAYS-1:0] free_first; // WAYS = 4: first empty way, one-hot
    reg  [WAYS-1:0] ring_oh;    // WAYS = 4: ring way, one-hot
    reg             free_seen;  // scratch: an empty way lies below
    wire            any_free;   // a way is empty
    wire            any_match;  // a way holds the tag
    wire [WAYS-1:0] victim;     // WAYS = 4: empty way, else the ring way

    // ---- first empty way and ring way, both from flops ----
    always @* begin : free_proc
        integer way_idx; // walks the ways
        free_first = {WAYS{1'b0}};
        free_seen  = 1'b0;
        for (way_idx = 0; way_idx < WAYS; way_idx = way_idx + 1) begin
            free_first[way_idx] = ~way_valid[way_idx] & ~free_seen;
            free_seen           = free_seen | ~way_valid[way_idx];
        end // for way_idx
        ring_oh = {WAYS{1'b0}};
        ring_oh[ring_way] = 1'b1;
    end // always free_proc

    assign any_free  = ~(&way_valid);
    assign any_match = |tag_match;
    assign victim    = any_free ? free_first : ring_oh;

    // ---- the pick: the tag's way, else the victim ----
    assign way_write = (tag_match | (victim & {WAYS{~any_match}}))
                       & {WAYS{set_go}};
    assign ring_adv  = set_go & ~any_match & ~any_free;

endmodule

`default_nettype wire
`endif // L1D_PF3_SMS_WAY_PICK_V
