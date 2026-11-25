// ============================================================================
// l1d_s1_lookup.v  -  Stage 1 lookup: is the line here, and which way would be replaced?
// ============================================================================
// IN SIMPLE WORDS
//   In Stage 1 the tag RAM has delivered the tags of ALL ways of the set, and the state flops the states of all
//   ways. This block does three things at once, all combinational:
//     1. HIT?     compares lookup_tag with the tag of every way. A way can hit only if its state is not I (a stale
//                 tag in an invalid way must be ignored). hit = any way matched, hit_way = its number.
//     2. VICTIM?  victim_way = the way to replace on a refill: the FIRST INVALID way if there is one, otherwise the
//                 way the PLRU tree points to.
//     3. PLRU     plru_bits_next = the PLRU bits after an access to accessed_way (the way the caller will use: the hit
//                 way for a CPU request, the victim for a refill).
//
//   Worked example (4 ways): states = I, E, E, M and the tags of ways 1..3 are A, B, C. lookup_tag = B ->
//   match = 0100 (bit 2), hit = 1, hit_way = 2. lookup_tag = D -> hit = 0; victim_way = 0 (the first invalid way).
//
//   Doing the tag compare, the way choice and the data RAM address in ONE cycle is a known timing risk (T-3).
//
// WHAT KIND OF LOGIC IS IT?  Combinational (no clock, no state). The comparator is one generate loop over WAYS.
//
// PORTS (new name, old name, meaning)
//   way_tags_flat                        (was tags)                     tag of every way, laid end to end
//   way_states_flat                      (was states)                   state (2 bits) of every way
//   plru_bits                            (was plru)                     PLRU bits of the set
//   lookup_tag                           (was tag)                      tag of the line being looked up
//   accessed_way                         (was acc_way)                  way the caller will access (for the PLRU update)
//   plru_bits_next                       (was plru_new)                 PLRU bits after that access
//   hit                                                                 lookup_tag matches a valid way
//   hit_way                                                             the way that matched (valid when hit = 1)
//   victim_way                                                          way to replace on a refill: the first invalid way, otherwise the PLRU way
//
// VERILOG WORDS USED IN THIS FILE
//   way_tags_flat[w*TAG_W +: TAG_W]   the tag of way w (TAG_W bits starting at bit w*TAG_W)
//   way_states_flat[2*w +: 2]         the 2-bit state of way w
//   |match                            reduction OR: 1 if any way matched
//   genvar w / generate for           builds one comparator per way
// ============================================================================

`default_nettype none

module l1d_s1_lookup #(
    parameter WAYS  = 4,
    parameter TAG_W = 50,
    parameter WAY_W = $clog2(WAYS),
    parameter NB    = WAYS - 1
) (
    input  wire [WAYS*TAG_W-1:0] way_tags_flat,           // tag of every way, laid end to end
    input  wire [2*WAYS-1:0]     way_states_flat,         // state (2 bits) of every way
    input  wire [NB-1:0]         plru_bits,               // PLRU bits of the set
    input  wire [TAG_W-1:0]      lookup_tag,              // tag of the line being looked up
    input  wire [WAY_W-1:0]      accessed_way,            // way the caller will access (for the PLRU update)
    output wire                  hit,                     // lookup_tag matches a valid way
    output wire [WAY_W-1:0]      hit_way,                 // the way that matched (valid when hit = 1)
    output wire [WAY_W-1:0]      victim_way,              // way to replace on a refill: the first invalid way, otherwise the PLRU way
    output wire [NB-1:0]         plru_bits_next           // PLRU bits after that access
);

    // valid[w] = way w holds a line (state is not I);  match[w] = it holds THIS line (valid and equal tag)
    wire [WAYS-1:0] valid;
    wire [WAYS-1:0] match;

    // one comparator per way (g_way[0] .. g_way[WAYS-1])
    genvar w;
    generate
        for (w = 0; w < WAYS; w = w + 1) begin : g_way
            assign valid[w] = (way_states_flat[2*w +: 2] != 2'b00);                    // not ST_I
            assign match[w] = valid[w] && (way_tags_flat[w*TAG_W +: TAG_W] == lookup_tag);
        end
    endgenerate

    // hit = at least one way matched; l1d_pri_enc turns the one-hot match into the number hit_way
    assign hit = |match;

    l1d_pri_enc #(.N(WAYS), .IDX_W(WAY_W)) u_hit (
        .request_bits     (match),
        .lowest_set_index (hit_way),
        .any_set          ()
    );

    // first invalid way (request_bits = ~valid): preferred victim, because replacing an invalid way evicts nothing
    wire [WAY_W-1:0] inv_way;
    wire             any_inv;
    l1d_pri_enc #(.N(WAYS), .IDX_W(WAY_W)) u_inv (
        .request_bits     (~valid),
        .lowest_set_index (inv_way),
        .any_set          (any_inv)
    );

    // the PLRU tree: its victim, and the PLRU bits after an access to accessed_way
    wire [WAY_W-1:0] plru_victim;
    l1d_plru #(.WAYS(WAYS), .WAY_W(WAY_W), .NB(NB)) u_plru (
        .plru_bits      (plru_bits),
        .accessed_way   (accessed_way),
        .victim_way     (plru_victim),
        .plru_bits_next (plru_bits_next)
    );

    // victim: an invalid way if there is one, otherwise the PLRU way
    assign victim_way = any_inv ? inv_way : plru_victim;

endmodule

`default_nettype wire
