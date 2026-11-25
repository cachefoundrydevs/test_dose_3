// ============================================================================
// l1d_arb_rr_flat.v  -  round-robin arbiter with a flat, parallel grant (same behaviour as l1d_arb_rr)
// ============================================================================
// IN SIMPLE WORDS
//   Several units want ONE resource; one of them gets it per cycle, in fair turns (round robin). This is the same
//   arbiter as l1d_arb_rr (same ports, same register mask_r, same grant in every cycle), only the grant is built
//   differently: every requester asks "does anybody beat me?" for all the others AT THE SAME TIME, instead of
//   first searching the requests after the last winner, then all requests, then choosing between the two answers.
//   Picture a queue at a counter where the last customer served stands at the back: requester j wins when it asks
//   and nobody who asks stands in front of it.
//
// STAGE / CYCLE   : combinational grant in the request cycle; mask_r is the only register (moves on a taken grant).
// INSTANTIATED BY : l1d_l2_port, instances u_read_rr and u_write_rr (2 copies, N = NUM_BANKS = 4).
//
// PARAMETERS (name, default, unit, meaning)
//   N      4  requesters  number of requesters
//   IDX_W  2  bits        bits of a requester number, $clog2(N) (1 when N = 1)
//
// PORTS (direction, width at the defaults, level, meaning)
//   clk           in  1  rising edge   clock
//   rst           in  1  high          synchronous reset (mask_r = 0: requester 0 goes first)
//   request_bits  in  4  high          one bit per requester that wants the resource
//   grant_taken   in  1  high          the current grant was used this cycle, so the turn moves on
//   grant_onehot  out 4  high, comb.   one-hot grant (zero when nobody asks)
//   grant_index   out 2  comb.         number of the granted requester (0 when nobody asks)
//   any_request   out 1  high, comb.   at least one requester asks
//
// HOW THE GRANT IS FORMED (TIMING OPT L2-5, exact for EVERY value of mask_r, not only the reachable thermometer codes)
//   mask_r[i] = 1 means requester i comes after the last winner in this round (it is served first).
//   Requester i, if it asks, beats requester j when
//     i < j : mask_r[i] | ~mask_r[j]   (i is in the first round, or j is not: then lower wins)
//     i > j : mask_r[i] & ~mask_r[j]   (only a first-round i beats a second-round j)
//   grant_onehot[j] = request_bits[j] & ~|(request_bits[i] & beats(i, j) over every i != j)
//   The beat terms come from the register only (one gate), so a request passes 2 gates (AND-NOR, AND) to the grant.
//   Old form (l1d_arb_rr): masked = request & mask_r -> prefix OR -> use_masked (OR of all) -> 2:1 mux -> grant,
//   93 to 150 ps in the l1d_l2_port runs; this form: about 2 to 3 gates.
//   Next mask: bit j = some requester below j wins = |grant_onehot[j-1:0] (bit 0 = 0); this equals the old
//   use_masked ? |masked[j-1:0] : |request_bits[j-1:0] for every mask_r and every request pattern (also with no
//   request, where both are 0). Checked exhaustively for N = 2, 3, 4, 5, 8 (model_arb_it5.py).
//
// WORKED EXAMPLE (N = 4)
//   mask_r = 1100 (last winner was 1), request_bits = 0101 (requesters 0 and 2).
//   j = 2: i = 0 < 2: mask_r[0] | ~mask_r[2] = 0 | 0 = 0, so 0 does not beat 2; nobody above asks -> grant 2.
//   j = 0: i = 2 > 0: mask_r[2] & ~mask_r[0] = 1 -> 2 beats 0, no grant. grant_onehot = 0100, grant_index = 2.
//   Taken: mask_next = 1000 (bit 3 = |grant[2:0] = 1, bit 2 = |grant[1:0] = 0 ...) -> the round goes on at 3.
//
// ASSUMPTIONS (V-9.2): none (grant correct for every request pattern and every register value).
// ============================================================================

`ifndef L1D_ARB_RR_FLAT_V
`define L1D_ARB_RR_FLAT_V

`default_nettype none

module l1d_arb_rr_flat #(
    parameter N     = 4,                                  // default 4: number of requesters
    parameter IDX_W = (N > 1) ? $clog2(N) : 1             // default 2: bits of a requester number
) (
    // ---- clock and reset ----
    input  wire             clk,                          // clock: all registers change on its rising edge
    input  wire             rst,                          // synchronous reset, active high (mask_r = 0)
    // ---- requests in ----
    input  wire [N-1:0]     request_bits,                 // N = 4 bits, high: requester wants the resource (comb.)
    input  wire             grant_taken,                  // high: the current grant was used, the turn moves on
    // ---- grant out (combinational) ----
    output wire [N-1:0]     grant_onehot,                 // N = 4 bits: one-hot grant (zero when nobody asks)
    output wire [IDX_W-1:0] grant_index,                  // IDX_W = 2 bits: number of the granted requester
    output wire             any_request                   // high: at least one requester asks
);

    reg  [N-1:0]   mask_r;              // N = 4 bits: requesters after the last winner (same register as l1d_arb_rr)
    wire [N*N-1:0] beat_terms;          // N*N = 16 bits: bit j*N+i = requester i asks and beats requester j
    wire [N-1:0]   beaten;              // N = 4 bits: some asking requester beats requester j
    wire [N-1:0]   mask_after_grant;    // N = 4 bits: next mask_r, bit j = the winner is below j
    genvar         winner_idx;          // requester whose grant is formed
    genvar         rival_idx;           // requester compared with it
    genvar         index_bit;           // bit of the grant number

    generate
        for (winner_idx = 0; winner_idx < N; winner_idx = winner_idx + 1) begin : g_winner
            for (rival_idx = 0; rival_idx < N; rival_idx = rival_idx + 1) begin : g_rival
                if (rival_idx < winner_idx) begin : g_lower
                    // a lower rival wins when it is in the first round or the winner is not
                    assign beat_terms[winner_idx*N + rival_idx] = request_bits[rival_idx]
                        & (mask_r[rival_idx] | ~mask_r[winner_idx]);
                end else if (rival_idx > winner_idx) begin : g_upper
                    // a higher rival wins only from the first round against a second-round requester
                    assign beat_terms[winner_idx*N + rival_idx] = request_bits[rival_idx]
                        & (mask_r[rival_idx] & ~mask_r[winner_idx]);
                end else begin : g_self
                    assign beat_terms[winner_idx*N + rival_idx] = 1'b0;      // a requester never beats itself
                end
            end
            assign beaten[winner_idx]       = |beat_terms[winner_idx*N +: N];              // N = 4 terms, one OR
            assign grant_onehot[winner_idx] = request_bits[winner_idx] & ~beaten[winner_idx];   // asks, nobody beats it
            if (winner_idx == 0) begin : g_mask_first
                assign mask_after_grant[winner_idx] = 1'b0;                                 // nothing below requester 0
            end else begin : g_mask_rest
                assign mask_after_grant[winner_idx] = |grant_onehot[winner_idx-1:0];        // the winner is below j
            end
        end
        // grant number: OR of the one-hot bits whose number has this bit set
        for (index_bit = 0; index_bit < IDX_W; index_bit = index_bit + 1) begin : g_index
            wire [N-1:0] index_terms;                     // N = 4 bits: grant bits whose number has this bit set
            for (winner_idx = 0; winner_idx < N; winner_idx = winner_idx + 1) begin : g_term
                if ((winner_idx >> index_bit) & 1) begin : g_set
                    assign index_terms[winner_idx] = grant_onehot[winner_idx];             // number has the bit
                end else begin : g_clear
                    assign index_terms[winner_idx] = 1'b0;                                  // number lacks the bit
                end
            end
            assign grant_index[index_bit] = |index_terms;                                   // OR of the matching grants
        end
    endgenerate

    assign any_request = |request_bits;                   // at least one requester asks

    // turn register: moves only on a taken grant ("a grant that is not taken keeps its turn"), as l1d_arb_rr
    always @(posedge clk) begin : mask_proc
        if (rst)
            mask_r <= {N{1'b0}};                          // requester 0 first after reset
        else if (grant_taken && any_request)
            mask_r <= mask_after_grant;                   // requesters above the winner
    end

endmodule

`default_nettype wire

`endif // L1D_ARB_RR_FLAT_V
