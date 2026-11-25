`ifndef L1D_ARB_RR_FLAT_V
`define L1D_ARB_RR_FLAT_V
`default_nettype none
module l1d_arb_rr_flat #(
    parameter N     = 4,
    parameter IDX_W = (N > 1) ? $clog2(N) : 1
) (
    input  wire             clk,
    input  wire             rst,
    input  wire [N-1:0]     request_bits,
    input  wire             grant_taken,
    output wire [N-1:0]     grant_onehot,
    output wire [IDX_W-1:0] grant_index,
    output wire             any_request,
    output wire [N-1:0]     not_beaten
);
    reg  [N-1:0]   mask_r;
    wire [N*N-1:0] beat_terms;
    wire [N-1:0]   beaten;
    wire [N-1:0]   mask_after_grant;
    genvar         winner_idx;
    genvar         rival_idx;
    genvar         index_bit;
    generate
        for (winner_idx = 0; winner_idx < N; winner_idx = winner_idx + 1) begin : g_winner
            for (rival_idx = 0; rival_idx < N; rival_idx = rival_idx + 1) begin : g_rival
                if (rival_idx < winner_idx) begin : g_lower
                    assign beat_terms[winner_idx*N + rival_idx] = request_bits[rival_idx]
                        & (mask_r[rival_idx] | ~mask_r[winner_idx]);
                end else if (rival_idx > winner_idx) begin : g_upper
                    assign beat_terms[winner_idx*N + rival_idx] = request_bits[rival_idx]
                        & (mask_r[rival_idx] & ~mask_r[winner_idx]);
                end else begin : g_self
                    assign beat_terms[winner_idx*N + rival_idx] = 1'b0;
                end
            end
            assign beaten[winner_idx]       = |beat_terms[winner_idx*N +: N];
            assign not_beaten[winner_idx]   = ~beaten[winner_idx];
            assign grant_onehot[winner_idx] = request_bits[winner_idx] & not_beaten[winner_idx];
            if (winner_idx == 0) begin : g_mask_first
                assign mask_after_grant[winner_idx] = 1'b0;
            end else begin : g_mask_rest
                assign mask_after_grant[winner_idx] = |grant_onehot[winner_idx-1:0];
            end
        end
        for (index_bit = 0; index_bit < IDX_W; index_bit = index_bit + 1) begin : g_index
            wire [N-1:0] index_terms;
            for (winner_idx = 0; winner_idx < N; winner_idx = winner_idx + 1) begin : g_term
                if ((winner_idx >> index_bit) & 1) begin : g_set
                    assign index_terms[winner_idx] = grant_onehot[winner_idx];
                end else begin : g_clear
                    assign index_terms[winner_idx] = 1'b0;
                end
            end
            assign grant_index[index_bit] = |index_terms;
        end
    endgenerate
    assign any_request = |request_bits;
    always @(posedge clk) begin : mask_proc
        if (rst)
            mask_r <= {N{1'b0}};
        else if (grant_taken && any_request)
            mask_r <= mask_after_grant;
    end
endmodule
`default_nettype wire
`endif
