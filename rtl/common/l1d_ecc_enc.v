// ============================================================================
// l1d_ecc_enc.v  -  SECDED encoder: the check bits of one data word (Hamming + overall parity)
// ============================================================================
// IN SIMPLE WORDS  (cache_v6, PLAN_v6.md section 5)
//   A stored word can lose a bit (a soft error). SECDED = Single Error Correction, Double Error Detection: HAM_W Hamming
//   check bits locate ONE flipped bit so it can be put back, and one more parity bit over everything tells a single error
//   (odd parity) from a double error (even parity, syndrome not zero). This module makes those HAM_W + 1 check bits for a
//   data word on its way INTO the array. l1d_ecc_syn and l1d_ecc_fix read them on the way out.
//
//   HOW THE HAMMING BITS ARE BUILT. Every bit of the code word has a position number 1, 2, 3 ... Check bit i sits at position
//   2**i; the data bits fill the other positions in order (3, 5, 6, 7, 9, ...). Check bit i = XOR of all data bits whose
//   position number has bit i set. l1d_ecc_syn recomputes them: the XOR of stored and recomputed check bits (the syndrome)
//   is the position number of the flipped bit, or 0.
//   Example (DATA_W = 4, HAM_W = 3): data d0..d3 at positions 3, 5, 6, 7. c0 = d0^d1^d3 (positions 3, 5, 7 have bit 0 set),
//   c1 = d0^d2^d3 (3, 6, 7), c2 = d1^d2^d3 (5, 6, 7). Flip d2 (position 6 = 110b): syndrome = 110b = 6.
//
// STAGE / CYCLE
//   Combinational: used on the write data path of the data array (S2 store / refill, S3 AMO) and the tag array (S2 refill),
//   8 copies per line (one per 64-bit granule) and one per tag write. A 64-input XOR tree is 6 XOR2 levels (about 9 gate
//   equivalents; estimate, PLAN_v6 O-6).
//
// INSTANTIATED BY
//   l1d_bank_r (data write path, GRANULES = 8 copies per bank; tag write path, 1 copy).
//
// PARAMETERS
//   DATA_W   default 64: data bits of the word
//   HAM_W    default 7: Hamming check bits (2**HAM_W must be >= DATA_W + HAM_W + 1; 64 -> 7, 50 -> 6)
//
// PORTS
//   data        in   DATA_W      the data word
//   check       out  HAM_W + 1   {overall parity, Hamming bits}: the check bits stored beside the word
// ============================================================================
`ifndef L1D_ECC_ENC_V
`define L1D_ECC_ENC_V
`default_nettype none

module l1d_ecc_enc #(
    parameter DATA_W = 64,                                 // default 64: data bits of the word
    parameter HAM_W  = 7                                   // default 7: Hamming check bits
) (
    input  wire [DATA_W-1:0] data,                         // the data word, DATA_W = 64 bits
    output wire [HAM_W:0]    check                         // {overall parity, Hamming bits}, HAM_W + 1 = 8 bits
);
    // ---- position number of data bit d in the code word: the d-th position that is not a power of two (constant function) ----
    function integer data_pos;
        input integer bit_num;                             // data bit number 0 .. DATA_W-1
        integer pos;                                       // candidate position
        integer seen;                                      // data positions passed
        integer is_pow2;                                   // candidate is a power of two (a check bit position)
        integer k;                                         // loop variable: bit of the candidate
        begin
            data_pos = 0;
            seen     = 0;
            for (pos = 1; pos <= DATA_W + HAM_W; pos = pos + 1) begin
                is_pow2 = 0;
                for (k = 0; k <= HAM_W; k = k + 1)
                    if (pos == (1 << k)) is_pow2 = 1;
                if (!is_pow2) begin
                    if (seen == bit_num) data_pos = pos;
                    seen = seen + 1;
                end
            end
        end
    endfunction
    // ---- the mask of Hamming bit i: data bits whose position has bit i set (constant function) ----
    function [DATA_W-1:0] ham_mask;
        input integer ham_bit;                             // Hamming bit number 0 .. HAM_W-1
        integer d;                                         // loop variable: data bit
        begin
            for (d = 0; d < DATA_W; d = d + 1)
                ham_mask[d] = ((data_pos(d) >> ham_bit) & 1) ? 1'b1 : 1'b0;
        end
    endfunction

    // ---- the Hamming bits: one XOR tree per bit over the masked data ----
    wire [HAM_W-1:0] ham;                                  // HAM_W = 7 bits
    genvar hb;                                             // loop variable: Hamming bit
    generate
        for (hb = 0; hb < HAM_W; hb = hb + 1) begin : g_ham
            localparam [DATA_W-1:0] MASK = ham_mask(hb);   // the data bits that take part in Hamming bit hb
            assign ham[hb] = ^(data & MASK);
        end
    endgenerate
    // ---- the overall parity: even parity over data and Hamming bits ----
    assign check = {(^data) ^ (^ham), ham};

endmodule

`default_nettype wire
`endif // L1D_ECC_ENC_V
