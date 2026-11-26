// ============================================================================
// l1d_ecc_syn.v  -  SECDED syndrome: did this stored word lose a bit, and which one?
// ============================================================================
// IN SIMPLE WORDS  (cache_v6, PLAN_v6.md section 5)
//   The word comes out of the array with its check bits (made by l1d_ecc_enc). This module recomputes the Hamming bits
//   and XORs them with the stored ones: the SYNDROME. It also recomputes the overall parity.
//     syndrome = 0, parity even  -> no error
//     syndrome != 0, parity odd  -> one bit flipped, at position = syndrome (correctable; l1d_ecc_fix puts it back)
//     syndrome = 0, parity odd   -> the overall parity bit itself flipped (correctable, data unchanged)
//     syndrome != 0, parity even -> two bits flipped (uncorrectable)
//
// STAGE / CYCLE
//   Combinational, used in S1 on the registered SRAM outputs (l1d_bank_s1): GRANULES x WAYS = 32 copies for the data and
//   WAYS = 4 copies for the tags per bank. A 72-input XOR tree is 7 XOR2 levels (about 10 gate equivalents; estimate over the
//   8-level budget: PLAN_v6 O-6 names the split option).
//
// INSTANTIATED BY
//   l1d_bank_s1 (data: 32 copies; tags: 4 copies).
//
// PARAMETERS
//   DATA_W   default 64: data bits of the word;   HAM_W   default 7: Hamming check bits
//
// PORTS
//   data            in   DATA_W     the stored data bits
//   check           in   HAM_W + 1  the stored check bits {overall parity, Hamming bits}
//   syndrome        out  HAM_W      position of a single flipped bit (0 = none)
//   parity_odd      out  1          the whole word has odd parity (an odd number of bits flipped)
//   correctable     out  1          one bit flipped (data or check bit): the word can be fixed
//   uncorrectable   out  1          two bits flipped: the word cannot be fixed
// ============================================================================
`ifndef L1D_ECC_SYN_V
`define L1D_ECC_SYN_V
`default_nettype none

module l1d_ecc_syn #(
    parameter DATA_W = 64,                                 // default 64: data bits of the word
    parameter HAM_W  = 7                                   // default 7: Hamming check bits
) (
    input  wire [DATA_W-1:0] data,                         // the stored data, DATA_W = 64 bits
    input  wire [HAM_W:0]    check,                        // the stored check bits, HAM_W + 1 = 8 bits
    output wire [HAM_W-1:0]  syndrome,                     // position of the flipped bit, HAM_W = 7 bits (0 = none)
    output wire              parity_odd,                   // an odd number of bits flipped
    output wire              correctable,                  // one bit flipped
    output wire              uncorrectable                 // two bits flipped
);
    wire [HAM_W:0] recomputed;                             // the check bits of the stored data, 8 bits
    l1d_ecc_enc #(.DATA_W(DATA_W), .HAM_W(HAM_W)) u_enc (
        .data  (data),
        .check (recomputed)
    );
    wire syndrome_nonzero;                                 // some Hamming bit disagrees
    assign syndrome         = recomputed[HAM_W-1:0] ^ check[HAM_W-1:0];
    assign syndrome_nonzero = |syndrome;
    // the overall parity of the stored word: the recomputed parity (over data and recomputed Hamming bits) against the stored one,
    // plus the Hamming bits' own disagreement (a flipped stored Hamming bit shows in the syndrome and in the parity)
    assign parity_odd    = recomputed[HAM_W] ^ check[HAM_W] ^ (^syndrome);
    assign correctable   = parity_odd;                     // one flipped bit (data, Hamming bit or the parity bit itself)
    assign uncorrectable = syndrome_nonzero & ~parity_odd; // two flipped bits

endmodule

`default_nettype wire
`endif // L1D_ECC_SYN_V
