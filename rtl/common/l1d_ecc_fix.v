// ============================================================================
// l1d_ecc_fix.v  -  SECDED correction: put the flipped data bit back, given the syndrome
// ============================================================================
// IN SIMPLE WORDS  (cache_v6, PLAN_v6.md section 5)
//   l1d_ecc_syn said "one bit flipped, at code word position = syndrome". This module turns that position into a one-hot
//   flip vector over the DATA bits (a position that is a power of two names a check bit: nothing to flip in the data) and
//   XORs it into the word. The caller applies it only on a correctable error (fix_enable).
//
// STAGE / CYCLE
//   Combinational, on the S2 write path of the ECC scrub (l1d_bank_s2): the corrected line is written through the store
//   write port. Depth: a HAM_W-bit decode per data bit (AND of 7 literals = 2 to 3 levels) and one XOR.
//
// INSTANTIATED BY
//   l1d_bank_s2 (GRANULES = 8 copies for the hit line; 1 copy for the tag).
//
// PARAMETERS
//   DATA_W   default 64: data bits of the word;   HAM_W   default 7: Hamming check bits
//
// PORTS
//   data        in   DATA_W   the stored data
//   syndrome    in   HAM_W    position of the flipped bit (from l1d_ecc_syn)
//   fix_enable  in   1        1 = apply the correction (a correctable error)
//   fixed       out  DATA_W   the corrected data (= data when fix_enable = 0 or the error is in a check bit)
// ============================================================================
`ifndef L1D_ECC_FIX_V
`define L1D_ECC_FIX_V
`default_nettype none

module l1d_ecc_fix #(
    parameter DATA_W = 64,                                 // default 64: data bits of the word
    parameter HAM_W  = 7                                   // default 7: Hamming check bits
) (
    input  wire [DATA_W-1:0] data,                         // the stored data, DATA_W = 64 bits
    input  wire [HAM_W-1:0]  syndrome,                     // position of the flipped bit, HAM_W = 7 bits
    input  wire              fix_enable,                   // apply the correction
    output wire [DATA_W-1:0] fixed                         // the corrected data, 64 bits
);
    // ---- position number of data bit d in the code word (the same constant function as l1d_ecc_enc) ----
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

    // tm_opt iter 3 (new signals): the syndrome is decoded in two halves in parallel (LO_W low bits, HAM_W - LO_W high bits) into
    // one-hot vectors; each data bit then needs one AND of one bit of each (same value as fix_enable & (syndrome == POS)).
    localparam LO_W = HAM_W / 2;                           // low half of the syndrome, LO_W = 3 bits
    wire [(1 << LO_W)-1:0]           lo_hot;               // low half decoded, 2**LO_W = 8 bits
    wire [(1 << (HAM_W - LO_W))-1:0] hi_hot;               // high half decoded, 2**(HAM_W-LO_W) = 16 bits
    genvar lv, hv;                                         // loop variables: decoded values
    generate
        for (lv = 0; lv < (1 << LO_W); lv = lv + 1) begin : g_lo
            assign lo_hot[lv] = (syndrome[LO_W-1:0] == lv);
        end
        for (hv = 0; hv < (1 << (HAM_W - LO_W)); hv = hv + 1) begin : g_hi
            assign hi_hot[hv] = (syndrome[HAM_W-1:LO_W] == hv);
        end
    endgenerate
    wire [DATA_W-1:0] flip;                                // one-hot (or zero): the data bit to put back, 64 bits
    genvar d;                                              // loop variable: data bit
    generate
        for (d = 0; d < DATA_W; d = d + 1) begin : g_flip
            localparam [HAM_W-1:0] POS = data_pos(d);      // the code word position of data bit d
            assign flip[d] = fix_enable & lo_hot[POS[LO_W-1:0]] & hi_hot[POS[HAM_W-1:LO_W]];
        end
    endgenerate
    assign fixed = data ^ flip;

endmodule

`default_nettype wire
`endif // L1D_ECC_FIX_V
