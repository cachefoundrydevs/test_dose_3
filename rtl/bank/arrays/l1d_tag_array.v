// ============================================================================
// l1d_tag_array.v  -  the TAGS of one bank (one small RAM per way), coded words (tag + ECC check bits)
// ============================================================================
// IN SIMPLE WORDS
//   A cache line is stored together with a "name tag": the upper address bits (50 bits) that say WHICH memory line
//   currently sits in that place. To find out whether an address is in the cache, the bank reads the tags of ALL
//   ways of the set and compares them (l1d_s1_lookup). This file holds those tags: WAYS (4) RAMs, each with SETS (64)
//   words of WORD_W bits - the RAM of way w is the generate copy g_way[w] of l1d_sram_tdp.
//   Reading one set therefore gives the 4 coded tags side by side in read_words_flat (way w at [w*WORD_W +: WORD_W]).
//   cache_v6 (ECC): the word is WORD_W = 57 bits = the 50-bit tag followed by its 7 SECDED check bits (l1d_ecc_enc on the
//   write path in l1d_bank_r, l1d_ecc_syn on the read path in l1d_bank_s1). With HAS_ECC = 0 WORD_W = 50.
//
//   The MESI state and the PLRU bits are NOT here; they are in flip-flops (l1d_meta_flops).
//
// WHEN IS IT READ AND WRITTEN?
//   read : launched in Stage 0 at the grant (set_index = the set of the granted operation), result in Stage 1
//          (the RAM has a registered output: 1 cycle latency).
//   write: in Stage 2, when a refill installs a line (write_word into the one way selected by way_write_enable), or when
//          the ECC scrub rewrites a corrected tag (cache_v6). A refill and a CPU request are never in S2 together.
//
// PORTS
//   read_enable                          read all ways this cycle
//   read_set_index                       set to read
//   way_write_enable                     one-hot way to write, zero for a read
//   write_set_index                      set to write
//   write_word                           coded tag to write
//   read_words_flat                      coded tag of every way, laid end to end
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

// cache_v2_bank_pipe (2026-11-03): TRUE DUAL PORT per way (l1d_sram_tdp). Port A reads all ways at the grant (read_*);
// port B writes in Stage 2 (write_*). A read and a write may share a cycle; the refill skip keeps reads of the set being
// installed away until the new tag is written (l1d_book_select).
module l1d_tag_array #(
    parameter SETS    = 64,                               // default 64: sets per bank
    parameter WAYS    = 4,                                // default 4: ways per set
    parameter WORD_W  = 57,                               // default 57: coded tag (50 tag + 7 ECC check bits)
    parameter INDEX_W = $clog2(SETS)                      // INDEX_W = 6 bits
) (
    input  wire                   clk,                    // clock: all registers change on its rising edge
    input  wire                   read_enable,            // read all ways this cycle (any grant)
    input  wire [INDEX_W-1:0]     read_set_index,         // set to read, INDEX_W = 6 bits
    input  wire [WAYS-1:0]        way_write_enable,       // one-hot way to write (Stage 2), zero = no write
    input  wire [INDEX_W-1:0]     write_set_index,        // set to write, INDEX_W = 6 bits
    input  wire [WORD_W-1:0]      write_word,             // coded tag to write, 57 bits
    output wire [WAYS*WORD_W-1:0] read_words_flat         // coded tag of every way, laid end to end, 4*57 = 228 bits
);
    genvar w;                                             // loop variable: way
    generate
        for (w = 0; w < WAYS; w = w + 1) begin : g_way
            l1d_sram_tdp #(
                .DEPTH  (SETS),
                .COL_W  (WORD_W),
                .NB_COL (1),
                .ADDR_W (INDEX_W)
            ) u_ram (
                .clk     (clk),
                .a_en    (read_enable),
                .a_we    (1'b0),
                .a_addr  (read_set_index),
                .a_wdata ({WORD_W{1'b0}}),
                .a_rdata (read_words_flat[w*WORD_W +: WORD_W]),
                .b_en    (way_write_enable[w]),
                .b_we    (way_write_enable[w]),
                .b_addr  (write_set_index),
                .b_wdata (write_word),
                .b_rdata ()
            );
        end
    endgenerate

`ifdef L1D_DEBUG
    always @(posedge clk)
        if (|way_write_enable)
            $display("[%0t] L1D TAG  %m write set=%0d ways=%b word=%h", $time, write_set_index, way_write_enable, write_word);
`endif
endmodule

`default_nettype wire
