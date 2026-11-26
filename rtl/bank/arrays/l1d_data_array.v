// ============================================================================
// l1d_data_array.v  -  the data array of one bank: ONE TRUE DUAL-PORT SRAM PER WAY, coded words (data + ECC check bits)
// ============================================================================
// cache_v2_bank_pipe (2026-11-03).
//   READ  (port A): at the grant (Stage G), set read_set_index, ALL ways at once. The hit way only selects among the
//                   outputs later (Stage S2). The outputs are registered once more by the bank at the end of Stage R.
//   WRITE (port B): in Stage 2, from flops: a refill line (all bytes) or the bytes of a store hit, one way, set
//                   write_set_index.
//   True dual port (user decision 2026-11-03): a read and a write happen in the same cycle, so a store hit no longer
//   blocks the grant of its S2 cycle. Same set and way on both ports in one cycle: the read gets the OLD line, which
//   the bank never uses (see l1d_sram_tdp.v and the wl_hit check in l1d_bank_r.v).
// cache_v6 (ECC): the word is WORD_W = 576 bits = the 512-bit line followed by its 64 check bits (8 granules x 8 bits, made
//   by l1d_ecc_enc in l1d_bank_r on the write path, checked by l1d_ecc_syn in l1d_bank_s1 on the read path). With HAS_ECC = 0
//   WORD_W = 512. The array itself does not know about ECC: it stores WORD_W bits per way and set.
//
// PORTS
//   read_enable        a read of all ways at read_set_index (any grant)
//   read_set_index     the set of the granted operation
//   write_enable       a write of way write_way at write_set_index (Stage 2, or the Stage 3 AMO write)
//   write_way          the way written (CB-3: debug print and check only)
//   write_way_onehot   CB-3: the way written, one-hot (= decode of write_way); it builds the per-way enables
//   write_set_index    the set of the write
//   (whole-word writes only: no byte write enables)
//   write_word         the coded line to write, WORD_W = 576 bits
//   read_words_flat    the coded lines of all ways: way w at [w*WORD_W +: WORD_W]
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

module l1d_data_array #(
    parameter SETS       = 64,                            // default 64: sets per bank
    parameter WAYS       = 4,                             // default 4: ways per set
    parameter WORD_W     = 576,                           // default 576: coded line (512 data + 64 ECC check bits)
    parameter INDEX_W    = $clog2(SETS),                  // INDEX_W = 6 bits
    parameter WAY_W      = (WAYS > 1) ? $clog2(WAYS) : 1  // WAY_W = 2 bits
) (
    input  wire                     clk,                  // clock: all registers change on its rising edge
    input  wire                     read_enable,          // read all ways this cycle
    input  wire [INDEX_W-1:0]       read_set_index,       // INDEX_W = 6 bits
    input  wire                     write_enable,         // write one way this cycle
    input  wire [WAY_W-1:0]         write_way,            // the way written, 2 bits (CB-3: debug / check only)
    input  wire [WAYS-1:0]          write_way_onehot,     // CB-3: the way written, one-hot (= decode of write_way), WAYS = 4 bits
    input  wire [INDEX_W-1:0]       write_set_index,      // INDEX_W = 6 bits
    input  wire [WORD_W-1:0]        write_word,           // the coded line, 576 bits
    output wire [WAYS*WORD_W-1:0]   read_words_flat       // WAYS * 576 = 2304 bits
);
    genvar w;                                             // loop variable: way
    generate
        for (w = 0; w < WAYS; w = w + 1) begin : g_way
            wire this_wr;                                 // this way is written
            // TIMING OPT (CB-3, 2026-11-26), part c: one-hot bit, no compare of the late binary way.
            // Same value: write_way_onehot = decode(write_way). Example: way 1 written -> 0010 -> only g_way[1].
            assign this_wr = write_enable && write_way_onehot[w];
            l1d_sram_tdp #(
                .DEPTH  (SETS),
                .COL_W  (WORD_W),          // ONE write column: whole-word writes (no byte enables)
                .NB_COL (1),
                .ADDR_W (INDEX_W)
            ) u_ram (
                .clk     (clk),
                .a_en    (read_enable),
                .a_we    (1'b0),
                .a_addr  (read_set_index),
                .a_wdata ({WORD_W{1'b0}}),
                .a_rdata (read_words_flat[w*WORD_W +: WORD_W]),
                .b_en    (this_wr),
                .b_we    (this_wr),                // whole word or nothing
                .b_addr  (write_set_index),
                .b_wdata (write_word),
                .b_rdata ()
            );
        end
    endgenerate

`ifndef SYNTHESIS
    // CB-3 always-on check: the one-hot way equals the decode of the binary way
    always @(posedge clk) begin
        if (write_enable && (write_way_onehot != ({{(WAYS-1){1'b0}}, 1'b1} << write_way)))
            $display("[%0t] L1D DATA %m ERROR CB-3 one-hot write way %b differs from way %0d", $time, write_way_onehot, write_way);
    end
`endif
`ifdef L1D_DEBUG
    always @(posedge clk) begin
        if (write_enable)
            $display("[%0t] L1D DATA %m word write set/way=%0d/%0d data=%h", $time, write_set_index, write_way, write_word);
    end
`endif
endmodule

`default_nettype wire
