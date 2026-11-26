// ============================================================================
// l1d_bank_r.v  -  bank stage R: the tag and data SRAMs (coded words), the ECC encoders of the write path, the R check of a
//                  fresh request, the R -> S1 registers
// ============================================================================
// IN SIMPLE WORDS
//   The granted operation's set was sent to the SRAMs in stage G (l1d_bank_s0). In stage R the SRAMs give back the coded
//   tags of all 4 ways (l1d_tag_array, 57 bits each: 50 tag + 7 check bits) and the coded lines of all 4 ways
//   (l1d_data_array, 576 bits each: 512 data + 64 check bits). Their outputs go straight into flip-flops at the end of R
//   (tag_words_q, s1_words): no logic between the SRAM and the flop, so the slow SRAM read has the whole cycle.
//   The SRAM WRITE port is also here: Stage 2 writes a refill line, a store's merged line or an ECC-corrected line (cache_v6),
//   Stage 3 an AMO's line; every written line gets fresh check bits from 8 encoders (l1d_ecc_enc), and a written tag gets
//   its 7 check bits.
//   THE R CHECK (cache_v6, PLAN_v6 3.5): a request that issued FRESH (in its arrival cycle, on trust) is cancelled here when
//   (1) its tracker entry has a dependency bit set (an older request of its set where one of the two writes), or (2) a store
//   or AMO wrote its set at the very SRAM edge of its read (read-first: it got the old line), or (3) an AMO writes its set
//   now in S3. It runs on as a ghost (s1_kill) that changes nothing; at S2 its entry goes back to READY. A booked (non-fresh)
//   request is never cancelled: its dependencies were 0 when it was booked.
//
//   g_set (stage G) --> tag SRAMs (4 ways) --> tag_words_q (register, to S1)
//                   --> data SRAMs (4 ways) --> s1_words    (register, to S1)
//   r_* registers ------------------------------------> s1_* registers (to S1) + s1_kill (R check)
//   write port: refill / store / ECC fix (S2) or AMO (S3) line --> encoders --> data SRAM;  refill / ECC fix tag (S2) --> encoder --> tag SRAM
//
// STAGE / CYCLE
//   Cycle c0+1 of a hit. Inputs: the grant and set (combinational from stage G), the r_* registers, the dependency bits of the
//   entry in R (flops of the tracker), the write data of S2 / S3 (combinational in their cycle). Outputs: only registers.
//   Estimated depth of the R check: deps OR (2) + 6-bit set compares (3) + OR (1) + AND (1) = 7 levels into s1_kill.
//
// INSTANTIATED BY
//   l1d_bank, instance u_r (one per bank).
//
// PARAMETERS
//   `L1D_PARAMS   all cache parameters (l1d_defines.vh); used here: SETS (64), WAYS (4), LINE_BYTES (64), HAS_ECC and the
//                 widths of l1d_fields.vh (TAG_W 50, TWORD_W 57, LINE_W 512, DWORD_W 576, INDEX_W 6, WAY_W 2, REQ_W 148, ...)
//   BANK_ID       default 0: number of this bank (debug messages only)
//
// PORTS (name, direction, width at the defaults, meaning)
//   clk, rst                     in   1      clock; synchronous reset, active high
//   any_gnt, g_set               in   1/6    stage G reads the SRAMs; the set they read
//   r_valid ... r_sid            in   regs   the Stage R registers (from l1d_bank_s0); r_seat = tracker entry, r_fast = fresh issue
//   r_deps                       in   8      the dependency bits of the entry in R (tracker flops): any set = cancel a fresh op
//   s2_refill                    in   1      S2 installs a refill: write its line and its tag in way s2_way, set of s2_line
//   s2_st_wr                     in   1      S2 writes a line in way s2_way: a store's merged line, or an ECC-corrected line
//   s2_tag_fix_wr                in   1      S2 writes an ECC-corrected tag in way s2_way (cache_v6)
//   s2_way, s2_line              in   2/58   way and line address of the S2 write
//   s2_way_onehot                in   4      (CB-3) the same way, one-hot (from u_s2; SRAM write enables)
//   s2_write_line                in   512    the line S2 writes (refill line, merged store line or corrected line)
//   s2_write_tag                 in   50     the tag S2 writes (the refill's tag, or the corrected tag)
//   s3_amo_v, s3_way, s3_line    in   1/2/58 S3 AMO write: way and line
//   s3_merged                    in   512    the AMO's merged line
//   s1_valid ... s1_sid, s1_kill out  regs   the S1 registers (fields of the operation now in S1); s1_kill = cancelled
//   tag_words_q                  out  228    the coded tags of all ways (registered SRAM output), WAYS*TWORD_W = 4*57 bits
//   s1_words                     out  2304   the coded lines of all ways (registered SRAM output), WAYS*DWORD_W = 4*576 bits
//
// WORKED EXAMPLE (a fresh load, the same set was written by a store in S2 one cycle earlier)
//   c0  : the store is in S2 and writes its line (s2_st_wr = 1); the load issues fresh and its SRAM read starts.
//   c0+1: wl_valid_q = 1 and wl_set_q = the store's set = r_set -> wl_hit = 1 -> r_kill = 1: the read data may be the old
//         line, so s1_kill = 1 at the edge. In S2 the load gives act_code = ACT_NONE with s2_kill: its entry goes back to
//         READY, it is booked again and then reads the new line.
// ============================================================================
`ifndef L1D_BANK_R_V
`define L1D_BANK_R_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_bank_r #(
    `L1D_PARAMS,                                       // all cache parameters (l1d_defines.vh)
    parameter BANK_ID = 0                              // default 0: number of this bank (debug messages only)
) (
    clk, rst,
    any_gnt, g_set,
    r_valid, r_kind, r_seat, r_req, r_fast, r_mid, r_line, r_mway, r_mop, r_sinv, r_sid,
    r_deps,
    s2_refill, s2_st_wr, s2_tag_fix_wr, s2_way, s2_way_onehot, s2_line, s2_write_line, s2_write_tag,
    s3_amo_v, s3_way, s3_line, s3_merged,
    s1_valid, s1_kind, s1_seat, s1_req, s1_mid, s1_line, s1_mway, s1_mop, s1_sinv, s1_sid, s1_kill,
    tag_words_q, s1_words
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // ======================================================================
    // Ports
    // ======================================================================
    input  wire                        clk;                      // clock: all registers change on its rising edge
    input  wire                        rst;                      // synchronous reset, active high
    // ---- the grant of stage G ----
    input  wire                        any_gnt;                  // the SRAMs read this cycle
    input  wire [INDEX_W-1:0]          g_set;                    // the set they read, INDEX_W = 6 bits
    // ---- the Stage R registers (from l1d_bank_s0) ----
    input  wire                        r_valid;                  // Stage R holds an operation
    input  wire [1:0]                  r_kind;                   // its kind (K_LSU, K_REFILL, K_MAINT, K_SNOOP)
    input  wire [SEAT_W-1:0]           r_seat;                   // its tracker entry, SEAT_W = 3 bits
    input  wire [REQ_W-1:0]            r_req;                    // its request packet, REQ_W = 148 bits
    input  wire                        r_fast;                   // the CPU request issued fresh (R check applies)
    input  wire [MSHR_ID_W-1:0]        r_mid;                    // MSHR entry of a refill token, 3 bits
    input  wire [LINE_ADDR_W-1:0]      r_line;                   // its line address, LINE_ADDR_W = 58 bits
    input  wire [WAY_W-1:0]            r_mway;                   // way of a flush/clean token, WAY_W = 2 bits
    input  wire                        r_mop;                    // operation of a flush/clean token
    input  wire                        r_sinv;                   // snoop: 1 = invalidate
    input  wire [SNP_ID_W-1:0]         r_sid;                    // snoop buffer entry, 1 bit
    input  wire [TRK_ENTRIES-1:0]      r_deps;                   // dependency bits of the entry in R, TRK_ENTRIES = 8 bits
    // ---- the write port (S2 and S3) ----
    input  wire                        s2_refill;                // S2 installs a refill (line + tag)
    input  wire                        s2_st_wr;                 // S2 writes a line (store merge or ECC fix)
    input  wire                        s2_tag_fix_wr;            // S2 writes a corrected tag (ECC fix)
    input  wire [WAY_W-1:0]            s2_way;                   // way of the S2 write, 2 bits (CB-3: debug / check only)
    input  wire [WAYS-1:0]             s2_way_onehot;            // CB-3: the same way, one-hot (= decode of s2_way), WAYS = 4 bits
    input  wire [LINE_ADDR_W-1:0]      s2_line;                  // line address of the operation in S2, 58 bits
    input  wire [LINE_W-1:0]           s2_write_line;            // the line S2 writes, LINE_W = 512 bits
    input  wire [TAG_W-1:0]            s2_write_tag;             // the tag S2 writes, TAG_W = 50 bits
    input  wire                        s3_amo_v;                 // S3 holds an AMO hit: it writes its line
    input  wire [WAY_W-1:0]            s3_way;                   // its way, 2 bits
    input  wire [LINE_ADDR_W-1:0]      s3_line;                  // its line address, 58 bits
    input  wire [LINE_W-1:0]           s3_merged;                // the old line with the AMO result in place, 512 bits
    // ---- the S1 registers (R -> S1) ----
    output reg                         s1_valid;                 // stage S1 holds an operation
    output reg  [1:0]                  s1_kind;                  // its kind
    output reg  [SEAT_W-1:0]           s1_seat;                  // its tracker entry, 3 bits
    output reg  [REQ_W-1:0]            s1_req;                   // its request packet, 148 bits
    output reg  [MSHR_ID_W-1:0]        s1_mid;                   // MSHR entry of the refill token, 3 bits
    output reg  [LINE_ADDR_W-1:0]      s1_line;                  // its line address, 58 bits
    output reg  [WAY_W-1:0]            s1_mway;                  // way of the flush/clean token, 2 bits
    output reg                         s1_mop;                   // operation of the flush/clean token (0 clean, 1 flush)
    output reg                         s1_sinv;                  // snoop: 1 = invalidate
    output reg  [SNP_ID_W-1:0]         s1_sid;                   // snoop buffer entry, 1 bit
    output reg                         s1_kill;                  // a fresh request that was cancelled in R (ghost)
    output reg  [WAYS*TWORD_W-1:0]     tag_words_q;              // the coded tags of all ways, WAYS*TWORD_W = 228 bits
    output reg  [WAYS*DWORD_W-1:0]     s1_words;                 // the coded lines of all ways, WAYS*DWORD_W = 2304 bits

    // set numbers of the S2 and S3 writes and of the op in R
    wire [INDEX_W-1:0] s2_set;                                   // set of the S2 write, 6 bits
    wire [INDEX_W-1:0] s3_set;                                   // set of the S3 AMO write, 6 bits
    wire [INDEX_W-1:0] r_set;                                    // set of the op in R, 6 bits
    assign s2_set = s2_line[LA_SET_LSB +: INDEX_W];
    assign s3_set = s3_line[LA_SET_LSB +: INDEX_W];
    assign r_set  = r_line[LA_SET_LSB +: INDEX_W];

    // ======================================================================
    // Write path: the line (S3 AMO first, else S2) and its check bits; the tag and its check bits
    // ======================================================================
    wire                   data_wr;                              // some line is written this cycle
    wire [WAY_W-1:0]       data_wr_way;                          // its way, 2 bits (CB-3: debug / check only)
    wire [WAYS-1:0]        data_wr_way_onehot;                   // CB-3: its way, one-hot (= decode of data_wr_way), 4 bits
    wire [WAYS-1:0]        s3_way_onehot;                        // CB-3: decode of the s3_way flop, 4 bits
    wire [INDEX_W-1:0]     data_wr_set;                          // its set, 6 bits
    wire [LINE_W-1:0]      data_wr_line;                         // the line, 512 bits
    wire [LINE_ECC_W-1:0]  data_wr_check;                        // its check bits, 8 granules x 8 = 64 bits (0 wide without ECC)
    wire [DWORD_W-1:0]     data_wr_word;                         // the coded line, 576 bits
    assign data_wr      = s2_refill || s2_st_wr || s3_amo_v;
    assign data_wr_way  = s3_amo_v ? s3_way : s2_way;
    // TIMING OPT (CB-3, 2026-11-26), part c: the SRAM write enables use the one-hot way: the S2 way from
    // u_s2's control copy (not the 2300-load s2_way flops), the S3 way decoded from its own flop.
    // = decode(data_wr_way) in every cycle. Worked example: AMO in S3, way 3 -> 1000; else a store hit
    // in way 1 -> s2_way_onehot = 0010.
    assign data_wr_way_onehot = s3_amo_v ? s3_way_onehot : s2_way_onehot;
    assign data_wr_set  = s3_amo_v ? s3_set : s2_set;
    assign data_wr_line = s3_amo_v ? s3_merged : s2_write_line;  // AMO result, or the S2 line (refill, merged store, ECC fix)
    wire                   tag_wr;                               // a tag is written this cycle
    wire [WAYS-1:0]        tag_we;                               // one-hot way of that write, WAYS = 4 bits
    wire [TAG_ECC_W-1:0]   tag_wr_check;                         // its check bits, 7 bits (0 wide without ECC)
    wire [TWORD_W-1:0]     tag_wr_word;                          // the coded tag, 57 bits
    assign tag_wr = s2_refill || s2_tag_fix_wr;
    genvar gw;                                                   // loop variable: way whose write enable is built
    generate
        for (gw = 0; gw < WAYS; gw = gw + 1) begin : g_tagwe
            wire [WAY_W-1:0] write_way_number;                   // CB-3: way number gw, WAY_W = 2 bits
            assign write_way_number  = gw;
            assign s3_way_onehot[gw] = (s3_way == write_way_number);   // CB-3
            assign tag_we[gw]        = tag_wr && s2_way_onehot[gw];      // CB-3: was tag_wr && (s2_way == gw)
        end
    endgenerate
    generate
        if (HAS_ECC != 0) begin : g_ecc_enc
            // 8 encoders for the line (one per 64-bit granule) and one for the tag
            genvar ge;                                           // loop variable: granule
            for (ge = 0; ge < GRANULES; ge = ge + 1) begin : g_gran
                l1d_ecc_enc #(.DATA_W(GRAN_BITS), .HAM_W(DATA_HAM_W)) u_enc (
                    .data  (data_wr_line[ge*GRAN_BITS +: GRAN_BITS]),
                    .check (data_wr_check[ge*DATA_ECC_W +: DATA_ECC_W])
                );
            end
            l1d_ecc_enc #(.DATA_W(TAG_W), .HAM_W(TAG_HAM_W)) u_tag_enc (
                .data  (s2_write_tag),
                .check (tag_wr_check)
            );
            assign data_wr_word = {data_wr_check, data_wr_line};
            assign tag_wr_word  = {tag_wr_check, s2_write_tag};
        end else begin : g_no_ecc_enc
            assign data_wr_word = data_wr_line;
            assign tag_wr_word  = s2_write_tag;
        end
    endgenerate

    // ======================================================================
    // Tag array: read at the grant (all ways), output REGISTERED at the end of R; written in S2 (refill tag, ECC-corrected tag)
    // ======================================================================
    wire [WAYS*TWORD_W-1:0] tag_rd;                              // SRAM outputs in Stage R, 228 bits
    l1d_tag_array #(.SETS(SETS), .WAYS(WAYS), .WORD_W(TWORD_W), .INDEX_W(INDEX_W)) u_tags (
        .clk              (clk),
        .read_enable      (any_gnt),
        .read_set_index   (g_set),
        .way_write_enable (tag_we),
        .write_set_index  (s2_set),
        .write_word       (tag_wr_word),
        .read_words_flat  (tag_rd)
    );

    // ======================================================================
    // Data array: one SRAM per way, read at the grant (all ways), output REGISTERED at the end of R; written from flops in S2 / S3
    // ======================================================================
    wire [WAYS*DWORD_W-1:0] line_rd;                             // SRAM outputs in Stage R, 2304 bits
    l1d_data_array #(.SETS(SETS), .WAYS(WAYS), .WORD_W(DWORD_W), .INDEX_W(INDEX_W), .WAY_W(WAY_W)) u_data (
        .clk               (clk),
        .read_enable       (any_gnt),
        .read_set_index    (g_set),
        .write_enable      (data_wr),
        .write_way         (data_wr_way),
        .write_way_onehot  (data_wr_way_onehot),                 // CB-3
        .write_set_index   (data_wr_set),
        .write_word        (data_wr_word),
        .read_words_flat   (line_rd)
    );

    // ======================================================================
    // The R check of a fresh CPU request (PLAN_v6 3.5)
    // ======================================================================
    wire r_lsu;                                                  // the op in R is a CPU request (or a prefetch)
    wire r_fresh_cpu;                                            // ... a fresh, tracked CPU request (the check applies)
    assign r_lsu       = r_valid && (r_kind == K_LSU);
    assign r_fresh_cpu = r_lsu && r_fast && !r_req[REQ_PF_BIT];
    // (2) a write to the set at the SRAM edge of the read (read-first: the old line came back): a store / ECC fix (S2) or an AMO (S3)
    //     wrote at the end of the grant cycle; (3) an AMO writes the set now in S3 (the read is done, the data is stale)
    reg                wl_valid_q;                               // in the previous cycle a line of this bank was written ...
    reg  [INDEX_W-1:0] wl_set_q;                                 // ... in this set, 6 bits
    always @(posedge clk) begin : write_last_proc
        if (rst) wl_valid_q <= 1'b0;
        else     wl_valid_q <= data_wr;
        wl_set_q <= data_wr_set;
    end // always write_last_proc
    wire wl_hit;                                                 // the set was written at the edge of this op's read
    wire amo_hit;                                                // an AMO writes the set now
    wire deps_hit;                                               // the entry waits for an older request of its set
    wire r_kill;                                                 // the fresh request is cancelled
    assign wl_hit   = wl_valid_q && (wl_set_q == r_set);
    assign amo_hit  = s3_amo_v && (s3_set == r_set);
    assign deps_hit = |r_deps;
    assign r_kill   = r_fresh_cpu && (deps_hit || wl_hit || amo_hit);

    // ======================================================================
    // R -> S1 registers (the SRAM outputs are registered here: no logic between the SRAM and these flops)
    // ======================================================================
    always @(posedge clk) begin : r_to_s1_proc
        if (rst) begin
            s1_valid <= 1'b0;
        end else begin
            s1_valid    <= r_valid;
            s1_kind     <= r_kind;
            s1_seat     <= r_seat;
            s1_req      <= r_req;
            s1_mid      <= r_mid;
            s1_line     <= r_line;
            s1_mway     <= r_mway;
            s1_mop      <= r_mop;
            s1_sinv     <= r_sinv;
            s1_sid      <= r_sid;
            s1_kill     <= r_kill;
            tag_words_q <= tag_rd;
            s1_words    <= line_rd;
        end
    end // always r_to_s1_proc

// ---- simulation checks (always on): a booked (non-fresh) request must never see a hazard here; an S2 and an S3 write never meet ----
`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (!rst && r_lsu && !r_fast && !r_req[REQ_PF_BIT] && (deps_hit || wl_hit || amo_hit))
            $display("[%0t] L1D B%0d R  ERROR booked request entry %0d sees a hazard (deps=%b wl=%b amo=%b): booking rule broken",
                     $time, BANK_ID, r_seat, deps_hit, wl_hit, amo_hit);
        if (!rst && s3_amo_v && (s2_refill || s2_st_wr))
            $display("[%0t] L1D B%0d R  ERROR S2 and S3 write the data array in one cycle (AMO bubble broken)", $time, BANK_ID);
        if (!rst && (s2_refill || s2_st_wr || s2_tag_fix_wr) && (s2_way_onehot != ({{(WAYS-1){1'b0}}, 1'b1} << s2_way)))
            $display("[%0t] L1D B%0d R  ERROR CB-3 one-hot write way %b differs from way %0d", $time, BANK_ID, s2_way_onehot, s2_way);
    end
`endif
`ifdef L1D_DEBUG
    always @(posedge clk) begin
        if (!rst && r_kill)
            $display("[%0t] L1D B%0d R  KILL fresh entry %0d set %0d (deps=%b wl=%b amo=%b): ghost, READY again at S2",
                     $time, BANK_ID, r_seat, r_set, deps_hit, wl_hit, amo_hit);
    end
`endif

endmodule

`default_nettype wire
`endif // L1D_BANK_R_V
