// ==========================================================================
// l1d_pf3_stream.v - stream prefetcher, 3-cycle version (training record
//                    at the unit pins -> request in the unit's output
//                    register three clock edges later)
// ==========================================================================
// IN SIMPLE WORDS : the same stream prefetcher as l1d_pf_stream: one note
//   card per 4 KB page (16 cards), a confidence counter per card, DEPTH
//   lines asked ahead of the program in its direction, one request per
//   cycle at most, a duplicate filter of the last RECENT_N requests, no PC
//   needed. What changed is the structure: every card compares itself
//   with the training lanes in S1 and updates itself in S2
//   (l1d_pf3_stream_card), and the first request of a stream comes
//   straight out of S2 (direct candidate) instead of waiting for the
//   scan. The scan (round robin over the pending cards) asks the rest.
//
// STAGES (one clock each):
//   S1  lowest-lane pick, page tag fold, line maths; every card compares
//       itself with the lanes; compares with the event before -> e1_*
//       (and EC_N = 4 copies of the line / target operands, one per 4
//       cards, rule T-6)
//   S2  cards: step rule, write, direct candidate; scan: build the line
//       of the card picked one cycle before; victim for a miss -> the
//       offer flops: one flag per card (direct offer), the direct line
//       picked by the hit vector (one-hot, early) and put together from
//       one late bit (event line or kept ahead), the scan offer; the
//       offer's line + 1 and - 1 are registered too (key window)
//   S3  offer = OR of the flags (cand_*, from flops; a duplicate flag
//       drops its line here); l1d_pf3_rank sends
//       it (output register pf_req_*); the recent keys take the offer at
//       the end of S3. The cards of the event now in S2 check only the
//       recent keys: an S2 offer is made only when S3 is empty, so the
//       window of the last RECENT_N sent keys is exactly as before.
//       Every key carries its line + 1 and - 1, so a card compares its
//       stored ahead line directly (ahead + 1 == key is ahead == key - 1).
//   Scan pick (X1) runs in parallel: it picks the next pending card one
//   cycle ahead (sel_q), from flops.
//
// INSTANTIATED BY : l1d_prefetch_unit, instance u_stream, 1 copy.
//
// PARAMETERS:
//   name         default unit    meaning
//   NB           4       lanes   training lanes = banks
//   ADDR_W       64      bits    cache byte address
//   PA_W         56      bits    physical address bits used
//   LINE_BYTES   64      bytes   cache line
//   PAGE_BYTES   4096    bytes   prefetch never leaves this page
//   STR_ENTRIES  16      cards   stream table size (>= 2)
//   TAG_W        12      bits    hashed page tag
//   DEPTH        4       lines   lead kept ahead of the program (>= 1)
//   CONF_ON      2       count   confidence at which requests start (>=1)
//   CONF_MAX     3       count   confidence saturation value
//   RECENT_N     4       keys    last requested lines kept for dedupe
//
// PORTS (default widths; b = lane, bus slice [b*W +: W]):
//   clk, rst          in  1      clock, synchronous reset (high)
//   pf_enable         in  1      1 = on; 0 flushes (as l1d_pf_stream)
//   train_valid       in  4      training record per lane
//   train_no_train    in  4      do not train
//   train_addr        in  4x64   byte address
//   train_is_store    in  4      store access (unique stream)
//   train_miss        in  4      the access missed
//   train_pf_hit      in  4      the access hit a prefetched line
//   held              in  1      rank still holds the stream's offer
//   cand_valid        out 1      S3 offer: OR of 16 card flags + scan flag
//                                (copy for the rank's control logic)
//   cand_valid_dp     out 5      the same, NB+1 copies for the rank's
//                                data: bit b = bank b's line select, bit
//                                NB = the hold loads
//   cand_line         out 58     its line address (flops, 2:1 by a flop)
//   cand_line_scan    out 58     the scan offer's line (flops)
//   cand_line_direct  out 58     the direct offer's line (flops)
//   cand_pick_scan    out 1      1 = cand_line is cand_line_scan (own
//                                copy of the select flop): the rank
//                                compares both lines and picks last
//   cand_bank         out 4      its bank, one-hot
//   cand_unique       out 1      ask unique
//
// WORKED EXAMPLE (DEPTH 4, page P, lines 50, 51, 52 all misses, idle):
//   52 at the pins in cycle t: S1 (t) compares; S2 (t+1) confirms the
//   stream (conf 2) and makes the direct candidate line 53 (bank 1);
//   S3 (t+2): rank sends it, pf_req_valid[1] = 1 in cycle t+3. The scan
//   then asks 54, 55, 56, one every second cycle (the candidate place is
//   free again after the rank decided).
//
// LOGIC LEVELS (T-9, estimates): S1 fold 2 + card compare 4 + lane AND-OR
//   2 = 8; S2 card about 9 -> per card offer flag; direct line: hit 1 +
//   16-input AND-OR 4 (parallel), late bit OR 3 + 2:1 1; scan pick (round
//   robin over 16) about 6; victim (one-hot classes, lowest card of the
//   lowest class) about 7 + alloc AND 1; S3 offer OR of 17 flags 3,
//   built OV_N = 11 times (rank control, rank data x 5, slot free, 4 key
//   loads); the key
//   window is flops only (the S3 offer is never in it when it counts).
//
// ASSUMPTIONS (V-9.2): A1 one record per cycle (lowest lane). A2 sizes
//   powers of two, CONF_ON <= CONF_MAX, CONF_ON >= 1. A3 a prefetch is a
//   hint: dropping or repeating one costs speed only. A4 a card's
//   confidence never exceeds CONF_MAX (it saturates).
// DIFFERENCES FROM l1d_pf_stream (no function removed):
//   D1 the first request of a stream comes directly from S2 when the
//      scan is idle; the order between streams can differ from the pure
//      round robin.
//   D2 the 2-entry output queue is replaced by the rank's hold place:
//      a candidate is made only when the place is free, so nothing is
//      overwritten (same as D3 of l1d_pf_stream).
// ==========================================================================
`ifndef L1D_PF3_STREAM_V
`define L1D_PF3_STREAM_V
`default_nettype none

module l1d_pf3_stream #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter NB          = 4,    // default 4 lanes
    parameter ADDR_W      = 64,   // default 64 bits: cache address
    parameter PA_W        = 56,   // default 56 bits: physical address
    parameter LINE_BYTES  = 64,   // default 64 bytes: cache line
    parameter PAGE_BYTES  = 4096, // default 4096 bytes: page limit
    parameter STR_ENTRIES = 16,   // default 16 cards
    parameter TAG_W       = 12,   // default 12 bits: page tag
    parameter DEPTH       = 4,    // default 4 lines: lead
    parameter CONF_ON     = 2,    // default 2: confidence to start
    parameter CONF_MAX    = 3,    // default 3: confidence maximum
    parameter RECENT_N    = 4     // default 4: dedupe keys
) (
    // ---- clock and reset ----
    input  wire                      clk,            // clock
    input  wire                      rst,            // sync reset, high
    input  wire                      pf_enable,      // 1 = on
    // ---- training records ----
    input  wire [NB-1:0]             train_valid,    // 4
    input  wire [NB-1:0]             train_no_train, // 4
    input  wire [NB*ADDR_W-1:0]      train_addr,     // 4 x 64
    input  wire [NB-1:0]             train_is_store, // 4
    input  wire [NB-1:0]             train_miss,     // 4
    input  wire [NB-1:0]             train_pf_hit,   // 4
    // ---- rank ----
    input  wire                      held,           // offer still held
    // ---- offer (S3, from flops) ----
    output wire                      cand_valid,     // offer
    output wire [NB:0]               cand_valid_dp,  // NB+1 = 5: same, data
                                                     //   copies (one per
                                                     //   bank + one)
    output wire [ADDR_W-$clog2(LINE_BYTES)-1:0] cand_line, // 58 bits
    output wire [ADDR_W-$clog2(LINE_BYTES)-1:0] cand_line_scan,   // 58 bits
    output wire [ADDR_W-$clog2(LINE_BYTES)-1:0] cand_line_direct, // 58 bits
    output wire                      cand_pick_scan, // 1 = scan line
    output reg  [NB-1:0]             cand_bank,      // 4: one-hot
    output wire                      cand_unique     // ask unique
);

    // ---- local constants (value at the default) ----
    localparam NC         = STR_ENTRIES;            // 16 cards
    localparam OFFSET_W   = $clog2(LINE_BYTES);     // 6 bits
    localparam PAGE_W     = $clog2(PAGE_BYTES);     // 12 bits
    localparam LIP_W      = PAGE_W - OFFSET_W;      // 6 bits line in page
    localparam PAGE_NUM_W = PA_W - PAGE_W;          // 44 bits
    localparam PAD_W      = ADDR_W - PA_W;          // 8 bits
    localparam CONF_W     = $clog2(CONF_MAX + 1);   // 2 bits
    localparam NCLS       = CONF_MAX + 2;           // 5 victim classes
    localparam BANK_W     = $clog2(NB);             // 2 bits
    localparam WIDE_W     = LIP_W + 2;              // 8 bits: no wrap
    localparam [LIP_W-1:0] LIP_TWO = 2;             // 2 lines
    localparam EC_N       = 4;                      // 4 copies of the event
    localparam OV_N       = NB + 7;                 // 11 copies of the offer
    // copies of the scan-offer flop sgo_q (fanout split, T-6): 0 = the
    // offer OR copies for the rank (control + data), 1 = the other offer
    // OR copies (slot free, key loads), 2 = the cand_line page / unique
    // selects, 3 = the offer line / +-1 / tag selects and the bank
    // decode, 4 = cand_pick_scan (the rank's compare selects)
    localparam SG_N       = 5;                      // 5 copies of sgo_q
                                                    //   compare operands
    localparam CPC        = NC / EC_N;              // 4 cards per copy
    localparam EC_W       = 5 * LIP_W + 4;          // 34 bits per copy: 5
                                                    //   lines + 4 E-1 flags
    localparam [WIDE_W-1:0] DEPTH_WIDE = DEPTH;     // 4
    localparam [LIP_W-1:0]  DEPTH_LIP  = DEPTH;     // 4, LIP_W = 6 bits
    localparam [WIDE_W-1:0] LAST_WIDE  = (1 << LIP_W) - 1; // 63
    localparam [LIP_W-1:0]  LAST_LINE  = (1 << LIP_W) - 1; // 63
    localparam [LIP_W-1:0]  LIP_ONE    = 1;         // 1

    // ---- S1: lanes ----
    reg  [NB-1:0]            lane_pick;      // 4: chosen lane
    reg                      lane_seen;      // a lower lane has one
    reg  [NB*TAG_W-1:0]      lane_tag;       // 4 x 12
    wire [NB*LIP_W-1:0]      lane_line;      // 4 x 6
    wire [NB*(LIP_W+1)-1:0]  lane_line_p1;   // 4 x 7
    reg  [NB*LIP_W-1:0]      lane_tgt_up;    // 4 x 6
    reg  [NB*LIP_W-1:0]      lane_tgt_dn;    // 4 x 6
    reg  [WIDE_W-1:0]        line_wide;      // scratch, 8 bits
    reg  [WIDE_W-1:0]        up_sum;         // scratch, 8 bits
    reg  [LIP_W-1:0]         dn_diff;        // scratch, LIP_W = 6 bits
    reg                      pick_any;       // a record is chosen
    reg  [TAG_W-1:0]         pick_tag;       // 12 bits
    reg  [PAGE_NUM_W-1:0]    pick_page;      // 44 bits
    reg  [LIP_W-1:0]         pick_line;      // 6 bits
    reg  [LIP_W-1:0]         pick_tgt_up;    // 6 bits
    reg  [LIP_W-1:0]         pick_tgt_dn;    // 6 bits
    reg                      pick_trig;      // miss or pf hit
    reg                      pick_miss;      // missed
    reg                      pick_store;     // store
    reg                      pick_same_tag;  // tag == E-1's tag
    reg                      pick_same;      // line == E-1's line
    reg                      pick_up;        // line == E-1's line + 1
    reg                      pick_dn;        // line == E-1's line - 1

    // ---- S1 -> S2 registers ----
    reg                      e1_vld;         // event valid
    reg  [TAG_W-1:0]         e1_tag;         // 12 bits
    reg  [PAGE_NUM_W-1:0]    e1_page;        // 44 bits
    reg  [LIP_W-1:0]         e1_line;        // 6 bits
    reg  [LIP_W:0]           e1_line_p1w;    // 7 bits, line + 1
    reg  [LIP_W-1:0]         e1_line_p1;     // 6 bits
    reg  [LIP_W-1:0]         e1_line_m1;     // 6 bits
    reg  [LIP_W-1:0]         e1_line_p2;     // 6 bits, line + 2
    reg  [LIP_W-1:0]         e1_line_m2;     // 6 bits, line - 2
    reg                      e1_top;         // line is 63
    reg                      e1_bot;         // line is 0
    reg  [LIP_W-1:0]         e1_tgt_dn;      // 6 bits
    // copies of {E-1 same tag, same, up, down, line, line + 1, line - 1,
    // target up, target down} for the cards (fanout CPC each, rule T-6)
    wire [EC_N*EC_W-1:0]     ec_word;        // 4 x 34 bits
    // copies of slot_free & pf_enable for the cards (one per CPC cards):
    // the S3 offer OR is built in each copy (kept hierarchy)
    wire [NC:0]              offer_rows;     // 17: scan + 16 direct flags
    wire [EC_N-1:0]          cand_cp;        // 4: copies of cand_valid
    // more copies of the S3 offer flag, each its own OR (kept hierarchy):
    // 0 = rank control, 1 = rank data, 2 = slot free here, 3..6 = the
    // recent-key loads (valid + tag, line, line + 1, line - 1); rule T-6
    wire [OV_N-1:0]          offer_cp;       // 11: copies of cand_valid
    wire [EC_N-1:0]          slot_go_cp;     // 4: the offer place is free
    reg                      e1_trig;        // miss or pf hit
    reg                      e1_miss;        // missed
    reg                      e1_store;       // store
    reg  [NC-1:0]            e2_card;        // 16: cards E-1 wrote

    // ---- cards ----
    wire [NC-1:0]            card_hit;       // 16
    wire [NC-1:0]            card_pend;      // 16
    wire [NC*NCLS-1:0]       card_class;     // 16 x 5: victim class
    wire [NC-1:0]            card_dvld;      // 16: direct candidate
    wire [NC-1:0]            card_devp;      // 16: line from the event line
    wire [NC-1:0]            card_dup;       // 16: the event steps up
    wire [NC*LIP_W-1:0]      card_dline;     // 16 x 6: kept-ahead line
    wire [NC*LIP_W-1:0]      card_dlp1;      // 16 x 6: line + 1
    wire [NC*LIP_W-1:0]      card_dlm1;      // 16 x 6: line - 1
    wire [NC-1:0]            card_ddup;      // 16
    wire [NC-1:0]            card_dstore;    // 16
    wire [NC-1:0]            card_sgo;       // 16: scan: line to ask
    wire [NC*LIP_W-1:0]      card_sline;     // 16 x 6
    wire [NC*LIP_W-1:0]      card_slp1;      // 16 x 6: line + 1
    wire [NC*LIP_W-1:0]      card_slm1;      // 16 x 6: line - 1
    wire [NC-1:0]            card_sdup;      // 16
    wire [NC*PAGE_NUM_W-1:0] card_page;      // 16 x 44
    wire [NC*TAG_W-1:0]      card_tag;       // 16 x 12
    wire [NC-1:0]            card_store;     // 16
    wire [NC-1:0]            card_alloc;     // 16: allocate now
    wire [NC-1:0]            prev_card;      // 16: E-1 wrote the card

    // ---- S2: allocation ----
    wire                     any_hit;        // a card is the event's
    wire                     alloc;          // allocate a card
    // victim order as one-hot classes: class 0 = free card, class k =
    // valid with confidence k - 1 (old rank order: free first, then lowest
    // confidence, then lowest index)
    reg  [NCLS*NC-1:0]       class_cards;    // 5 x 16: cards of class k
    reg  [NCLS-1:0]          class_lower;    // 5: a lower class has a card
    wire [NCLS*NC-1:0]       class_first;    // 5 x 16: lowest card of class k
    wire [NCLS-1:0]          class_any;      // 5: class k has a card
    reg  [NC-1:0]            victim;         // 16: one-hot

    // ---- recent keys (written at the end of S3) and the key window ----
    reg  [RECENT_N-1:0]       recent_vld;    // 4
    reg  [RECENT_N*TAG_W-1:0] recent_tag;    // 4 x 12
    reg  [RECENT_N*LIP_W-1:0] recent_line;   // 4 x 6
    reg  [RECENT_N*LIP_W-1:0] recent_lp1;    // 4 x 6: each key line + 1
    reg  [RECENT_N*LIP_W-1:0] recent_lm1;    // 4 x 6: each key line - 1
    wire [RECENT_N-1:0]       win_tag_eq;    // 4: key tag == e1_tag
    wire [RECENT_N-1:0]       win_ev_p1;     // 4: key == {e1 tag, line + 1}
    wire [RECENT_N-1:0]       win_ev_m1;     // 4: key == {e1 tag, line - 1}
    wire                      ev_dup_p1;     // e1 line + 1 was sent
    wire                      ev_dup_m1;     // e1 line - 1 was sent

    // ---- scan (round robin over pending cards, one cycle ahead) ----
    reg  [NC-1:0]            rr_mask_q;      // 16: cards after the last
    reg  [NC-1:0]            sel_q;          // 16: picked card, one-hot
    reg                      sel_vld_q;      // a card is picked
    reg  [NC-1:0]            pick_high;      // 16: pending after mask
    reg  [NC-1:0]            pick_base;      // 16: where to search
    reg  [NC-1:0]            pick_one;       // 16: next pick
    reg  [NC-1:0]            pick_above;     // 16: next mask
    reg                      pick_seen;      // scratch
    wire                     slot_free;      // the offer place is free
    wire                     scan_take;      // the scan line may go
    reg                      s_go;           // scan: a line to ask
    reg  [LIP_W-1:0]         s_line;         // 6 bits
    reg                      s_dup;          // duplicate
    reg  [PAGE_NUM_W-1:0]    s_page;         // 44 bits
    reg  [TAG_W-1:0]         s_tag;          // 12 bits
    reg                      s_store;        // store stream
    reg  [LIP_W-1:0]         d_line;         // 6 bits: hit card's line
    reg                      d_evp;          // it comes from the event line
    reg                      d_up;           // the event steps up
    reg  [LIP_W-1:0]         d_kp;           // 6 bits: kept-ahead line
    reg  [LIP_W-1:0]         d_kp_p1;        // 6 bits: + 1
    reg  [LIP_W-1:0]         d_kp_m1;        // 6 bits: - 1
    reg  [LIP_W-1:0]         d_lp1;          // 6 bits: its line + 1
    reg  [LIP_W-1:0]         d_lm1;          // 6 bits: its line - 1
    reg  [LIP_W-1:0]         s_lp1;          // 6 bits: scan line + 1
    reg  [LIP_W-1:0]         s_lm1;          // 6 bits: scan line - 1
    reg  [LIP_W-1:0]         d_lp1_q;        // 6 bits (flop)
    reg  [LIP_W-1:0]         d_lm1_q;        // 6 bits (flop)
    reg  [LIP_W-1:0]         s_lp1_q;        // 6 bits (flop)
    reg  [LIP_W-1:0]         s_lm1_q;        // 6 bits (flop)
    reg                      d_store;        // hit card's store bit

    // ---- offer flops (S2 -> S3) ----
    reg  [NC-1:0]            dvld_q;         // 16: direct line per card
    reg  [NC-1:0]            ddup_q;         // 16: it is a duplicate
    wire [NC-1:0]            doffer;         // 16: S3 direct offer per card
    reg  [LIP_W-1:0]         d_line_q;       // 6 bits: direct line
    reg                      d_store_q;      // direct: store stream
    reg  [TAG_W-1:0]         e2_tag;         // 12 bits: direct key tag
    reg  [PAGE_NUM_W-1:0]    e2_page;        // 44 bits: direct page
    reg  [LIP_W-1:0]         e2_line;        // 6 bits: event before's line
    reg  [LIP_W:0]           e2_line_p1w;    // 7 bits: its line + 1
    reg  [LIP_W-1:0]         e2_tgt_dn;      // 6 bits: its down target
    reg                      e2_store;       // its store bit
    reg                      sdup_q;         // it is a duplicate
    wire                     soffer;         // S3 scan offer (rank
                                             //   offer OR copies)
    wire                     soffer_b;       // the same from copy 1 (the
                                             //   other offer OR copies)
    wire [SG_N-1:0]          sgo_cp;         // 5: copies of sgo_q
    wire [NC:0]              offer_rows_b;   // 17: {soffer_b, doffer}
    reg  [LIP_W-1:0]         s_line_q;       // 6 bits
    reg  [PAGE_NUM_W-1:0]    s_page_q;       // 44 bits
    reg  [TAG_W-1:0]         s_tag_q;        // 12 bits
    reg                      s_store_q;      // store stream
    wire [TAG_W-1:0]         offer_tag;      // 12 bits: offer key tag
    wire [LIP_W-1:0]         offer_line;     // 6 bits: offer key line
    wire [LIP_W-1:0]         offer_lp1;      // 6 bits: offer line + 1
    wire [LIP_W-1:0]         offer_lm1;      // 6 bits: offer line - 1

    // ======================================================================
    // S1: lane values (from the pins)
    // ======================================================================
    genvar lane_g; // training lane
    generate
        for (lane_g = 0; lane_g < NB; lane_g = lane_g + 1) begin : g_lane
            assign lane_line[lane_g*LIP_W +: LIP_W] =
                train_addr[lane_g*ADDR_W + OFFSET_W +: LIP_W];
            assign lane_line_p1[lane_g*(LIP_W+1) +: LIP_W+1] =
                {1'b0, train_addr[lane_g*ADDR_W + OFFSET_W +: LIP_W]}
                + {{LIP_W{1'b0}}, 1'b1};
        end // for lane_g
    endgenerate

    always @* begin : lane_proc
        integer lane_idx; // training lane
        integer bit_idx;  // page number bit
        lane_tag = {(NB*TAG_W){1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            // page tag: page number XOR-folded into TAG_W bits
            for (bit_idx = 0; bit_idx < PAGE_NUM_W; bit_idx = bit_idx + 1) begin
                lane_tag[lane_idx*TAG_W + (bit_idx % TAG_W)] =
                    lane_tag[lane_idx*TAG_W + (bit_idx % TAG_W)]
                    ^ train_addr[lane_idx*ADDR_W + PAGE_W + bit_idx];
            end // for bit_idx
            // stream end targets: line + DEPTH (capped) and line - DEPTH
            line_wide = {{(WIDE_W-LIP_W){1'b0}},
                         lane_line[lane_idx*LIP_W +: LIP_W]};
            up_sum    = line_wide + DEPTH_WIDE;
            lane_tgt_up[lane_idx*LIP_W +: LIP_W] =
                (up_sum > LAST_WIDE) ? LAST_LINE : up_sum[LIP_W-1:0];
            dn_diff   = lane_line[lane_idx*LIP_W +: LIP_W] - DEPTH_LIP;
            lane_tgt_dn[lane_idx*LIP_W +: LIP_W] =
                (line_wide >= DEPTH_WIDE) ? dn_diff
                                          : {LIP_W{1'b0}};
        end // for lane_idx
    end // always lane_proc

    // ---- S1: lowest trainable lane, its values, compares with E-1 ----
    always @* begin : pick_proc
        integer lane_idx; // training lane
        lane_seen = 1'b0;
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_pick[lane_idx] = train_valid[lane_idx]
                                  & ~train_no_train[lane_idx] & ~lane_seen;
            lane_seen = lane_seen
                        | (train_valid[lane_idx] & ~train_no_train[lane_idx]);
        end // for lane_idx
        pick_any      = 1'b0;
        pick_tag      = {TAG_W{1'b0}};
        pick_page     = {PAGE_NUM_W{1'b0}};
        pick_line     = {LIP_W{1'b0}};
        pick_tgt_up   = {LIP_W{1'b0}};
        pick_tgt_dn   = {LIP_W{1'b0}};
        pick_trig     = 1'b0;
        pick_miss     = 1'b0;
        pick_store    = 1'b0;
        pick_same_tag = 1'b0;
        pick_same     = 1'b0;
        pick_up       = 1'b0;
        pick_dn       = 1'b0;
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            pick_any    = pick_any | lane_pick[lane_idx];
            pick_tag    = pick_tag | ({TAG_W{lane_pick[lane_idx]}}
                          & lane_tag[lane_idx*TAG_W +: TAG_W]);
            pick_page   = pick_page | ({PAGE_NUM_W{lane_pick[lane_idx]}}
                          & train_addr[lane_idx*ADDR_W + PAGE_W +: PAGE_NUM_W]);
            pick_line   = pick_line | ({LIP_W{lane_pick[lane_idx]}}
                          & lane_line[lane_idx*LIP_W +: LIP_W]);
            pick_tgt_up = pick_tgt_up | ({LIP_W{lane_pick[lane_idx]}}
                          & lane_tgt_up[lane_idx*LIP_W +: LIP_W]);
            pick_tgt_dn = pick_tgt_dn | ({LIP_W{lane_pick[lane_idx]}}
                          & lane_tgt_dn[lane_idx*LIP_W +: LIP_W]);
            pick_trig   = pick_trig | (lane_pick[lane_idx]
                          & (train_miss[lane_idx] | train_pf_hit[lane_idx]));
            pick_miss   = pick_miss | (lane_pick[lane_idx] & train_miss[lane_idx]);
            pick_store  = pick_store | (lane_pick[lane_idx]
                          & train_is_store[lane_idx]);
            // compares with the event now in S2 (its card is being written,
            // so the cards' own S1 compares are stale for that card)
            pick_same_tag = pick_same_tag | (lane_pick[lane_idx]
                & (lane_tag[lane_idx*TAG_W +: TAG_W] == e1_tag));
            pick_same = pick_same | (lane_pick[lane_idx]
                & (lane_line[lane_idx*LIP_W +: LIP_W] == e1_line));
            pick_up   = pick_up | (lane_pick[lane_idx]
                & ({1'b0, lane_line[lane_idx*LIP_W +: LIP_W]} == e1_line_p1w));
            pick_dn   = pick_dn | (lane_pick[lane_idx]
                & (lane_line_p1[lane_idx*(LIP_W+1) +: LIP_W+1]
                   == {1'b0, e1_line}));
        end // for lane_idx
    end // always pick_proc

    always @(posedge clk) begin : s1_reg_proc
        if (rst) begin
            e1_vld <= 1'b0;
        end
        else begin
            e1_vld <= pick_any & pf_enable;
        end // rst
        e1_tag      <= pick_tag;
        e1_page     <= pick_page;
        e1_line     <= pick_line;
        e1_line_p1w <= {1'b0, pick_line} + {{LIP_W{1'b0}}, 1'b1};
        e1_line_p1  <= pick_line + LIP_ONE;
        e1_line_m1  <= pick_line - LIP_ONE;
        e1_line_p2  <= pick_line + LIP_TWO;
        e1_line_m2  <= pick_line - LIP_TWO;
        e1_top      <= (pick_line == LAST_LINE);
        e1_bot      <= (pick_line == {LIP_W{1'b0}});
        e1_tgt_dn   <= pick_tgt_dn;
        e1_trig     <= pick_trig;
        e1_miss     <= pick_miss;
        e1_store    <= pick_store;
    end // always s1_reg_proc

    // ======================================================================
    // S2: key window = the RECENT_N sent keys (flops only). The offer now
    // in S3 is not in it: an S2 offer (scan or direct) is made only when
    // the S3 place is empty (scan_take and slot_go need ~cand_valid), so
    // whenever an S2 duplicate flag is used, no key is in S3 and the
    // window is exactly the recent keys (no late valid bit, rule T-9)
    // ======================================================================
    assign offer_tag  = sgo_cp[3] ? s_tag_q  : e2_tag;
    assign offer_line = sgo_cp[3] ? s_line_q : d_line_q;
    // the keys' line +/- 1 (6-bit wrap, as the cards' ahead +/- 1): the
    // cards compare their stored ahead line with these instead of adding
    // 1 to ahead first; the offer's come ready-made from the card (flops)
    assign offer_lp1 = sgo_cp[3] ? s_lp1_q : d_lp1_q;
    assign offer_lm1 = sgo_cp[3] ? s_lm1_q : d_lm1_q;

    genvar key_g; // window key
    generate
        for (key_g = 0; key_g < RECENT_N; key_g = key_g + 1) begin : g_win_eq
            assign win_tag_eq[key_g] = (recent_tag[key_g*TAG_W +: TAG_W] == e1_tag);
            // the event's line +/- 1 among the keys (same for every card)
            assign win_ev_p1[key_g] = recent_vld[key_g] & win_tag_eq[key_g]
                & (recent_line[key_g*LIP_W +: LIP_W] == e1_line_p1);
            assign win_ev_m1[key_g] = recent_vld[key_g] & win_tag_eq[key_g]
                & (recent_line[key_g*LIP_W +: LIP_W] == e1_line_m1);
        end // for key_g
    endgenerate
    assign ev_dup_p1 = |win_ev_p1;
    assign ev_dup_m1 = |win_ev_m1;

    // ======================================================================
    // S2: the cards
    // ======================================================================
    genvar ecopy_g; // event operand copy
    generate
        for (ecopy_g = 0; ecopy_g < EC_N; ecopy_g = ecopy_g + 1) begin : g_ecopy
            l1d_pf3_or_rows #(.W (1), .N (NC + 1)) u_cand_or (
                .rows   (offer_rows),
                .or_out (cand_cp[ecopy_g])
            );
            assign slot_go_cp[ecopy_g] = ~cand_cp[ecopy_g] & ~held & pf_enable;
            l1d_pf3_rep #(.W (EC_W), .HAS_RST (0), .HAS_EN (0)) u_ec (
                .clk (clk), .rst (1'b0), .en (1'b1),
                .d   ({pick_same_tag, pick_same, pick_up, pick_dn,
                       pick_line, pick_line + LIP_ONE, pick_line - LIP_ONE,
                       pick_tgt_up, pick_tgt_dn}),
                .q   (ec_word[ecopy_g*EC_W +: EC_W])
            );
        end // for ecopy_g
    endgenerate

    genvar card_g; // stream card
    generate
        for (card_g = 0; card_g < NC; card_g = card_g + 1) begin : g_card
            assign prev_card[card_g]  = e2_card[card_g];
            assign card_alloc[card_g] = alloc & victim[card_g];
            l1d_pf3_stream_card #(
                .NB         (NB),
                .TAG_W      (TAG_W),
                .PAGE_NUM_W (PAGE_NUM_W),
                .LIP_W      (LIP_W),
                .CONF_W     (CONF_W),
                .CONF_ON    (CONF_ON),
                .CONF_MAX   (CONF_MAX),
                .RECENT_N   (RECENT_N),
                .DEPTH      (DEPTH)
            ) u_card (
                .clk           (clk),
                .rst           (rst),
                .flush         (~pf_enable),
                .lane_pick     (lane_pick),
                .lane_tag      (lane_tag),
                .lane_line     (lane_line),
                .lane_line_p1  (lane_line_p1),
                .ev_vld        (e1_vld),
                .ev_page       (e1_page),
                .ev_line       (ec_word[(card_g/CPC)*EC_W + 4*LIP_W +: LIP_W]),
                .ev_line_p1    (ec_word[(card_g/CPC)*EC_W + 3*LIP_W +: LIP_W]),
                .ev_line_m1    (ec_word[(card_g/CPC)*EC_W + 2*LIP_W +: LIP_W]),
                .ev_top        (e1_top),
                .ev_bot        (e1_bot),
                .ev_tgt_up     (ec_word[(card_g/CPC)*EC_W + LIP_W +: LIP_W]),
                .ev_tgt_dn     (ec_word[(card_g/CPC)*EC_W +: LIP_W]),
                .ev_trig       (e1_trig),
                .prev_card     (prev_card[card_g]),
                .prev_same_tag (ec_word[(card_g/CPC)*EC_W + 5*LIP_W + 3]),
                .prev_same     (ec_word[(card_g/CPC)*EC_W + 5*LIP_W + 2]),
                .prev_up       (ec_word[(card_g/CPC)*EC_W + 5*LIP_W + 1]),
                .prev_dn       (ec_word[(card_g/CPC)*EC_W + 5*LIP_W]),
                .alloc_en      (card_alloc[card_g]),
                .p_tag         (e2_tag),
                .p_page        (e2_page),
                .p_line        (e2_line),
                .p_line_p1w    (e2_line_p1w),
                .p_tgt_dn      (e2_tgt_dn),
                .p_store       (e2_store),
                .scan_idle     (~sel_vld_q),
                .scan_sel      (sel_q[card_g]),
                .slot_go       (slot_go_cp[card_g/CPC]),
                .recent_tag_eq (win_tag_eq),
                .ev_dup_p1     (ev_dup_p1),
                .ev_dup_m1     (ev_dup_m1),
                .recent_lp1    (recent_lp1),
                .recent_lm1    (recent_lm1),
                .recent_tag    (recent_tag),
                .recent_vld    (recent_vld),
                .hit           (card_hit[card_g]),
                .pend_out      (card_pend[card_g]),
                .class_out     (card_class[card_g*NCLS +: NCLS]),
                .dir_vld       (card_dvld[card_g]),
                .dir_evp       (card_devp[card_g]),
                .dir_up        (card_dup[card_g]),
                .dir_kp        (card_dline[card_g*LIP_W +: LIP_W]),
                .dir_kp_p1     (card_dlp1[card_g*LIP_W +: LIP_W]),
                .dir_kp_m1     (card_dlm1[card_g*LIP_W +: LIP_W]),
                .dir_dup       (card_ddup[card_g]),
                .dir_store     (card_dstore[card_g]),
                .scan_go       (card_sgo[card_g]),
                .scan_line     (card_sline[card_g*LIP_W +: LIP_W]),
                .scan_lp1      (card_slp1[card_g*LIP_W +: LIP_W]),
                .scan_lm1      (card_slm1[card_g*LIP_W +: LIP_W]),
                .scan_dup      (card_sdup[card_g]),
                .page_out      (card_page[card_g*PAGE_NUM_W +: PAGE_NUM_W]),
                .tag_out       (card_tag[card_g*TAG_W +: TAG_W]),
                .store_out     (card_store[card_g])
            );
        end // for card_g
    endgenerate

    // ---- S2: allocation of a card for a missing page ----
    assign any_hit = |card_hit;
    assign alloc   = e1_vld & ~any_hit & e1_miss;

    // Same choice as the rank compare (free first, then lowest confidence,
    // then lowest index), built flat from the cards' class flops: per class
    // the lowest card by a kept-hierarchy log-depth find-first (4 OR levels
    // + 1, in parallel for the 5 classes), "a lower class has a card" from
    // the find-first's any bit (prefix of 5), combine 2 = about 8 levels
    // with no ripple chain (rule T-9). Assumes conf <= CONF_MAX (A4).
    always @* begin : class_cards_proc
        integer card_idx;  // card
        integer class_idx; // victim class
        // the cards keep their class as a one-hot flop (class_out)
        for (class_idx = 0; class_idx < NCLS; class_idx = class_idx + 1) begin
            for (card_idx = 0; card_idx < NC; card_idx = card_idx + 1) begin
                class_cards[class_idx*NC + card_idx] =
                    card_class[card_idx*NCLS + class_idx];
            end // for card_idx
        end // for class_idx
    end // always class_cards_proc

    genvar class_g; // generate index: victim class
    generate
        for (class_g = 0; class_g < NCLS; class_g = class_g + 1) begin : g_class
            l1d_pf3_ffs #(.W (NC)) u_class_first (
                .in_vec  (class_cards[class_g*NC +: NC]),
                .low_oh  (class_first[class_g*NC +: NC]),
                .low2_oh (),
                .any_set (class_any[class_g]),
                .two_set ()
            );
        end // for class_g
    endgenerate

    always @* begin : victim_proc
        integer class_idx; // victim class
        class_lower[0] = 1'b0;
        for (class_idx = 1; class_idx < NCLS; class_idx = class_idx + 1) begin
            class_lower[class_idx] = class_lower[class_idx-1]
                | class_any[class_idx-1];
        end // for class_idx
        victim = {NC{1'b0}};
        for (class_idx = 0; class_idx < NCLS; class_idx = class_idx + 1) begin
            victim = victim | (class_first[class_idx*NC +: NC]
                               & {NC{~class_lower[class_idx]}});
        end // for class_idx
    end // always victim_proc

    // ---- S2: offers. A new offer only when the place is free (no offer
    //      in S3 and none held by the rank) ----
    assign slot_free = ~offer_cp[NB+2] & ~held;
    assign scan_take = slot_free & pf_enable;
    // direct offer allowed: slot_go_cp & ~sel_vld_q (inside the cards)

    always @* begin : offer_sel_proc
        integer card_idx; // stream card
        s_go    = 1'b0;
        s_line  = {LIP_W{1'b0}};
        s_dup   = 1'b0;
        s_page  = {PAGE_NUM_W{1'b0}};
        s_tag   = {TAG_W{1'b0}};
        s_store = 1'b0;
        d_evp   = 1'b0;
        d_up    = 1'b0;
        d_kp    = {LIP_W{1'b0}};
        d_kp_p1 = {LIP_W{1'b0}};
        d_kp_m1 = {LIP_W{1'b0}};
        s_lp1   = {LIP_W{1'b0}};
        s_lm1   = {LIP_W{1'b0}};
        d_store = 1'b0;
        for (card_idx = 0; card_idx < NC; card_idx = card_idx + 1) begin
            // scan: the card picked one cycle before (sel_q, a flop)
            s_go    = s_go | card_sgo[card_idx];
            s_line  = s_line | ({LIP_W{sel_q[card_idx]}}
                      & card_sline[card_idx*LIP_W +: LIP_W]);
            s_lp1   = s_lp1 | ({LIP_W{sel_q[card_idx]}}
                      & card_slp1[card_idx*LIP_W +: LIP_W]);
            s_lm1   = s_lm1 | ({LIP_W{sel_q[card_idx]}}
                      & card_slm1[card_idx*LIP_W +: LIP_W]);
            s_dup   = s_dup | (sel_q[card_idx] & card_sdup[card_idx]);
            s_page  = s_page | ({PAGE_NUM_W{sel_q[card_idx]}}
                      & card_page[card_idx*PAGE_NUM_W +: PAGE_NUM_W]);
            s_tag   = s_tag | ({TAG_W{sel_q[card_idx]}}
                      & card_tag[card_idx*TAG_W +: TAG_W]);
            s_store = s_store | (sel_q[card_idx] & card_store[card_idx]);
            // direct: the event's card (hit vector, one-hot and early)
            d_evp   = d_evp | (card_hit[card_idx] & card_devp[card_idx]);
            d_up    = d_up | (card_hit[card_idx] & card_dup[card_idx]);
            d_kp    = d_kp | ({LIP_W{card_hit[card_idx]}}
                      & card_dline[card_idx*LIP_W +: LIP_W]);
            d_kp_p1 = d_kp_p1 | ({LIP_W{card_hit[card_idx]}}
                      & card_dlp1[card_idx*LIP_W +: LIP_W]);
            d_kp_m1 = d_kp_m1 | ({LIP_W{card_hit[card_idx]}}
                      & card_dlm1[card_idx*LIP_W +: LIP_W]);
            d_store = d_store | (card_hit[card_idx] & card_dstore[card_idx]);
        end // for card_idx
        // the direct line: from the event line (+/- 1 by the step) or the
        // hit card's kept ahead; the late d_evp picks last (one bit OR of
        // 16 instead of a 6-bit select in every card)
        if (d_evp) begin
            d_line = d_up ? e1_line_p1 : e1_line_m1;
            d_lp1  = d_up ? e1_line_p2 : e1_line;
            d_lm1  = d_up ? e1_line    : e1_line_m2;
        end
        else begin
            d_line = d_kp;
            d_lp1  = d_kp_p1;
            d_lm1  = d_kp_m1;
        end // else (kept ahead)
    end // always offer_sel_proc

    // ---- scan pick for the next cycle (round robin, from flops) ----
    always @* begin : scan_pick_proc
        integer card_idx; // stream card
        pick_high = card_pend & rr_mask_q;
        pick_base = (|pick_high) ? pick_high : card_pend;
        pick_one  = {NC{1'b0}};
        pick_seen = 1'b0;
        for (card_idx = 0; card_idx < NC; card_idx = card_idx + 1) begin
            if (pick_base[card_idx] && !pick_seen) begin
                pick_one[card_idx] = 1'b1;
                pick_seen          = 1'b1;
            end
        end // for card_idx
        pick_above = {NC{1'b0}};
        pick_seen  = 1'b0;
        for (card_idx = 0; card_idx < NC; card_idx = card_idx + 1) begin
            pick_above[card_idx] = pick_seen;
            pick_seen            = pick_seen | pick_one[card_idx];
        end // for card_idx
    end // always scan_pick_proc

    always @(posedge clk) begin : s2_reg_proc
        if (rst || !pf_enable) begin
            sel_vld_q <= 1'b0;
            sel_q     <= {NC{1'b0}};
            rr_mask_q <= {NC{1'b1}};
            dvld_q    <= {NC{1'b0}};
            ddup_q    <= {NC{1'b0}};
            sdup_q    <= 1'b0;
            e2_card   <= {NC{1'b0}};
        end
        else begin
            if (!sel_vld_q || scan_take) begin
                sel_vld_q <= |card_pend;
                sel_q     <= pick_one;
                if (|card_pend) begin
                    rr_mask_q <= pick_above;
                end
            end // pick
            // scan first (a direct offer needs the scan idle, scan_idle);
            // a duplicate is not offered
            // the flags are registered raw; S3 drops a duplicate
            // (the scan-taken flag sgo_q is the kept copies u_sgo)
            sdup_q <= s_dup;
            dvld_q <= card_dvld;
            ddup_q <= card_ddup;
            e2_card  <= card_hit | card_alloc;
        end // rst
        // offer data, loaded every cycle (used only with a flag)
        d_line_q  <= d_line;
        d_lp1_q   <= d_lp1;
        d_lm1_q   <= d_lm1;
        s_lp1_q   <= s_lp1;
        s_lm1_q   <= s_lm1;
        d_store_q <= d_store;
        e2_tag    <= e1_tag;
        e2_page   <= e1_page;
        e2_line   <= e1_line;
        e2_line_p1w <= e1_line_p1w;
        e2_tgt_dn <= e1_tgt_dn;
        e2_store  <= e1_store;
        s_line_q  <= s_line;
        s_page_q  <= s_page;
        s_tag_q   <= s_tag;
        s_store_q <= s_store;
    end // always s2_reg_proc

    // ======================================================================
    // S3: the offer and the recent keys
    // ======================================================================
    // the copies of sgo_q: same reset and data, kept hierarchy so
    // synthesis does not merge them
    l1d_pf3_rep #(.W(SG_N), .HAS_RST(1), .HAS_EN(0)) u_sgo (
        .clk (clk),
        .rst (rst | ~pf_enable),
        .en  (1'b1),
        .d   ({SG_N{scan_take & s_go}}),
        .q   (sgo_cp)
    );
    assign soffer       = sgo_cp[0] & ~sdup_q;
    assign soffer_b     = sgo_cp[1] & ~sdup_q;
    assign doffer       = dvld_q & ~ddup_q;
    assign offer_rows   = {soffer, doffer};
    assign offer_rows_b = {soffer_b, doffer};
    genvar ocopy_g; // offer flag copy
    generate
        for (ocopy_g = 0; ocopy_g < OV_N; ocopy_g = ocopy_g + 1) begin : g_offer
            l1d_pf3_or_rows #(.W (1), .N (NC + 1)) u_offer_or (
                .rows   ((ocopy_g <= NB) ? offer_rows : offer_rows_b),
                .or_out (offer_cp[ocopy_g])
            );
        end // for ocopy_g
    endgenerate
    assign cand_valid    = offer_cp[0];
    assign cand_valid_dp = offer_cp[NB+1:1];
    assign cand_line   = {{PAD_W{1'b0}}, (sgo_cp[2] ? s_page_q : e2_page),
                          offer_line};
    assign cand_unique = sgo_cp[2] ? s_store_q : d_store_q;
    // both possible lines (flops) and the select copy, for the rank's
    // compares: cand_line == (cand_pick_scan ? scan : direct) always
    assign cand_line_scan   = {{PAD_W{1'b0}}, s_page_q, s_line_q};
    assign cand_line_direct = {{PAD_W{1'b0}}, e2_page, d_line_q};
    assign cand_pick_scan   = sgo_cp[4];

    always @* begin : bank_proc
        integer bank_idx; // cache bank
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
            cand_bank[bank_idx] = (offer_line[BANK_W-1:0] == bank_idx[BANK_W-1:0]);
        end // for bank_idx
    end // always bank_proc

    always @(posedge clk) begin : recent_proc
        if (rst || !pf_enable) begin
            recent_vld <= {RECENT_N{1'b0}};
        end
        else if (offer_cp[NB+3]) begin
            recent_vld <= {recent_vld[RECENT_N-2:0], 1'b1};
        end // rst / offer
        if (offer_cp[NB+3]) begin
            recent_tag  <= {recent_tag[(RECENT_N-1)*TAG_W-1:0], offer_tag};
        end
        if (offer_cp[NB+4]) begin
            recent_line <= {recent_line[(RECENT_N-1)*LIP_W-1:0], offer_line};
        end
        if (offer_cp[NB+5]) begin
            recent_lp1  <= {recent_lp1[(RECENT_N-1)*LIP_W-1:0], offer_lp1};
        end
        if (offer_cp[NB+6]) begin
            recent_lm1  <= {recent_lm1[(RECENT_N-1)*LIP_W-1:0], offer_lm1};
        end
    end // always recent_proc

`ifdef L1D_DEBUG
    // ---- checks (simulation only) ----
    always @(posedge clk) begin : debug_proc
        if (!rst) begin
            if ((card_hit & (card_hit - 1'b1)) != {NC{1'b0}}) begin
                $display("[%0t] L1D PF3_STREAM ERROR two cards hit", $time);
            end
            if (alloc && ((victim & (victim - 1'b1)) != {NC{1'b0}})) begin
                $display("[%0t] L1D PF3_STREAM ERROR victim not one-hot",
                         $time);
            end
            if (sgo_cp[0] && (dvld_q != {NC{1'b0}})) begin
                $display("[%0t] L1D PF3_STREAM ERROR scan and direct offer",
                         $time);
            end
        end // !rst
    end // always debug_proc
`endif

endmodule

`default_nettype wire
`endif // L1D_PF3_STREAM_V
