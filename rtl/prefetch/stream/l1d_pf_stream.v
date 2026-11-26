// ==========================================================================
// l1d_pf_stream.v - stream prefetcher of the L1D (page table of streams)
// ==========================================================================
// IN SIMPLE WORDS : A program that walks memory one 64-byte line after
//   the other inside a 4 KB page is a "stream". This block keeps one
//   note card per page (16 cards). Each real access moves the card of
//   its page: step +1 or -1 raises a confidence counter, any other jump
//   starts the card again. When the confidence reaches CONF_ON the block
//   asks the L2 for the next DEPTH lines in the direction of travel,
//   one request per cycle, and it stays DEPTH lines ahead of the program.
//   It needs no instruction PC and trains on physical addresses.
//   Spec: ../README.md. Shared rules: ../../README.md (sections 1 to 6).
//
// STAGES (one clock each, every stage output is a flop; T-1, T-5):
//   S0  filter + hash + 16 tag compares + victim pick  -> m_* regs
//   S1  table read (one-hot), step rule, write back    -> table, pend
//   S2  pick one pending stream (round robin)          -> sel_q
//   S3  build the next line (ahead +/- 1), page edge   -> cand_*
//   S4  duplicate filter + 2-entry output queue        -> pf_* regs
//   Training to first request: 5 registered stages (README target is
//   3 to 4: gap of 1 to 2 cycles, see GR-1 note below).
//
// INSTANTIATED BY : the L1D prefetch wrapper, one instance per cache.
//
// PARAMETERS (V-7.10), also in the module header below:
//   name         default  unit    meaning
//   PA_W         56       bits    physical address width
//   LINE_BYTES   64       bytes   cache line
//   PAGE_BYTES   4096     bytes   prefetch never leaves this page
//   STR_ENTRIES  16       entries stream table size (>= 2)
//   TAG_W        12       bits    hashed page tag for the compare
//   DEPTH        4        lines   lead kept ahead of the program (>= 1)
//   CONF_ON      2        count   confidence at which issuing starts
//   CONF_MAX     3        count   confidence saturation value
//   RECENT_N     4        entries last requested lines kept for dedupe
//
// PORTS (default widths):
//   clk, rst                 in  1   clock, synchronous reset (high)
//   pf_enable                in  1   global on/off, low flushes output
//   tr_valid                 in  1   a real demand access this cycle
//   tr_addr                  in  56  physical address, [55:6] is used
//   tr_is_store              in  1   store (ReadUnique stream)
//   tr_no_train              in  1   AMO / LR / SC / NC: ignore
//   tr_miss                  in  1   access missed (same cycle as valid)
//   tr_pf_hit                in  1   hit on a prefetched line (same cycle)
//   pf_ready                 in  1   the cache can take a prefetch
//   pf_valid                 out 1   prefetch request (registered)
//   pf_line_addr             out 50  physical line address; bank = [1:0]
//   pf_unique                out 1   1 = ReadUnique
//   pf_flag                  out 1   always 1: marks a prefetch
//
// WORKED EXAMPLE (README section 11 A, DEPTH 4, all misses, page 0x12345):
//   line 50 -> allocate card (conf 0); line 51 -> conf 1;
//   line 52 -> conf 2, ahead = 52, target = 56 -> requests 53,54,55,56
//   (one per cycle, banks 1,2,3,0); line 53 (hit on prefetched line)
//   -> conf 3, target 57 -> request 57.
//
// ASSUMPTIONS (V-9.2), each checked under L1D_DEBUG:
//   A1 at most one training event per cycle, in program order.
//   A2 tr_miss / tr_pf_hit are valid in the same cycle as tr_valid (the
//      parent delays the address until the cache answer is known).
//   A3 STR_ENTRIES >= 2; LINE_BYTES, PAGE_BYTES powers of two;
//      CONF_ON <= CONF_MAX; PA_W > page bits.
//   A4 a prefetch is a hint: dropping or repeating one costs speed only.
//
// DEVIATIONS FROM THE README (named, none removes a function):
//   D1 each card also stores the full 44-bit page number (README keeps
//      only the 12-bit tag). The issue scan runs after the access, so
//      it needs the page; the output page is the page of the latest
//      access to the card (as the README says).
//   D2 no 4-entry train FIFO: the pipeline takes one event per cycle
//      and never stalls, so a FIFO would never fill (T-5).
//   D3 a request held by back-pressure is not overwritten by a newer
//      one: the card's ahead pointer only advances when its request is
//      taken into the candidate register, so nothing is lost or stale.
//   D4 duplicate filter key = 12-bit page tag + 6-bit line (18 bits).
//
// COMBINATIONAL OUTPUTS (V-3.11): none, pf_* come from flops.
//
// TIMING (T-9, ESTIMATES, nothing synthesised yet): logic levels per
//   stage: S0 about 9 (page fold 3, 12-bit compare 4, victim pick 8 in
//   parallel), S1 about 12 (one-hot AND-OR read 5, +/-1 compare 3,
//   rule 3, write enable 1), S2 about 7, S3 about 9, S4 about 8.
//   Critical path: S1 (read entry, step rule, write enable).
//   GR-1 STATUS: NOT PROVEN. No measured slack exists. Stage count is
//   5 against a README target of 3 to 4: gap 1 to 2 cycles.
// ==========================================================================
`ifndef L1D_PF_STREAM_V
`define L1D_PF_STREAM_V
`default_nettype none

module l1d_pf_stream #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter PA_W        = 56,   // default 56 bits: physical address
    parameter LINE_BYTES  = 64,   // default 64 bytes: cache line
    parameter PAGE_BYTES  = 4096, // default 4096 bytes: page limit
    parameter STR_ENTRIES = 16,   // default 16: stream cards
    parameter TAG_W       = 12,   // default 12 bits: page tag
    parameter DEPTH       = 4,    // default 4 lines: lead
    parameter CONF_ON     = 2,    // default 2: confidence to start
    parameter CONF_MAX    = 3,    // default 3: confidence maximum
    parameter RECENT_N    = 4     // default 4: dedupe memory
) (
    // ---- clock and reset ----
    input  wire                                   clk,          // clock
    input  wire                                   rst,          // sync reset
    // ---- control ----
    input  wire                                   pf_enable,    // 1 = on
    // ---- training in (from the LSU and the cache) ----
    input  wire                                   tr_valid,     // event
    input  wire [PA_W-1:0]                        tr_addr,      // 56 bits
    input  wire                                   tr_is_store,  // store
    input  wire                                   tr_no_train,  // ignore
    input  wire                                   tr_miss,      // missed
    input  wire                                   tr_pf_hit,    // pf hit
    // ---- prefetch out (to the shared prefetch port) ----
    input  wire                                   pf_ready,     // accept
    output wire                                   pf_valid,     // request
    output wire [PA_W-$clog2(LINE_BYTES)-1:0]     pf_line_addr, // 50 bits
    output wire                                   pf_unique,    // 1=unique
    output wire                                   pf_flag       // prefetch
);

    // ---- local constants (value at the default) ----
    localparam OFFSET_W    = $clog2(LINE_BYTES);      // 6 bits
    localparam PAGE_W      = $clog2(PAGE_BYTES);      // 12 bits
    localparam LIP_W       = PAGE_W - OFFSET_W;       // 6 bits line in page
    localparam LIP1_W      = LIP_W + 1;               // 7 bits, with carry
    localparam PAGE_NUM_W  = PA_W - PAGE_W;           // 44 bits page number
    localparam CONF_W      = $clog2(CONF_MAX + 1);    // 2 bits
    localparam DEPTH_W     = $clog2(DEPTH + 1);       // 3 bits
    localparam WIDE_W      = LIP_W + DEPTH_W + 1;     // 10 bits, no wrap
    localparam KEY_W       = TAG_W + LIP_W;           // 18 bits dedupe key
    localparam DATA_W      = PAGE_NUM_W + LIP_W + 1;  // 51 bits queue word
    localparam [LIP_W-1:0]  LAST_LINE = {LIP_W{1'b1}};// 63: last line
    localparam [WIDE_W-1:0] DEPTH_WIDE = DEPTH;       // 4, widened
    localparam [WIDE_W-1:0] LAST_WIDE  = LAST_LINE;   // 63, widened
    localparam [CONF_W-1:0] CONF_ON_V  = CONF_ON;     // 2
    localparam [CONF_W-1:0] CONF_MAX_V = CONF_MAX;    // 3
    localparam [LIP1_W-1:0] ONE_LIP1   = {{LIP_W{1'b0}}, 1'b1}; // 7 bits

    // ---- stream table, one field per vector, entry i at [i*W +: W] ----
    reg  [STR_ENTRIES-1:0]            valid_q;  // 16: card in use
    reg  [STR_ENTRIES*TAG_W-1:0]      tag_q;    // 16 x 12: hashed page
    reg  [STR_ENTRIES*PAGE_NUM_W-1:0] page_q;   // 16 x 44: latest page
    reg  [STR_ENTRIES*LIP_W-1:0]      last_q;   // 16 x 6: latest line
    reg  [STR_ENTRIES-1:0]            dir_q;    // 16: 1 = up, 0 = down
    reg  [STR_ENTRIES*CONF_W-1:0]     conf_q;   // 16 x 2: confidence
    reg  [STR_ENTRIES*LIP_W-1:0]      ahead_q;  // 16 x 6: furthest asked
    reg  [STR_ENTRIES*LIP_W-1:0]      tgt_q;    // 16 x 6: line to reach
    reg  [STR_ENTRIES-1:0]            store_q;  // 16: store stream
    reg  [STR_ENTRIES-1:0]            pend_q;   // 16: lines still to ask

    // ---- S0 combinational (cycle of tr_valid) ----
    wire                    s0_event_ok;   // 1: event is trained
    wire [PAGE_NUM_W-1:0]   s0_page;       // 44: page number
    wire [LIP_W-1:0]        s0_line;       // 6: line in page
    wire [TAG_W-1:0]        s0_tag;        // 12: hashed page
    wire                    s0_trigger;    // 1: miss or prefetched hit
    wire [WIDE_W-1:0]       s0_line_wide;  // 10: line, widened
    wire [WIDE_W-1:0]       s0_up_sum;     // 10: line + DEPTH
    wire [WIDE_W-1:0]       s0_dn_diff;    // 10: line - DEPTH
    wire [LIP_W-1:0]        s0_tgt_up;     // 6: target going up
    wire [LIP_W-1:0]        s0_tgt_dn;     // 6: target going down
    reg  [STR_ENTRIES-1:0]  s0_hit_vec;    // 16: tag match, one-hot
    reg  [STR_ENTRIES-1:0]  s0_victim;     // 16: card to replace
    wire                    s0_same_prev;  // 1: same page as S1 event
    reg  [STR_ENTRIES-1:0]  victim_elig;   // 16: may be replaced
    reg  [STR_ENTRIES-1:0]  tier_mask;     // 16: cards of one tier
    reg  [STR_ENTRIES-1:0]  tier_pick;     // 16: best tier found
    reg                     victim_found;  // 1: first set bit seen

    // ---- S0 -> S1 registers (event being updated in S1) ----
    reg                     m_vld_q;       // 1: event valid
    reg  [TAG_W-1:0]        m_tag_q;       // 12: hashed page
    reg  [PAGE_NUM_W-1:0]   m_page_q;      // 44: page number
    reg  [LIP_W-1:0]        m_line_q;      // 6: line in page
    reg                     m_store_q;     // 1: store access
    reg                     m_miss_q;      // 1: access missed
    reg                     m_trig_q;      // 1: miss or prefetched hit
    reg  [LIP_W-1:0]        m_tgt_up_q;    // 6: target if going up
    reg  [LIP_W-1:0]        m_tgt_dn_q;    // 6: target if going down
    reg  [STR_ENTRIES-1:0]  m_hit_q;       // 16: tag match from S0
    reg                     m_same_prev_q; // 1: same page as event before
    reg  [STR_ENTRIES-1:0]  m_victim_q;    // 16: card to replace
    reg  [STR_ENTRIES-1:0]  p_alloc_q;     // 16: card the event before
                                           //     allocated (else 0)

    // ---- S1 combinational ----
    wire                    fwd_alloc;     // 1: use the card just made
    reg  [STR_ENTRIES-1:0]  hit_eff;       // 16: card to update
    wire                    hit_any;       // 1: some card to update
    wire                    alloc_new;     // 1: allocate a card
    reg  [LIP_W-1:0]        rd_last;       // 6: card's latest line
    reg                     rd_dir;        // 1: card's direction
    reg  [CONF_W-1:0]       rd_conf;       // 2: card's confidence
    reg  [LIP_W-1:0]        rd_ahead;      // 6: card's ahead pointer
    reg  [LIP_W-1:0]        rd_tgt;        // 6: card's target
    reg                     rd_pend;       // 1: card's pending bit
    wire [LIP1_W-1:0]       line_wide7;    // 7: new line, widened
    wire [LIP1_W-1:0]       last_wide7;    // 7: old line, widened
    wire                    delta_same;    // 1: same line again
    wire                    delta_up;      // 1: next line up
    wire                    delta_down;    // 1: next line down
    wire                    conf_unset;    // 1: confidence is zero
    wire                    step_up;       // 1: counts as a step up
    wire                    step_down;     // 1: counts as a step down
    wire                    step_ok;       // 1: a step in the direction
    wire                    flip_ok;       // 1: a step against it
    wire                    ahead_behind;  // 1: ahead pointer is behind
    reg  [CONF_W-1:0]       new_conf;      // 2: confidence after
    reg                     new_dir;       // 1: direction after
    reg  [LIP_W-1:0]        new_ahead;     // 6: ahead pointer after
    reg  [LIP_W-1:0]        new_tgt;       // 6: target after
    wire                    new_pend;      // 1: pending after

    // ---- S2 pick ----
    reg  [STR_ENTRIES-1:0]  rr_mask_q;     // 16: cards after the last one
    reg  [STR_ENTRIES-1:0]  sel_q;         // 16: picked card, one-hot
    reg                     sel_vld_q;     // 1: a card is picked
    reg  [STR_ENTRIES-1:0]  pick_high;     // 16: pending after last one
    reg  [STR_ENTRIES-1:0]  pick_base;     // 16: set to search
    reg  [STR_ENTRIES-1:0]  pick_one;      // 16: chosen, one-hot
    reg  [STR_ENTRIES-1:0]  pick_above;    // 16: cards after the chosen
    reg                     pick_seen;     // 1: running OR for loops
    wire                    allow_build;   // 1: S3 may take the pick

    // ---- S3 build ----
    reg  [LIP_W-1:0]        sel_ahead;     // 6: picked card's ahead
    reg  [LIP_W-1:0]        sel_tgt;       // 6: picked card's target
    reg                     sel_dir;       // 1: picked card's direction
    reg                     sel_pend;      // 1: picked card's pending
    reg                     sel_store;     // 1: picked card is a store
    reg  [PAGE_NUM_W-1:0]   sel_page;      // 44: picked card's page
    reg  [TAG_W-1:0]        sel_tag;       // 12: picked card's tag
    wire                    build_done;    // 1: ahead reached target
    wire [LIP_W-1:0]        build_next;    // 6: next line to ask
    wire                    build_issue;   // 1: a candidate is made
    wire                    build_finish;  // 1: card has nothing to do
    reg                     cand_vld_q;    // 1: candidate valid
    reg  [PAGE_NUM_W-1:0]   cand_page_q;   // 44: candidate page
    reg  [LIP_W-1:0]        cand_line_q;   // 6: candidate line
    reg                     cand_store_q;  // 1: candidate unique
    reg  [TAG_W-1:0]        cand_tag_q;    // 12: candidate page tag

    // ---- S4 filter and queue ----
    reg  [RECENT_N*KEY_W-1:0] recent_key_q; // 4 x 18: last requests
    reg  [RECENT_N-1:0]     recent_vld_q;  // 4: entry used
    reg                     cand_dup;      // 1: candidate was asked
    wire                    cand_push;     // 1: candidate enters queue
    wire                    queue_pop;     // 1: head taken by the cache
    reg                     q0_vld_q;      // 1: head valid (= pf_valid)
    reg  [DATA_W-1:0]       q0_data_q;     // 51: head {page,line,store}
    reg                     q1_vld_q;      // 1: spill valid
    reg  [DATA_W-1:0]       q1_data_q;     // 51: spill word

    integer entry_idx;                     // walks the table

    // ---- hash of the page number (XOR fold, 44 -> 12 bits) ----
    function [TAG_W-1:0] fold_page;
        input [PAGE_NUM_W-1:0] page_number;
        integer bit_idx;
        reg [TAG_W-1:0] folded;
        begin
            folded = {TAG_W{1'b0}};
            for (bit_idx = 0; bit_idx < PAGE_NUM_W; bit_idx = bit_idx + 1) begin
                folded[bit_idx % TAG_W] = folded[bit_idx % TAG_W]
                                          ^ page_number[bit_idx];
            end // for bit_idx
            fold_page = folded;
        end
    endfunction

    // ======================================================================
    // S0: filter, hash, compare with all cards, pick a victim card
    // ======================================================================
    assign s0_event_ok  = tr_valid & pf_enable & ~tr_no_train;
    assign s0_page      = tr_addr[PA_W-1:PAGE_W];
    assign s0_line      = tr_addr[PAGE_W-1:OFFSET_W];
    assign s0_tag       = fold_page(s0_page);
    assign s0_trigger   = tr_miss | tr_pf_hit;
    assign s0_line_wide = {{(WIDE_W-LIP_W){1'b0}}, s0_line};
    assign s0_up_sum    = s0_line_wide + DEPTH_WIDE;
    assign s0_dn_diff   = s0_line_wide - DEPTH_WIDE;
    // target going up stops at the last line of the page
    assign s0_tgt_up    = (s0_up_sum > LAST_WIDE) ? LAST_LINE
                                                  : s0_up_sum[LIP_W-1:0];
    // target going down stops at line 0
    assign s0_tgt_dn    = (s0_line_wide >= DEPTH_WIDE) ? s0_dn_diff[LIP_W-1:0]
                                                       : {LIP_W{1'b0}};
    // same page as the event now in S1 (its card change is not yet seen)
    assign s0_same_prev = m_vld_q & (m_tag_q == s0_tag);

    // tag compare against every card (one-hot, T-2: 12 bits only)
    always @* begin : s0_compare_proc
        integer entry_idx; // walks the cards
        for (entry_idx = 0; entry_idx < STR_ENTRIES; entry_idx = entry_idx + 1) begin
            s0_hit_vec[entry_idx] = valid_q[entry_idx]
                & (tag_q[entry_idx*TAG_W +: TAG_W] == s0_tag);
        end // for entry_idx
    end // always s0_compare_proc

    // victim: an unused card first, else the lowest confidence, ties to
    // the lowest index. The card the S1 event takes is never offered.
    always @* begin : s0_victim_proc
        integer tier_idx;  // walks the victim tiers
        integer entry_idx; // walks the cards
        tier_pick = {STR_ENTRIES{1'b0}};
        for (entry_idx = 0; entry_idx < STR_ENTRIES; entry_idx = entry_idx + 1) begin
            victim_elig[entry_idx] = ~(m_vld_q & m_victim_q[entry_idx]);
        end // for entry_idx
        // tier 0 = unused card; tier t = used card with confidence t-1.
        // Counting down means the lowest non-empty tier wins last.
        for (tier_idx = CONF_MAX + 1; tier_idx >= 0; tier_idx = tier_idx - 1) begin
            tier_mask = {STR_ENTRIES{1'b0}};
            for (entry_idx = 0; entry_idx < STR_ENTRIES; entry_idx = entry_idx + 1) begin
                if (tier_idx == 0) begin
                    tier_mask[entry_idx] = victim_elig[entry_idx]
                                           & ~valid_q[entry_idx];
                end
                else begin
                    tier_mask[entry_idx] = victim_elig[entry_idx]
                        & valid_q[entry_idx]
                        & ({{(32-CONF_W){1'b0}}, conf_q[entry_idx*CONF_W +: CONF_W]}
                           == (tier_idx - 1));
                end // else (tier_idx)
            end // for entry_idx
            if (|tier_mask) begin
                tier_pick = tier_mask;
            end
        end // for tier_idx
        // lowest index of the best tier, one-hot
        s0_victim    = {STR_ENTRIES{1'b0}};
        victim_found = 1'b0;
        for (entry_idx = 0; entry_idx < STR_ENTRIES; entry_idx = entry_idx + 1) begin
            if (tier_pick[entry_idx] && !victim_found) begin
                s0_victim[entry_idx] = 1'b1;
                victim_found         = 1'b1;
            end
        end // for entry_idx
    end // always s0_victim_proc

    // S0 -> S1 registers
    always @(posedge clk) begin : s0_register_proc
        if (rst) begin
            m_vld_q       <= 1'b0;
            m_same_prev_q <= 1'b0;
            m_tag_q       <= {TAG_W{1'b0}};
            m_hit_q       <= {STR_ENTRIES{1'b0}};
            m_victim_q    <= {STR_ENTRIES{1'b0}};
        end
        else begin
            m_vld_q       <= s0_event_ok;
            m_same_prev_q <= s0_same_prev;
            m_tag_q       <= s0_tag;
            m_hit_q       <= s0_hit_vec;
            m_victim_q    <= s0_victim;
        end // else (rst)
        // data flops need no reset: m_vld_q guards them (S-6)
        m_page_q   <= s0_page;
        m_line_q   <= s0_line;
        m_store_q  <= tr_is_store;
        m_miss_q   <= tr_miss;
        m_trig_q   <= s0_trigger;
        m_tgt_up_q <= s0_tgt_up;
        m_tgt_dn_q <= s0_tgt_dn;
    end // always s0_register_proc

    // ======================================================================
    // S1: read the card, apply the step rule, write the card back
    // ======================================================================
    // Hazard with the event before (it writes the table at the end of
    // this cycle's predecessor, so the table is already up to date):
    //  - it made a card for this page: S0 could not see it -> use it;
    //  - it took the card S0 matched -> that match is stale -> drop it.
    assign fwd_alloc = m_same_prev_q & (|p_alloc_q);
    always @* begin : s1_hit_proc
        if (fwd_alloc) begin
            hit_eff = p_alloc_q;
        end
        else begin
            hit_eff = m_hit_q & ~p_alloc_q;
        end // else (fwd_alloc)
    end // always s1_hit_proc
    assign hit_any   = |hit_eff;
    assign alloc_new = m_vld_q & ~hit_any & m_miss_q;

    // one-hot AND-OR read of the card
    always @* begin : s1_read_proc
        integer entry_idx; // walks the cards
        rd_last  = {LIP_W{1'b0}};
        rd_dir   = 1'b0;
        rd_conf  = {CONF_W{1'b0}};
        rd_ahead = {LIP_W{1'b0}};
        rd_tgt   = {LIP_W{1'b0}};
        rd_pend  = 1'b0;
        for (entry_idx = 0; entry_idx < STR_ENTRIES; entry_idx = entry_idx + 1) begin
            rd_last  = rd_last  | (last_q[entry_idx*LIP_W +: LIP_W]
                                   & {LIP_W{hit_eff[entry_idx]}});
            rd_dir   = rd_dir   | (dir_q[entry_idx] & hit_eff[entry_idx]);
            rd_conf  = rd_conf  | (conf_q[entry_idx*CONF_W +: CONF_W]
                                   & {CONF_W{hit_eff[entry_idx]}});
            rd_ahead = rd_ahead | (ahead_q[entry_idx*LIP_W +: LIP_W]
                                   & {LIP_W{hit_eff[entry_idx]}});
            rd_tgt   = rd_tgt   | (tgt_q[entry_idx*LIP_W +: LIP_W]
                                   & {LIP_W{hit_eff[entry_idx]}});
            rd_pend  = rd_pend  | (pend_q[entry_idx] & hit_eff[entry_idx]);
        end // for entry_idx
    end // always s1_read_proc

    // step rule (README section 5); 7-bit maths so line 63 + 1 and
    // line 0 - 1 never look like a step across the page edge
    assign line_wide7  = {1'b0, m_line_q};
    assign last_wide7  = {1'b0, rd_last};
    assign delta_same  = (m_line_q == rd_last);
    assign delta_up    = (line_wide7 == (last_wide7 + ONE_LIP1));
    assign delta_down  = (line_wide7 == (last_wide7 - ONE_LIP1));
    assign conf_unset  = (rd_conf == {CONF_W{1'b0}});
    assign step_up     = delta_up   & (conf_unset |  rd_dir);
    assign step_down   = delta_down & (conf_unset | ~rd_dir);
    assign step_ok     = step_up | step_down;
    assign flip_ok     = (delta_up | delta_down) & ~step_ok;
    // pointer already passed by the program (it ran ahead of us)
    assign ahead_behind = delta_up ? (rd_ahead < m_line_q)
                                   : (rd_ahead > m_line_q);

    always @* begin : s1_rule_proc
        if (delta_same) begin
            // same line again: card unchanged
            new_conf  = rd_conf;
            new_dir   = rd_dir;
            new_ahead = rd_ahead;
            new_tgt   = rd_tgt;
        end
        else if (step_ok) begin
            // step in the direction (or the first step): confidence + 1
            new_conf  = (rd_conf == CONF_MAX_V) ? rd_conf
                        : (rd_conf + {{(CONF_W-1){1'b0}}, 1'b1});
            new_dir   = delta_up;
            new_ahead = ((rd_conf < CONF_ON_V) || ahead_behind) ? m_line_q
                                                                : rd_ahead;
            new_tgt   = delta_up ? m_tgt_up_q : m_tgt_dn_q;
        end
        else if (flip_ok) begin
            // step against the direction: restart, direction flips
            new_conf  = {CONF_W{1'b0}};
            new_dir   = delta_up;
            new_ahead = m_line_q;
            new_tgt   = delta_up ? m_tgt_up_q : m_tgt_dn_q;
        end
        else begin
            // any other jump: restart in place, direction unset
            new_conf  = {CONF_W{1'b0}};
            new_dir   = 1'b0;
            new_ahead = m_line_q;
            new_tgt   = m_tgt_dn_q;
        end // else (jump)
    end // always s1_rule_proc
    // issue only on a miss or a prefetched hit, and only if confirmed
    assign new_pend = (new_conf >= CONF_ON_V) & (m_trig_q | rd_pend);

    // ======================================================================
    // S2: pick one card with lines still to ask (round robin)
    // ======================================================================
    // A card picked here may already be finished when S3 reads it; S3
    // then just clears its pending bit. No mask is needed (D3).
    assign allow_build = ~cand_vld_q | ~q1_vld_q;

    always @* begin : s2_pick_proc
        integer entry_idx; // walks the cards
        pick_high = pend_q & rr_mask_q;
        pick_base = (|pick_high) ? pick_high : pend_q;
        // lowest set bit of pick_base, one-hot
        pick_one  = {STR_ENTRIES{1'b0}};
        pick_seen = 1'b0;
        for (entry_idx = 0; entry_idx < STR_ENTRIES; entry_idx = entry_idx + 1) begin
            if (pick_base[entry_idx] && !pick_seen) begin
                pick_one[entry_idx] = 1'b1;
                pick_seen           = 1'b1;
            end
        end // for entry_idx
        // cards after the chosen one (next round-robin start)
        pick_above = {STR_ENTRIES{1'b0}};
        pick_seen  = 1'b0;
        for (entry_idx = 0; entry_idx < STR_ENTRIES; entry_idx = entry_idx + 1) begin
            pick_above[entry_idx] = pick_seen;
            pick_seen             = pick_seen | pick_one[entry_idx];
        end // for entry_idx
    end // always s2_pick_proc

    // ======================================================================
    // S3: build the next line of the picked card
    // ======================================================================
    always @* begin : s3_read_proc
        integer entry_idx; // walks the cards
        sel_ahead = {LIP_W{1'b0}};
        sel_tgt   = {LIP_W{1'b0}};
        sel_dir   = 1'b0;
        sel_pend  = 1'b0;
        sel_store = 1'b0;
        sel_page  = {PAGE_NUM_W{1'b0}};
        sel_tag   = {TAG_W{1'b0}};
        for (entry_idx = 0; entry_idx < STR_ENTRIES; entry_idx = entry_idx + 1) begin
            sel_ahead = sel_ahead | (ahead_q[entry_idx*LIP_W +: LIP_W]
                                     & {LIP_W{sel_q[entry_idx]}});
            sel_tgt   = sel_tgt   | (tgt_q[entry_idx*LIP_W +: LIP_W]
                                     & {LIP_W{sel_q[entry_idx]}});
            sel_dir   = sel_dir   | (dir_q[entry_idx] & sel_q[entry_idx]);
            sel_pend  = sel_pend  | (pend_q[entry_idx] & sel_q[entry_idx]);
            sel_store = sel_store | (store_q[entry_idx] & sel_q[entry_idx]);
            sel_page  = sel_page  | (page_q[entry_idx*PAGE_NUM_W +: PAGE_NUM_W]
                                     & {PAGE_NUM_W{sel_q[entry_idx]}});
            sel_tag   = sel_tag   | (tag_q[entry_idx*TAG_W +: TAG_W]
                                     & {TAG_W{sel_q[entry_idx]}});
        end // for entry_idx
    end // always s3_read_proc

    // done = the ahead pointer reached the target (target is already
    // clamped to the page, so the page edge stops the card here)
    assign build_done   = sel_dir ? (sel_ahead >= sel_tgt)
                                  : (sel_ahead <= sel_tgt);
    assign build_next   = sel_dir ? (sel_ahead + {{(LIP_W-1){1'b0}}, 1'b1})
                                  : (sel_ahead - {{(LIP_W-1){1'b0}}, 1'b1});
    assign build_issue  = sel_vld_q & allow_build & sel_pend & ~build_done;
    assign build_finish = sel_vld_q & allow_build & sel_pend & build_done;

    // ======================================================================
    // S4: duplicate filter and 2-entry output queue
    // ======================================================================
    always @* begin : s4_dup_proc
        integer recent_idx; // walks the dedupe list
        cand_dup = 1'b0;
        for (recent_idx = 0; recent_idx < RECENT_N; recent_idx = recent_idx + 1) begin
            if (recent_vld_q[recent_idx]
                && (recent_key_q[recent_idx*KEY_W +: KEY_W]
                    == {cand_tag_q, cand_line_q})) begin
                cand_dup = 1'b1;
            end
        end // for recent_idx
    end // always s4_dup_proc
    // a dropped duplicate leaves the candidate register too (allow_build)
    assign cand_push = cand_vld_q & ~cand_dup & ~q1_vld_q;
    assign queue_pop = q0_vld_q & pf_ready;

    // ======================================================================
    // clocked table, scan and queue state
    // ======================================================================
    always @(posedge clk) begin : table_proc
        if (rst || !pf_enable) begin
            // flush on reset and while the prefetcher is off
            pend_q     <= {STR_ENTRIES{1'b0}};
            sel_vld_q  <= 1'b0;
            sel_q      <= {STR_ENTRIES{1'b0}};
            cand_vld_q <= 1'b0;
            q0_vld_q   <= 1'b0;
            q1_vld_q   <= 1'b0;
            rr_mask_q  <= {STR_ENTRIES{1'b1}};
            recent_vld_q <= {RECENT_N{1'b0}};
            p_alloc_q  <= {STR_ENTRIES{1'b0}};
            if (rst) begin
                valid_q <= {STR_ENTRIES{1'b0}};
                tag_q   <= {(STR_ENTRIES*TAG_W){1'b0}};
                page_q  <= {(STR_ENTRIES*PAGE_NUM_W){1'b0}};
                last_q  <= {(STR_ENTRIES*LIP_W){1'b0}};
                dir_q   <= {STR_ENTRIES{1'b0}};
                conf_q  <= {(STR_ENTRIES*CONF_W){1'b0}};
                ahead_q <= {(STR_ENTRIES*LIP_W){1'b0}};
                tgt_q   <= {(STR_ENTRIES*LIP_W){1'b0}};
                store_q <= {STR_ENTRIES{1'b0}};
            end // if (rst)
        end
        else begin
            // ---- S2: pick (holds while S3 cannot take it) ----
            if (!sel_vld_q || allow_build) begin
                sel_vld_q <= |pend_q;
                sel_q     <= pick_one;
                if (|pend_q) begin
                    rr_mask_q <= pick_above;
                end
            end // if (pick)

            // ---- S3: candidate register ----
            if (allow_build) begin
                cand_vld_q   <= build_issue;
                cand_page_q  <= sel_page;
                cand_line_q  <= build_next;
                cand_store_q <= sel_store;
                cand_tag_q   <= sel_tag;
            end // if (allow_build)

            // ---- S4: remember the pushed line for the dedupe ----
            if (cand_push) begin
                recent_key_q <= {recent_key_q[(RECENT_N-1)*KEY_W-1:0],
                                 cand_tag_q, cand_line_q};
                recent_vld_q <= {recent_vld_q[RECENT_N-2:0], 1'b1};
            end // if (cand_push)

            // ---- S4: output queue, head is the pf_* register ----
            if (!q0_vld_q) begin
                if (cand_push) begin
                    q0_vld_q  <= 1'b1;
                    q0_data_q <= {cand_page_q, cand_line_q, cand_store_q};
                end
            end
            else if (queue_pop) begin
                if (q1_vld_q) begin
                    q0_data_q <= q1_data_q;
                    q1_vld_q  <= 1'b0;
                end
                else if (cand_push) begin
                    q0_data_q <= {cand_page_q, cand_line_q, cand_store_q};
                end
                else begin
                    q0_vld_q <= 1'b0;
                end // else (pop alone)
            end
            else if (cand_push) begin
                q1_vld_q  <= 1'b1;
                q1_data_q <= {cand_page_q, cand_line_q, cand_store_q};
            end // else if (push alone)

            // ---- table: scan side (S3 write), loses to S1 below ----
            for (entry_idx = 0; entry_idx < STR_ENTRIES; entry_idx = entry_idx + 1) begin
                if (sel_q[entry_idx] && build_issue) begin
                    ahead_q[entry_idx*LIP_W +: LIP_W] <= build_next;
                end
                if (sel_q[entry_idx] && build_finish) begin
                    pend_q[entry_idx] <= 1'b0;
                end
            end // for entry_idx

            // ---- table: S1 update or allocate ----
            p_alloc_q <= m_victim_q & {STR_ENTRIES{alloc_new}};
            for (entry_idx = 0; entry_idx < STR_ENTRIES; entry_idx = entry_idx + 1) begin
                if (m_vld_q && hit_eff[entry_idx]) begin
                    page_q[entry_idx*PAGE_NUM_W +: PAGE_NUM_W] <= m_page_q;
                    last_q[entry_idx*LIP_W +: LIP_W]           <= m_line_q;
                    dir_q[entry_idx]                           <= new_dir;
                    conf_q[entry_idx*CONF_W +: CONF_W]         <= new_conf;
                    ahead_q[entry_idx*LIP_W +: LIP_W]          <= new_ahead;
                    tgt_q[entry_idx*LIP_W +: LIP_W]            <= new_tgt;
                    pend_q[entry_idx]                          <= new_pend;
                end
                else if (alloc_new && m_victim_q[entry_idx]) begin
                    valid_q[entry_idx]                         <= 1'b1;
                    tag_q[entry_idx*TAG_W +: TAG_W]            <= m_tag_q;
                    page_q[entry_idx*PAGE_NUM_W +: PAGE_NUM_W] <= m_page_q;
                    last_q[entry_idx*LIP_W +: LIP_W]           <= m_line_q;
                    dir_q[entry_idx]                           <= 1'b0;
                    conf_q[entry_idx*CONF_W +: CONF_W]         <= {CONF_W{1'b0}};
                    ahead_q[entry_idx*LIP_W +: LIP_W]          <= m_line_q;
                    tgt_q[entry_idx*LIP_W +: LIP_W]            <= m_tgt_dn_q;
                    store_q[entry_idx]                         <= m_store_q;
                    pend_q[entry_idx]                          <= 1'b0;
                end // else if (alloc)
            end // for entry_idx
        end // else (run)
    end // always table_proc

    // ---- outputs (registered) ----
    wire [DATA_W-1:0] head_data; // 51: head word
    assign head_data    = q0_data_q;
    assign pf_valid     = q0_vld_q;
    assign pf_line_addr = head_data[DATA_W-1:1];  // {page, line}
    assign pf_unique    = head_data[0];
    assign pf_flag      = 1'b1;

`ifdef L1D_DEBUG
    // ---- watch-dogs (V-9.3), simulation only ----
    integer check_a;   // first card of a pair check
    integer check_b;   // second card of a pair check
    always @(posedge clk) begin : debug_proc
        if (!rst) begin
            if ((hit_eff & (hit_eff - 1'b1)) != {STR_ENTRIES{1'b0}}) begin
                $display("L1D_PF_STREAM ERROR: two cards updated at %0t", $time);
            end
            if (alloc_new && ((m_victim_q & (m_victim_q - 1'b1))
                              != {STR_ENTRIES{1'b0}} || m_victim_q == 0)) begin
                $display("L1D_PF_STREAM ERROR: victim not one-hot at %0t", $time);
            end
            for (check_a = 0; check_a < STR_ENTRIES; check_a = check_a + 1) begin
                for (check_b = check_a + 1; check_b < STR_ENTRIES; check_b = check_b + 1) begin
                    if (valid_q[check_a] && valid_q[check_b]
                        && tag_q[check_a*TAG_W +: TAG_W]
                           == tag_q[check_b*TAG_W +: TAG_W]) begin
                        $display("L1D_PF_STREAM ERROR: duplicate tag %0d %0d at %0t",
                                 check_a, check_b, $time);
                    end
                end // for check_b
            end // for check_a
        end // if (!rst)
    end // always debug_proc
`endif

endmodule

`default_nettype wire
`endif
