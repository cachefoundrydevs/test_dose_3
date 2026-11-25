// ============================================================================
// l1d_bank_s2.v  -  bank stage S2: the decision for each operation, the hit answer, the ECC decision and scrub, the
//                   prefetch decision, the training / eviction / feedback records, the snoop answer, the S2 -> S3 registers
// ============================================================================
// IN SIMPLE WORDS
//   S2 is where each operation learns its fate. A CPU request that hit picks its word with the registered hit way and
//   answers at once (s3_load = 1: the answer is pushed into the response merger in this cycle, see l1d_bank_s3). A store
//   hit merges its bytes into the old line and writes the whole line. A miss looks into the MSHR (line already coming?)
//   and the victim cache (line just evicted?) and either waits in its tracker entry, swaps the line back, or gets a new
//   MSHR entry. A snoop answers present / dirty / data, or waits for a line that is being installed or written back. A
//   refill installs its line; its victim goes into the victim cache. An AMO hit answers its OLD value here and the AMO
//   unit computes the NEW value, which is written one cycle later in S3 (the s3_* registers at the end of this module).
//
//   NEW IN cache_v6 (PLAN_v6.md sections 5 and 6)
//   ECC      S1 registered the syndromes of every way. S2 looks only at the HIT way (an AND-OR by s2_way): no error -> the
//            answer goes as before; a CORRECTABLE error -> no answer, no store write, the corrected line (8 x l1d_ecc_fix)
//            is written through the store write port NOW (s2_st_wr with s2_write_line) and the request is re-issued from
//            its tracker entry (act_code = ACT_WECC); an UNCORRECTABLE error -> the answer goes as before and the sticky
//            flag of the top is raised. A TAG error in any valid way of the set blocked the request in S1
//            (s2_tag_blocked): S2 writes the corrected tag of the first erroneous way (s2_tag_fix_wr, s2_write_way,
//            s2_write_tag) and re-issues the request (ACT_WECC).
//   PREFETCH a prefetch op (s2_req[REQ_PF_BIT]) never answers and has no tracker entry. Tags hit, MSHR hit or a line in the
//            victim cache -> dropped; else, with a free MSHR entry and room for a prefetch entry -> m_alloc as a prefetch
//            (no answer, unique if REQ_PFU_BIT); else dropped. Every outcome is reported to the prefetch unit (pf_fb_*).
//   TRAINING the first non-cancelled S2 pass of a tracked CPU request sends one training record (train_*) to the prefetch
//            unit: port, PC (from the tracker), byte address, store, miss, hit on a prefetched line, "do not train".
//   EVICTION a refill's victim, a snoop invalidate hit or a flush / clean line leaving the array is reported (evict_*).
//
//   s2_* registers (from l1d_bank_s1) --+--> decision (first match wins, see below) ------> act_*, m_alloc, n_alloc, vc_swap,
//   MSHR / victim-cache / NC lookups ---+                                                  s3_load, s2_pkt, pf_fb_* (regs)
//                                       +--> ECC (hit way) --> s2_st_wr / s2_tag_fix_wr, s2_write_line / _tag, ACT_WECC
//                                       +--> snoop decision --> snoop_response_* , snp_act_*, vc_snp
//                                       +--> AMO unit --> s3_* registers (to l1d_bank_s3)
//                                       +--> training record, eviction event (registered, to l1d_prefetch_unit)
//
// THE DECISION FOR A CPU REQUEST (the big always @* block; first match wins)
//   prefetch op .......... drop (feedback IN_CACHE / IN_FLIGHT), allocate a prefetch MSHR entry (ISSUED), or drop (DROPPED)
//   cancelled in R ....... ACT_NONE with s2_kill: the tracker puts the entry back to READY
//   tag ECC error ........ ACT_WECC: the tag of the first erroneous way is corrected and written; the entry is READY again
//   not cacheable ........ SC: answer "fail"; AMO: answer 0; everything else: NC queue (ACT_OWNED), or wait (ACT_WNC)
//   SC without reservation  answer "fail" (data word = 1)
//   tag HIT, data CE ..... ACT_WECC: the corrected line is written now; the entry is READY again (no answer)
//   tag HIT .............. answer from the cache line (s3_load); a UE raises the sticky flag and still answers
//   MSHR hit ............. the line is already coming: the entry waits for THAT MSHR entry (ACT_WMSHR)
//   victim-cache hit, VALID   plain load: answer from the slot and swap it back (if an MSHR entry is free);
//                             store class: swap back (ACT_WMSHR) or wait for any MSHR entry (ACT_WANY), then retry
//   victim-cache hit, WBPEND  plain load: answer from the slot; store class: wait for the B response (ACT_WVC)
//   victim-cache hit, MOVING  wait for any MSHR entry (ACT_WANY)
//   real miss ............ allocate an MSHR entry (ACT_OWNED), or all busy: ACT_WANY
//
// STAGE / CYCLE
//   Cycle c0+3 of a hit (the answer is valid at the pins at c0+4). Inputs: the S2 registers and the lookups of the miss
//   units (combinational in this cycle). Outputs: combinational decisions (used by the miss units, the response arbiter,
//   the SRAM write port and the tracker in this cycle), and registers: the S3 AMO registers, the training record, the
//   eviction event, the prefetch feedback, the two ECC pulses.
//   ESTIMATED LOGIC LEVELS (T-9, not measured): the answer path is unchanged from cache_v5 (hit word AND-OR 2 + decision
//   tree about 5 + response select 2); the decision tree grows by 2 levels (the CE gate: hit-way AND-OR of 4 flags = 2
//   levels, in parallel with the tree); the write-line path gains one AO22 level (refill line / corrected line / merged
//   store line select) before the 8 encoders of l1d_bank_r; the tag scrub path is a find-first over 4 flops (2) + 50-bit
//   AND-OR (2) + l1d_ecc_fix decode (3) + XOR (1) + encoder (7 XOR levels) = about 15 levels into the tag SRAM: this path
//   is OVER the budget by estimate (PLAN_v6 O-6 class); it is a rare path (a tag error) and is listed as a known gap.
//   TIMING OPT (S2-I1, 5 GHz study): the AMO unit runs once per way on the registered words (s2_words) and the hit way
//   picks the RESULT (late select); the data ECC decisions pick per-way pre-qualified flags with the one-hot hit way
//   (hit_way_onehot). Measured before: 502.5 ps into s3_new; expected after: about 335 ps (AMO unit bound, a known gap).
//   TIMING OPT (S2-I3): the AMO arithmetic is inline per way (one merged signed / unsigned compare, Kogge-Stone carry,
//   no XOR on a raw old / operand pin pair) and every arm is pre-qualified by the early one-hot way and op decode, so the
//   late compare result and carry enter one AND level and one OR level before s3_new (see "AMO: S2 computes").
//   Measured before: 369.8 ps; expected after: about 315 to 335 ps (estimate; still over the 200 ps goal, a known gap).
//   TIMING OPT (S2-I4): Ling pseudo carries (first stage g(i) | g(i-1), one cell less) for the ADD and the compare
//   trees, the hit way folded into the last ADD stage, the ADD sum chosen by the late pseudo carry between two early
//   arms, shared op-qualified operand nets (fewer operand pin loads). Measured before: 329.5 ps; expected after: about
//   295 to 300 ps, compare bound (estimate, not measured; still over the 200 ps goal, a known gap of about 95 ps).
//
// INSTANTIATED BY
//   l1d_bank, instance u_s2 (one per bank).
//
// PARAMETERS
//   `L1D_PARAMS   all cache parameters (l1d_defines.vh); used here: WAYS (4), LINE_BYTES (64), WRITE_DATA_WIDTH (64),
//                 MEM_SIZE_BITS (2), PC_W (64), HAS_ECC (1) and the widths of l1d_fields.vh
//   BANK_ID       default 0: number of this bank (debug messages only)
//
// PORTS (name, direction, width at the defaults, meaning)
//   clk, rst                          in   1      clock; synchronous reset, active high
//   s2_valid ... s2_slot              in   regs   the S2 registers (from l1d_bank_s1)
//   s2_tags                           in   200    the raw tags of all ways (tag scrub)
//   s2_raw_hit, s2_pf_hit             in   1/1    the tags hit (prefetch decision); a demand hit used a prefetched line
//   s2_tag_blocked                    in   1      the CPU request was blocked in S1 by a tag ECC error
//   s2_data_synd/_fix/_ce/_ue         in   224/32/4/4  data ECC of every way and granule (syndrome, correctable) / per way CE, UE
//   s2_tag_synd/_ce/_ue               in   24/4/4 tag ECC per way (syndrome; correctable and valid; uncorrectable and valid)
//   s2_seat_trained, s2_seat_pc       in   1/64   the entry in S2 sent its training record already; its PC (tracker)
//   q2_data                           in   512    the line of the MSHR entry being installed (refill write)
//   mshr_hit/_hid/_hasd/_can/_aid     in   1/3/1/1/3  MSHR lookup of s2_line: hit, entry, entry holds data, can allocate, new entry
//   prefetch_can_allocate             in   1      fewer than PF_MSHR_MAX prefetch entries are busy (flop of the MSHR)
//   vc_hit/_slot/_state/_data/_dirty  in   1/3/3/512/1  victim-cache lookup of s2_line
//   nc_can                            in   1      the non-cacheable queue can take a request
//   s2_refill, s2_maint               out  1      a refill installs / a flush-clean result (this cycle)
//   s2_st_wr, s2_write_line           out  1/512  a line is written in way s2_write_way now: merged store line or corrected line
//   s2_tag_fix_wr, s2_write_tag       out  1/50   a corrected tag is written in way s2_write_way; the tag written (refill or fix)
//   s2_write_way                      out  2      the way of the S2 write (the hit way, or the erroneous tag's way)
//   s2_uniq, s2_amo1, s2_alloc_is_write  out 1    the request needs E/M; is a one-request AMO; allocates as a write
//   s2_alloc_no_response              out  1      the MSHR fetches the line WITHOUT answering (AMO, vector or prefetch miss)
//   s2_alloc_is_prefetch              out  1      the MSHR allocation is a prefetch (counted against PF_MSHR_MAX)
//   s2_line_answer                    out  512    (C-20) the whole line a vector load answers with (cache way or victim cache)
//   vec_release_valid/_slot           out  1/1    (C-20) a vector store wrote its line: free its data place
//   line_q, st_merged                 out  512    the line of the way used in S1; the store bytes merged into it
//   act_valid/_code/_id               out  1/3/3  result for a CPU request that waits, re-issues or was handed to a miss unit
//   s3_load, s2_pkt                   out  1/74   the request answers now, and its response packet
//   m_alloc, m_swap, vc_swap, n_alloc out  1      allocate an MSHR entry (as a swap) / start a VC swap / queue as NC
//   ev_lr, ev_sc, ev_wr, ev_amor, ev_amow  out 1  events for the reservation unit
//   rsv_event_pulse                   out  1      an LR or SC completed here (to l1d_top, global reservation)
//   snoop_response_valid/_packet      out  1/572  the snoop answer (present, dirty, line address, line data)
//   snp_act_valid/_code/_id           out  1/2/3  result for a snoop that waits
//   vc_snp, vc_ins, vc_rel            out  1      snoop hit on a VC slot / insert a victim / release a MOVING slot
//   s3_amo_v ... s3_new               out  regs   the S3 registers of an AMO hit
//   s2_train_sent                     out  1      the training record of the entry in S2 goes now (to the tracker)
//   train_valid ... train_no_train    out  regs   the training record (1 / 2 / 64 / 64 / 1 / 1 / 1 / 1), valid one cycle
//   evict_valid, evict_line_addr      out  regs   a line left the array (1 / 58), valid one cycle
//   pf_fb_valid/_line_addr/_result    out  regs   the prefetch feedback (1 / 58 / 2, PF_FB_*), valid one cycle
//   ecc_correctable, ecc_uncorrectable  out regs  one-cycle pulses: a scrub was written / an uncorrectable error was seen
//
// WORKED EXAMPLE (load hit in way 2 with one flipped bit in granule 5)
//   S1: tag way 2 hits; the syndrome unit of way 2 granule 5 says "correctable". At the edge: s2_hit = 1, s2_way = 2,
//   s2_data_ce[2] = 1, s2_data_fix[2*8+5] = 1. S2: hit_data_ce = 1 -> no s3_load, act_valid = 1, act_code = ACT_WECC;
//   s2_st_wr = 1 with s2_write_line = line_q with granule 5 corrected (its 8 encoders in l1d_bank_r make new check bits);
//   ecc_correctable pulses next cycle. The tracker puts the entry back to READY; it is booked, issues, reads the corrected
//   line and answers (7 to 8 cycles after arrival instead of 4: the rare-case limit of DESIGN_RULES section 4).
//
// ASSUMPTIONS (V-9.2)
//   A1 a prefetch op is never issued fresh (s2_kill = 0 for it: it is always booked);  A2 at most one tag fix and one data
//   fix per cycle (one op in S2);  A3 the victim cache and the MSHR buffers hold lines without check bits (PLAN_v6 O-5).
//
// VERILOG WORDS USED IN THIS FILE
//   output reg x (in always @*)   a port driven by combinational logic written like a program (NOT a flip-flop)
//   output reg x (in always @(posedge clk))   a port that is a flip-flop
// ============================================================================
`ifndef L1D_BANK_S2_V
`define L1D_BANK_S2_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_bank_s2 #(
    `L1D_PARAMS,                                       // all cache parameters (l1d_defines.vh)
    parameter BANK_ID = 0                              // default 0: number of this bank (debug messages only)
) (
    clk, rst,
    s2_valid, s2_kind, s2_seat, s2_req, s2_mid, s2_line, s2_sinv,
    s2_hit, s2_scfail, s2_snp_hit, s2_hand, s2_way, s2_held, s2_swap, s2_kill,
    s2_words, s2_lines, s2_tags, s2_st_be, s2_st_din,
    s2_wbpend, s2_vvalid, s2_vdirty, s2_vshared, s2_vline, s2_slot,
    s2_raw_hit, s2_pf_hit, s2_tag_blocked,
    s2_data_synd, s2_data_fix, s2_data_ce, s2_data_ue,
    s2_tag_synd, s2_tag_ce, s2_tag_ue,
    s2_seat_trained, s2_seat_pc,
    q2_data,
    mshr_hit, mshr_hid, mshr_hasd, mshr_can, mshr_aid, prefetch_can_allocate,
    vc_hit, vc_slot, vc_state, vc_data, vc_dirty,
    nc_can,
    s2_refill, s2_maint, s2_st_wr, s2_tag_fix_wr, s2_write_way, s2_write_line, s2_write_tag,
    s2_uniq, s2_amo1, s2_alloc_is_write, s2_alloc_no_response, s2_alloc_is_prefetch,
    s2_line_answer, vec_release_valid, vec_release_slot,
    line_q, st_merged,
    act_valid, act_code, act_id,
    s3_load, s2_pkt,
    m_alloc, m_swap, vc_swap, n_alloc,
    ev_lr, ev_sc, ev_wr, ev_amor, ev_amow, rsv_event_pulse,
    snoop_response_valid, snoop_response_packet,
    snp_act_valid, snp_act_code, snp_act_id,
    vc_snp, vc_ins, vc_rel,
    s3_amo_v, s3_line, s3_way, s3_old, s3_be, s3_off, s3_new,
    s2_train_sent, train_valid, train_port, train_pc, train_addr, train_is_store, train_miss, train_pf_hit, train_no_train,
    evict_valid, evict_line_addr,
    pf_fb_valid, pf_fb_line_addr, pf_fb_result,
    ecc_correctable, ecc_uncorrectable
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // ---- local constants (value at the defaults) ----
    localparam [OFFSET_W-1:0] OFF0 = {OFFSET_W{1'b0}};         // zero line offset (debug messages print byte addresses), 6 bits
    localparam [WRITE_DATA_WIDTH-1:0] SC_FAIL_W = {{(WRITE_DATA_WIDTH-1){1'b0}}, SC_FAIL};   // SC result word: 1 = fail, 64 bits
    localparam SYND_W  = WAYS * GRANULES * DATA_HAM_W;         // all data syndromes, 4*8*7 = 224 bits
    localparam HSYND_W = GRANULES * DATA_HAM_W;                // the syndromes of one way, 8*7 = 56 bits
    localparam FIX_W   = WAYS * GRANULES;                      // "correctable" per way and granule, 32 bits
    localparam TSYND_W = WAYS * TAG_HAM_W;                     // all tag syndromes, 4*6 = 24 bits

    // ======================================================================
    // Ports
    // ======================================================================
    input  wire                        clk;                      // clock: all registers change on its rising edge
    input  wire                        rst;                      // synchronous reset, active high
    // ---- S2 registers (from l1d_bank_s1) ----
    input  wire                        s2_valid;                 // stage S2 holds an operation
    input  wire [1:0]                  s2_kind;                  // its kind (K_LSU, K_REFILL, K_MAINT, K_SNOOP)
    input  wire [SEAT_W-1:0]           s2_seat;                  // its tracker entry, SEAT_W = 3 bits
    input  wire [REQ_W-1:0]            s2_req;                   // its request packet, REQ_W = 148 bits
    input  wire [MSHR_ID_W-1:0]        s2_mid;                   // MSHR entry of the refill token (debug messages only), 3 bits
    input  wire [LINE_ADDR_W-1:0]      s2_line;                  // its line address, LINE_ADDR_W = 58 bits
    input  wire                        s2_sinv;                  // snoop: 1 = invalidate
    input  wire                        s2_hit;                   // the CPU request was a served cache hit in S1
    input  wire                        s2_scfail;                // an SC whose reservation check failed in S1
    input  wire                        s2_snp_hit;               // snoop: the tags hit
    input  wire                        s2_hand;                  // snoop: the line was M, its data is handed over
    input  wire [WAY_W-1:0]            s2_way;                   // the way used in S1, WAY_W = 2 bits
    input  wire                        s2_held;                  // the refill / maintenance token was held in S1
    input  wire                        s2_swap;                  // the refill is a swap back from the victim cache
    input  wire                        s2_kill;                  // the fresh request was cancelled in R (ghost)
    input  wire [WAYS*WRITE_DATA_WIDTH-1:0] s2_words;            // the load word of every way, WAYS*64 = 256 bits
    input  wire [WAYS*LINE_W-1:0]      s2_lines;                 // the lines of all ways, WAYS*LINE_W = 2048 bits
    input  wire [WAYS*TAG_W-1:0]       s2_tags;                  // the raw tags of all ways, WAYS*TAG_W = 200 bits
    input  wire [LINE_BYTES-1:0]       s2_st_be;                 // byte enables of a store hit, LINE_BYTES = 64 bits
    input  wire [LINE_W-1:0]           s2_st_din;                // store data at its byte position, LINE_W = 512 bits
    input  wire                        s2_wbpend;                // the victim is a flush/clean line
    input  wire                        s2_vvalid;                // a line is being evicted
    input  wire                        s2_vdirty;                // ... dirty (debug messages here; the victim cache takes it directly)
    input  wire                        s2_vshared;               // ... shared (debug messages here; the victim cache takes it directly)
    input  wire [LINE_ADDR_W-1:0]      s2_vline;                 // ... its line address, 58 bits
    input  wire [VC_ID_W-1:0]          s2_slot;                  // victim-cache slot that takes it (debug messages here), 3 bits
    input  wire                        s2_raw_hit;               // the tags hit (cacheable CPU op or prefetch, no state check)
    input  wire                        s2_pf_hit;                // a demand hit used a prefetched line (its first use)
    input  wire                        s2_tag_blocked;           // the CPU request was blocked in S1 by a tag ECC error
    input  wire [SYND_W-1:0]           s2_data_synd;             // data syndromes, way w granule g at [(w*8+g)*7 +: 7], 224 bits
    input  wire [FIX_W-1:0]            s2_data_fix;              // correctable per way and granule, 32 bits
    input  wire [WAYS-1:0]             s2_data_ce;               // some granule of the way is correctable, 4 bits
    input  wire [WAYS-1:0]             s2_data_ue;               // some granule of the way is uncorrectable, 4 bits
    input  wire [TSYND_W-1:0]          s2_tag_synd;              // tag syndromes, way w at [w*6 +: 6], 24 bits
    input  wire [WAYS-1:0]             s2_tag_ce;                // tag correctable AND the way is valid, 4 bits
    input  wire [WAYS-1:0]             s2_tag_ue;                // tag uncorrectable AND the way is valid, 4 bits
    input  wire                        s2_seat_trained;          // the entry in S2 already sent its training record (tracker)
    input  wire [PC_W-1:0]             s2_seat_pc;               // its PC, PC_W = 64 bits (tracker)
    input  wire [LINE_W-1:0]           q2_data;                  // the line of the MSHR entry being installed, 512 bits
    // ---- lookups of the miss units for s2_line ----
    input  wire                        mshr_hit;                 // an MSHR entry holds this line
    input  wire [MSHR_ID_W-1:0]        mshr_hid;                 // ... that entry, MSHR_ID_W = 3 bits
    input  wire                        mshr_hasd;                // ... and it already holds the line data
    input  wire                        mshr_can;                 // a free MSHR entry exists
    input  wire [MSHR_ID_W-1:0]        mshr_aid;                 // ... the entry an allocation takes, 3 bits
    input  wire                        prefetch_can_allocate;    // a prefetch may take an entry (fewer than PF_MSHR_MAX busy)
    input  wire                        vc_hit;                   // a victim-cache slot holds this line
    input  wire [VC_ID_W-1:0]          vc_slot;                  // ... that slot, VC_ID_W = 3 bits
    input  wire [2:0]                  vc_state;                 // ... its state (VC_VALID / VC_WBPEND / VC_MOVING)
    input  wire [LINE_W-1:0]           vc_data;                  // ... its line data, 512 bits
    input  wire                        vc_dirty;                 // ... the line is dirty
    input  wire                        nc_can;                   // the non-cacheable queue can take a request
    // ---- S2 results (combinational, this cycle) ----
    output wire                        s2_refill;                // the refill really installs (a held token does not)
    output wire                        s2_maint;                 // a flush/clean token is in S2 (result to l1d_maint)
    output wire                        s2_st_wr;                 // a line is written now: a store hit's merged line or a corrected line
    output wire                        s2_tag_fix_wr;            // a corrected tag is written now (tag scrub)
    output wire [WAY_W-1:0]            s2_write_way;             // the way of the S2 write: the hit way, or the erroneous tag's way, 2 bits
    output wire [LINE_W-1:0]           s2_write_line;            // the line S2 writes: refill line, merged store line or corrected line, 512 bits
    output wire [TAG_W-1:0]            s2_write_tag;             // the tag S2 writes: the refill's tag, or the corrected tag, 50 bits
    output wire                        s2_uniq;                  // the request needs the line in E / M (store, atomic, unique prefetch)
    output wire                        s2_amo1;                  // the request is a one-request AMO
    output wire                        s2_alloc_is_write;        // an MSHR allocation is a write (an AMO miss is fetched like a read)
    output wire                        s2_alloc_no_response;     // the MSHR fetches without answering (AMO, vector or prefetch miss)
    output wire                        s2_alloc_is_prefetch;     // the MSHR allocation is a prefetch
    output wire [LINE_W-1:0]           s2_line_answer;           // C-20: the line a vector load answers with, LINE_W = 512 bits
    output wire                        vec_release_valid;        // C-20: a vector store wrote its line: free its data place
    output wire [VEC_SLOT_W-1:0]       vec_release_slot;         // C-20: that place, VEC_SLOT_W = 1 bit
    output reg  [LINE_W-1:0]           line_q;                   // the line of the way used in S1 (victim / snoop data), 512 bits
    output reg  [LINE_W-1:0]           st_merged;                // the store bytes merged into that line, 512 bits
    output reg                         act_valid;                // the CPU request waits, re-issues or was handed to a miss unit
    output reg  [2:0]                  act_code;                 // ACT_NONE / WMSHR / WANY / WVC / WNC / OWNED / WECC
    output reg  [WID_W-1:0]            act_id;                   // MSHR entry or victim slot it waits for, WID_W = 3 bits
    output reg                         s3_load;                  // the CPU request answers now (pushed this cycle)
    output wire [RESP_W-1:0]           s2_pkt;                   // its response packet, RESP_W = 74 bits
    output reg                         m_alloc;                  // allocate an MSHR entry
    output reg                         m_swap;                   // ... as a swap from the victim cache (else a fetch)
    output reg                         vc_swap;                  // start the swap on victim-cache slot vc_slot
    output reg                         n_alloc;                  // queue the request in the non-cacheable unit
    output wire                        ev_lr;                    // an LR completed (reservation set)
    output wire                        ev_sc;                    // an SC completed
    output wire                        ev_wr;                    // a write to a cacheable line completed
    output wire                        ev_amor;                  // AMO read half completed (always 0 in this version)
    output wire                        ev_amow;                  // AMO write half completed (always 0 in this version)
    output wire                        rsv_event_pulse;          // an LR or SC completed here (to l1d_top)
    output wire                        snoop_response_valid;     // push a snoop answer
    output wire [SNPR_W-1:0]           snoop_response_packet;    // the answer packet, SNPR_W = 572 bits
    output reg                         snp_act_valid;            // the snoop in S2 has a result
    output reg  [1:0]                  snp_act_code;             // SACT_DONE / SACT_WMSHR / SACT_WVC
    output reg  [WID_W-1:0]            snp_act_id;               // MSHR entry or victim slot it waits for, 3 bits
    output reg                         vc_snp;                   // the snoop acts on a VALID victim-cache slot
    output wire                        vc_ins;                   // insert a victim (refill victim or flush/clean line)
    output wire                        vc_rel;                   // release a MOVING slot (swap installed into an empty way)
    // ---- S2 -> S3 registers (an AMO hit writes its line in S3) ----
    output reg                         s3_amo_v;                 // S3 holds an AMO hit
    output reg  [LINE_ADDR_W-1:0]      s3_line;                  // its line address, 58 bits
    output reg  [WAY_W-1:0]            s3_way;                   // its way, 2 bits
    output reg  [LINE_W-1:0]           s3_old;                   // the old line (read at its grant), 512 bits
    output reg  [LINE_BYTES-1:0]       s3_be;                    // bytes of the AMO word, 64 bits
    output reg  [OFFSET_W-1:0]         s3_off;                   // byte offset of the AMO word, OFFSET_W = 6 bits
    output reg  [WRITE_DATA_WIDTH-1:0] s3_new;                   // the AMO unit's result, aligned to bit 0, 64 bits
    // ---- training record (to l1d_prefetch_unit through l1d_bank and l1d_top; registered, valid one cycle) ----
    output wire                        s2_train_sent;            // the record of the entry in S2 goes now (the tracker marks it)
    output reg                         train_valid;              // a record this cycle
    output reg  [PORT_W-1:0]           train_port;               // the request port, PORT_W = 2 bits
    output reg  [PC_W-1:0]             train_pc;                 // the instruction PC, 64 bits
    output reg  [ADDR_W-1:0]           train_addr;               // the byte address, ADDR_W = 64 bits
    output reg                         train_is_store;           // store class
    output reg                         train_miss;               // the access missed (no served hit)
    output reg                         train_pf_hit;             // the access hit a prefetched line (first use)
    output reg                         train_no_train;           // atomic, non-cacheable or not the LSU: ignore the record
    // ---- eviction event (registered, valid one cycle) ----
    output reg                         evict_valid;              // a line left the array this cycle
    output reg  [LINE_ADDR_W-1:0]      evict_line_addr;          // its line address, 58 bits
    // ---- prefetch feedback (registered, valid one cycle) ----
    output reg                         pf_fb_valid;              // a prefetch op finished in S2
    output reg  [LINE_ADDR_W-1:0]      pf_fb_line_addr;          // its line address, 58 bits
    output reg  [1:0]                  pf_fb_result;             // PF_FB_ISSUED / IN_CACHE / IN_FLIGHT / DROPPED
    // ---- ECC pulses (registered, one cycle each) ----
    output reg                         ecc_correctable;          // a corrected line or tag was written (scrub)
    output reg                         ecc_uncorrectable;        // an uncorrectable error was seen (data of the hit way, or a tag)

    // ======================================================================
    // Fields of the operation in S2
    // ======================================================================
    wire               s2_lsu;                                   // the operation in S2 is a CPU request or a prefetch
    wire               s2_pf;                                    // the request packet is a prefetch op
    wire               s2_pfu;                                   // ... that wants the line unique
    wire               s2_pf_op;                                 // a prefetch op is in S2
    wire               s2_cpu;                                   // a tracked CPU request is in S2 (not a prefetch)
    wire               s2_live;                                  // ... and it was not cancelled in R
    wire               s2_line_op;                               // C-20: a whole-line VPU access (vector load or store)
    wire               s2_cache;                                 // cacheable
    wire               s2_store;                                 // store class
    wire               s2_amo;                                   // atomic (LR / SC / AMO)
    wire               s2_seq;                                   // with amo: 1 = AMO, 0 = LR / SC
    wire               s2_is_lr;                                 // an LR
    wire               s2_is_sc;                                 // an SC
    wire [ADDR_W-1:0]  s2_addr;                                  // byte address, 64 bits
    wire [OFFSET_W-1:0]         s2_off;                          // byte offset in the line, 6 bits
    wire [MEM_SIZE_BITS-1:0]    s2_size;                         // access size, 2 bits
    wire [MASTER_ID_BITS-1:0]   s2_mstid;                        // master id, 2 bits
    wire [IDX_BITS-1:0]         s2_idx;                          // ROB index, 3 bits
    wire [PORT_W-1:0]           s2_port;                         // request port, PORT_W = 2 bits (goes into the answer)
    wire [WRITE_DATA_WIDTH-1:0] s2_wdata;                        // store data / AMO operand, 64 bits
    assign s2_lsu     = s2_valid && (s2_kind == K_LSU);
    assign s2_pf      = s2_req[REQ_PF_BIT];
    assign s2_pfu     = s2_req[REQ_PFU_BIT];
    assign s2_pf_op   = s2_lsu && s2_pf;
    assign s2_cpu     = s2_lsu && !s2_pf;
    assign s2_live    = s2_cpu && !s2_kill;
    assign s2_line_op = s2_req[REQ_LINE_BIT];
    assign s2_cache   = s2_req[REQ_CACHE_BIT];
    assign s2_store   = (s2_req[REQ_RDWR_BIT] == OP_WRITE);
    assign s2_amo     = s2_req[REQ_AMO_BIT];
    assign s2_seq     = s2_req[REQ_SEQ_BIT];
    assign s2_is_lr   = s2_amo && !s2_seq && !s2_store;
    assign s2_is_sc   = s2_amo && !s2_seq &&  s2_store;
    assign s2_amo1    = s2_amo && s2_seq;                      // a one-request AMO (computed in S2, written in S3)
    assign s2_addr    = s2_req[REQ_ADDR_LSB +: ADDR_W];
    assign s2_off     = s2_addr[OFFSET_W-1:0];
    assign s2_size    = s2_req[REQ_SIZE_LSB +: MEM_SIZE_BITS];
    assign s2_mstid   = s2_req[REQ_MID_LSB +: MASTER_ID_BITS];
    assign s2_idx     = s2_req[REQ_IDX_LSB +: IDX_BITS];
    assign s2_port    = s2_req[REQ_PORT_LSB +: PORT_W];
    assign s2_wdata   = s2_req[REQ_DATA_LSB +: WRITE_DATA_WIDTH];
    // the fetch of a miss: unique for a store / atomic / unique prefetch; an AMO, vector or prefetch miss is fetched like a read
    // (no operand merge in the MSHR) and not answered by the MSHR (the AMO / vector op retries, the prefetch has nobody to answer)
    assign s2_uniq               = s2_pf ? s2_pfu : (s2_store || s2_amo);
    assign s2_alloc_is_write     = s2_req[REQ_RDWR_BIT] && !s2_amo1 && !s2_line_op;
    assign s2_alloc_no_response  = s2_amo1 || s2_line_op || s2_pf;
    assign s2_alloc_is_prefetch  = s2_pf;

    // the operation in S2 is a refill that really installs (a held token does not)
    wire s2_snp;                                                 // a snoop is in S2
    assign s2_refill = s2_valid && (s2_kind == K_REFILL) && !s2_held;
    assign s2_maint  = s2_valid && (s2_kind == K_MAINT);
    assign s2_snp    = s2_valid && (s2_kind == K_SNOOP);

    // cache_v2_bank_pipe: the line of the way used in S1 (hit way, victim way or maintenance way), from the registered lines:
    // the victim for the victim cache, the snoop answer data and the line the ECC scrub corrects
    integer lw;                                                  // loop variable: way
    always @* begin
        line_q = {LINE_W{1'b0}};
        for (lw = 0; lw < WAYS; lw = lw + 1)
            line_q = line_q | ({LINE_W{s2_way == lw}} & s2_lines[lw*LINE_W +: LINE_W]);
    end

    // ---- cache_v2_bank_pipe: STORE READ-MODIFY-WRITE (user decision 2026-11-03: no byte-write-enable memory, for ASIC).
    //      The store read its line at the grant like every request; in S2 its bytes are merged into the hit way's old line
    //      (line_q, from registers) and the WHOLE line is written. The old line is current: a same-set store is ordered by the
    //      tracker (one issued fresh in the write cycle is cancelled in R, wl_hit), and snoops / refills / flush tokens are not
    //      booked while a store is in R, S1 or S2. ----
    integer mb;                                                  // loop variable: byte of the line
    always @* begin
        for (mb = 0; mb < LINE_BYTES; mb = mb + 1)
            st_merged[mb*8 +: 8] = s2_st_be[mb] ? s2_st_din[mb*8 +: 8] : line_q[mb*8 +: 8];
    end

    // ======================================================================
    // TIMING OPT (S2-I1): the hit way as a one-hot, decoded once from the S2 register s2_way
    // ======================================================================
    // s2_way is the latest and most loaded signal of S2 (it selects 512-bit lines, syndromes and words; post-synthesis its
    // buffer tree alone is about 169 ps). The paths rebuilt below (AMO result select, ECC flags) take it as the LAST
    // input: everything that does not need the way is computed per way first, then one AND-OR by this one-hot picks it.
    // Exactly one bit is set when s2_way names a way (always at WAYS = 4); all zero for a code >= WAYS (WAYS not a power of 2).
    wire [WAYS-1:0] hit_way_onehot;                              // bit w = (s2_way == w), WAYS = 4 bits
    generate
        genvar hit_way_idx;                                      // loop variable: way
        for (hit_way_idx = 0; hit_way_idx < WAYS; hit_way_idx = hit_way_idx + 1) begin : g_hit_way_onehot
            assign hit_way_onehot[hit_way_idx] = (s2_way == hit_way_idx);   // decode of one way, 1 bit
        end
    endgenerate

    // ======================================================================
    // ECC of the HIT way (data) and of the first erroneous way (tag): flags, syndromes, corrected words
    // ======================================================================
    reg  [HSYND_W-1:0] hit_synd;                                 // the hit way's syndromes, 56 bits
    reg  [GRANULES-1:0] hit_fix;                                 // the hit way's correctable granules, 8 bits
    reg                hit_data_ce;                              // the hit way has a correctable granule
    reg                hit_data_ue;                              // the hit way has an uncorrectable granule
    integer ew;                                                  // loop variable: way
    always @* begin
        hit_synd    = {HSYND_W{1'b0}};
        hit_fix     = {GRANULES{1'b0}};
        hit_data_ce = 1'b0;
        hit_data_ue = 1'b0;
        for (ew = 0; ew < WAYS; ew = ew + 1) begin
            hit_synd    = hit_synd    | ({HSYND_W{s2_way == ew}}  & s2_data_synd[ew*HSYND_W +: HSYND_W]);
            hit_fix     = hit_fix     | ({GRANULES{s2_way == ew}} & s2_data_fix[ew*GRANULES +: GRANULES]);
            hit_data_ce = hit_data_ce | ((s2_way == ew) & s2_data_ce[ew]);
            hit_data_ue = hit_data_ue | ((s2_way == ew) & s2_data_ue[ew]);
        end
    end
    // the first way with a correctable tag error (find-first over 4 flops), its tag and its syndrome
    reg  [WAYS-1:0]    tag_fix_onehot;                           // one-hot (or zero), 4 bits
    reg                tag_fix_seen;                             // a lower way already has the error
    reg  [WAY_W-1:0]   tag_fix_way;                              // its number, 2 bits
    reg  [TAG_W-1:0]   tag_err_tag;                              // its raw tag, 50 bits
    reg  [TAG_HAM_W-1:0] tag_err_synd;                           // its syndrome, 6 bits
    wire               tag_fix_valid;                            // some valid way has a correctable tag error
    wire               tag_ue_any;                               // some valid way has an uncorrectable tag error
    integer tw;                                                  // loop variable: way
    always @* begin
        tag_fix_seen = 1'b0;
        tag_fix_way  = {WAY_W{1'b0}};
        tag_err_tag  = {TAG_W{1'b0}};
        tag_err_synd = {TAG_HAM_W{1'b0}};
        for (tw = 0; tw < WAYS; tw = tw + 1) begin
            tag_fix_onehot[tw] = s2_tag_ce[tw] & ~tag_fix_seen;
            tag_fix_seen       = tag_fix_seen | s2_tag_ce[tw];
            tag_fix_way        = tag_fix_way  | ({WAY_W{tag_fix_onehot[tw]}} & tw[WAY_W-1:0]);
            tag_err_tag        = tag_err_tag  | ({TAG_W{tag_fix_onehot[tw]}} & s2_tags[tw*TAG_W +: TAG_W]);
            tag_err_synd       = tag_err_synd | ({TAG_HAM_W{tag_fix_onehot[tw]}} & s2_tag_synd[tw*TAG_HAM_W +: TAG_HAM_W]);
        end
    end
    assign tag_fix_valid = |s2_tag_ce;
    assign tag_ue_any    = |s2_tag_ue;
    // the ECC decisions of this cycle (a tracked, cacheable, served hit; or the tag block of S1)
    wire ecc_data_fix;                                           // data CE in the hit way: scrub, no answer, re-issue
    wire ecc_data_ue;                                            // data UE in the hit way: answer as today, sticky flag
    wire ecc_tag_blk;                                            // the request was blocked by a tag error: re-issue
    wire ecc_tag_fix_wr;                                         // ... and a correctable tag is written now
    wire ecc_tag_ue;                                             // ... and only uncorrectable tag errors exist (sticky flag)
    // TIMING OPT (S2-I1): the two data ECC decisions with the hit way LAST. Before: the way AND-OR of the CE / UE flags
    // (hit_data_ce, hit_data_ue), then the AND with the request qualifiers and the NOT of UE: way tree + 2 AO levels + AND.
    // Now: the qualifiers and "CE and not UE" are formed per way from S2 registers only (early), and the one-hot hit way
    // picks the result in one AND-OR (AO22 + OR2 at WAYS = 4), saving about 1 to 2 gate levels (about 20 to 30 ps) on the
    // ecc_correctable / ecc_uncorrectable / s3_amo_v flops and on the decision (s3_load, act_*) and the write select.
    // Same function: for the selected way k, (q & ce[k] & !ue[k]) = q & hit_data_ce & !hit_data_ue; no way selected -> 0 both.
    wire            live_cacheable_hit;                          // a live, cacheable, served hit is in S2 (early, 1 bit)
    wire [WAYS-1:0] way_data_fix_flag;                           // per way: that hit would be scrubbed (CE, no UE), WAYS = 4 bits
    wire [WAYS-1:0] way_data_ue_flag;                            // per way: that hit has an uncorrectable granule, WAYS = 4 bits
    assign live_cacheable_hit = s2_live && s2_cache && s2_hit;
    assign way_data_fix_flag  = {WAYS{live_cacheable_hit}} & s2_data_ce & ~s2_data_ue;   // WAYS = 4 bits
    assign way_data_ue_flag   = {WAYS{live_cacheable_hit}} & s2_data_ue;                 // WAYS = 4 bits
    // old form (kept for reference): ecc_data_ue = s2_live && s2_cache && s2_hit && hit_data_ue;
    //                                ecc_data_fix = s2_live && s2_cache && s2_hit && hit_data_ce && !hit_data_ue;
    assign ecc_data_ue    = |(hit_way_onehot & way_data_ue_flag);
    assign ecc_data_fix   = |(hit_way_onehot & way_data_fix_flag);
    assign ecc_tag_blk    = s2_live && s2_tag_blocked;
    assign ecc_tag_fix_wr = ecc_tag_blk && tag_fix_valid;
    assign ecc_tag_ue     = ecc_tag_blk && !tag_fix_valid && tag_ue_any;
    // the corrected line (8 x l1d_ecc_fix on the hit way's line) and the corrected tag
    wire [LINE_W-1:0] fixed_line;                                // line_q with every correctable granule put right, 512 bits
    wire [TAG_W-1:0]  fixed_tag;                                 // the erroneous way's tag put right, 50 bits
    generate
        if (HAS_ECC != 0) begin : g_ecc_fix
            genvar fg;                                           // loop variable: granule
            for (fg = 0; fg < GRANULES; fg = fg + 1) begin : g_fix_gran
                l1d_ecc_fix #(.DATA_W(GRAN_BITS), .HAM_W(DATA_HAM_W)) u_fix (
                    .data       (line_q[fg*GRAN_BITS +: GRAN_BITS]),
                    .syndrome   (hit_synd[fg*DATA_HAM_W +: DATA_HAM_W]),
                    .fix_enable (hit_fix[fg]),
                    .fixed      (fixed_line[fg*GRAN_BITS +: GRAN_BITS])
                );
            end
            l1d_ecc_fix #(.DATA_W(TAG_W), .HAM_W(TAG_HAM_W)) u_tag_fix (
                .data       (tag_err_tag),
                .syndrome   (tag_err_synd),
                .fix_enable (tag_fix_valid),
                .fixed      (fixed_tag)
            );
        end else begin : g_no_ecc_fix
            assign fixed_line = line_q;
            assign fixed_tag  = tag_err_tag;
        end
    endgenerate

    // ======================================================================
    // Load data: from the cache line and from a victim-cache line
    // ======================================================================
    // ---- load data: pick the requested bytes out of the cache line (hit_data) and out of a victim-cache line (vcl_data) ----
    wire [WRITE_DATA_WIDTH-1:0] vcl_data;                 // the requested bytes of the victim-cache line, 64 bits
    // cache_v2_bank_pipe: the hit word = the registered word of the hit way (cut per way in S1): one AND-OR, no shifter here
    reg  [WRITE_DATA_WIDTH-1:0] hit_data;                 // WRITE_DATA_WIDTH = 64 bits
    integer hw;                                           // loop variable: way
    always @* begin
        hit_data = {WRITE_DATA_WIDTH{1'b0}};
        for (hw = 0; hw < WAYS; hw = hw + 1)
            hit_data = hit_data | ({WRITE_DATA_WIDTH{s2_way == hw}} & s2_words[hw*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH]);
    end
    l1d_resp_gen #(.IN_BYTES(LINE_BYTES), .WRITE_DATA_WIDTH(WRITE_DATA_WIDTH),
                   .MEM_SIZE_BITS(MEM_SIZE_BITS), .OFF_W(OFFSET_W)) u_vc_data (
        .source_block (vc_data), .byte_offset (s2_off), .access_size (s2_size), .load_data (vcl_data)
    );

    // ======================================================================
    // Stage 2 decision for a CPU request or a prefetch op
    // ======================================================================
    reg  s3_from_vc;                                      // the answer takes victim-cache data
    reg  s3_scfail;                                       // the answer is a failed SC: data word = 1
    reg  s3_zero;                                         // the answer data word is 0 (non-cacheable AMO)
    reg  [1:0] pf_result;                                 // the prefetch outcome (PF_FB_*)
    wire vc_line_present;                                 // the victim cache holds a usable copy (VALID or WBPEND)
    assign vc_line_present = vc_hit && ((vc_state == VC_VALID) || (vc_state == VC_WBPEND));

    // The decision tree (see the header, first match wins). Defaults first (no response, no allocation, no action), then the branches.
    always @* begin
        s3_load    = 1'b0;
        s3_from_vc = 1'b0;
        s3_scfail  = 1'b0;
        s3_zero    = 1'b0;
        m_alloc    = 1'b0;
        m_swap     = 1'b0;
        vc_swap    = 1'b0;
        n_alloc    = 1'b0;
        act_valid  = 1'b0;
        act_code   = ACT_NONE;
        act_id     = {WID_W{1'b0}};
        pf_result  = PF_FB_DROPPED;
        if (s2_pf_op) begin
            // cache_v6: a prefetch op. Never an answer, never a tracker action, never a reservation event. The line is in the
            // cache or already on its way -> dropped with the reason; else a prefetch MSHR entry fetches it (at most
            // PF_MSHR_MAX per bank), else dropped (no room).
            if (s2_raw_hit || vc_line_present) begin
                pf_result = PF_FB_IN_CACHE;
            end else if (mshr_hit) begin
                pf_result = PF_FB_IN_FLIGHT;
            end else if (mshr_can && prefetch_can_allocate) begin
                m_alloc   = 1'b1;
                pf_result = PF_FB_ISSUED;
            end else begin
                pf_result = PF_FB_DROPPED;
            end
        end else if (s2_lsu && s2_kill) begin
            // a fresh request cancelled in R (deps, or a write to its set at its read edge): nothing changed; its entry goes
            // back to READY (ACT_NONE with s2_kill, read by the tracker) and is booked again by the normal rules (rule F-0)
            act_valid = 1'b1;
            act_code  = ACT_NONE;
        end else if (s2_lsu) begin
            if (s2_tag_blocked) begin
                // cache_v6: a tag ECC error in a valid way of the set made the hit / miss decision unreliable: S1 blocked the
                // request; the tag is corrected and written now (ecc_tag_fix_wr) and the request is re-issued from its entry
                act_valid = 1'b1;
                act_code  = ACT_WECC;
            end else if (!s2_cache) begin
                if (s2_is_sc) begin
                    s3_load   = 1'b1;                              // non-cacheable SC: fail, no access
                    s3_scfail = 1'b1;
                end else if (s2_amo1) begin
                    s3_load = 1'b1;                                // cache_v2_bank_pipe: non-cacheable AMO: answer 0, no access
                    s3_zero = 1'b1;
                end else begin
                    // non-cacheable (LR / AMO halves too, with no reservation or lock)
                    act_valid = 1'b1;
                    if (nc_can) begin
                        n_alloc  = 1'b1;
                        act_code = ACT_OWNED;
                    end else begin
                        act_code = ACT_WNC;
                    end
                end
            end else if (s2_scfail) begin
                s3_load   = 1'b1;                                  // SC without a valid reservation: fail
                s3_scfail = 1'b1;
            end else if (s2_hit && ecc_data_fix) begin
                // cache_v6: a correctable error in the hit way's line: no answer and no store write; the corrected line is
                // written now (s2_st_wr with fixed_line) and the request is re-issued from its entry (it then reads the clean line)
                act_valid = 1'b1;
                act_code  = ACT_WECC;
            end else if (s2_hit) begin
                s3_load = 1'b1;                                    // (1) hit: load data or store ack (a UE answers too: sticky flag)
            end else if (mshr_hit) begin
                act_valid = 1'b1;                                  // line already coming: wait for it
                act_code  = ACT_WMSHR;
                act_id    = mshr_hid;
            end else if (vc_hit && vc_state == VC_VALID) begin
                if (!s2_uniq) begin
                    s3_load    = 1'b1;                             // (3) victim-cache hit answers the load
                    s3_from_vc = 1'b1;
                    if (mshr_can) begin                            // and the line is swapped back
                        m_alloc = 1'b1;
                        m_swap  = 1'b1;
                        vc_swap = 1'b1;
                    end
                end else begin
                    act_valid = 1'b1;
                    if (mshr_can) begin                            // store class: swap back, then retry
                        m_alloc  = 1'b1;
                        m_swap   = 1'b1;
                        vc_swap  = 1'b1;
                        act_code = ACT_WMSHR;
                        act_id   = mshr_aid;
                    end else begin
                        act_code = ACT_WANY;
                    end
                end
            end else if (vc_hit && vc_state == VC_WBPEND) begin
                if (!s2_uniq) begin
                    s3_load    = 1'b1;                             // load answered from the slot
                    s3_from_vc = 1'b1;
                end else begin
                    act_valid = 1'b1;                              // store class: wait for the B response,
                    act_code  = ACT_WVC;                           // then fetch from memory
                    act_id    = vc_slot;
                end
            end else if (vc_hit) begin
                act_valid = 1'b1;                                  // MOVING slot: its MSHR entry will free
                act_code  = ACT_WANY;
            end else begin
                act_valid = 1'b1;                                  // real miss (or upgrade): fetch
                if (mshr_can) begin
                    m_alloc  = 1'b1;
                    if (s2_amo1 || s2_line_op) begin
                        // cache_v2_bank_pipe: an AMO miss is fetched WITHOUT an answer (no_response); the AMO waits for that entry,
                        // is retried after the install and is then computed in the pipeline as a hit.
                        // cache_v5 C-20: a vector load / store miss does the same (the MSHR answers only 64-bit words): after the
                        // install the retry hits; a vector store then writes its whole line (CC-15: fetch, then overwrite)
                        act_code = ACT_WMSHR;
                        act_id   = mshr_aid;
                    end else begin
                        act_code = ACT_OWNED;
                    end
                end else begin
                    act_code = ACT_WANY;
                end
            end
        end
    end

    // ======================================================================
    // The S2 write port: a store hit's merged line, the ECC-corrected line, the refill line; the tag (refill or fix)
    // ======================================================================
    wire store_hit_wr;                                           // a served store-class hit (not an AMO, not scrubbed) writes its line
    assign store_hit_wr  = s2_live && s2_hit && s2_store && !s2_amo1 && !ecc_data_fix;
    assign s2_st_wr      = store_hit_wr || ecc_data_fix;
    assign s2_tag_fix_wr = ecc_tag_fix_wr;
    assign s2_write_way  = ecc_tag_fix_wr ? tag_fix_way : s2_way;
    // one AO level: the three sources never coincide (a refill is its own operation kind; a scrub excludes the store write)
    assign s2_write_line = ({LINE_W{s2_refill}}     & q2_data)    |
                           ({LINE_W{ecc_data_fix}}  & fixed_line) |
                           ({LINE_W{store_hit_wr}}  & st_merged);
    assign s2_write_tag  = ecc_tag_fix_wr ? fixed_tag : s2_line[LA_TAG_LSB +: TAG_W];

    // ---- response packet built in S2: | rd_wr | size | master_id | idx | data |. The data word: 1 for a failed SC, 0 for a store ack,
    //      else the loaded bytes (from the victim-cache line or the cache line). ----
    // cache_v2_bank_pipe: an AMO answers its OLD value (hit_data) as load data (rd_wr = 0 in the answer); a non-cacheable AMO
    // (not supported: the PMA check should fault it) answers 0 and changes nothing.
    wire [WRITE_DATA_WIDTH-1:0] s2_rdata;                        // the data word of the answer, 64 bits
    assign s2_rdata = s3_scfail ? SC_FAIL_W :
                      s3_zero   ? {WRITE_DATA_WIDTH{1'b0}} :
                      (s2_store && !s2_amo1) ? {WRITE_DATA_WIDTH{1'b0}} :
                      (s3_from_vc ? vcl_data : hit_data);
    assign s2_pkt = {s2_port, (s2_req[REQ_RDWR_BIT] && !s2_amo1), s2_size, s2_mstid, s2_idx, s2_rdata};

    // ---- cache_v5 C-20: the whole line a vector load answers with: the hit way's line (line_q, the 4:1 way select already
    //      used for victims and snoops) or the victim-cache line. It leaves the bank beside s2_pkt (lsu_response_line_data)
    //      and is kept only by the VPU port's response merger. A vector store answers a plain write acknowledge. ----
    assign s2_line_answer    = s3_from_vc ? vc_data : line_q;
    // a vector store answers only after its line was written (hit in E / M, s2_st_wr): then its data place is free again
    assign vec_release_valid = s2_cpu && s3_load && s2_line_op && s2_store;
    assign vec_release_slot  = s2_req[REQ_DATA_LSB +: VEC_SLOT_W];

    // ---- completion of the CPU request in S2: answered, or handed to an MSHR / NC entry. A pass that ends in "wait and retry" changes
    //      no reservation or lock. These events go to the reservation unit. A prefetch op never completes anything here. ----
    wire s2_done_lsu;                                            // the tracked CPU request completes in S2
    assign s2_done_lsu = s2_cpu && (s3_load || (act_valid && act_code == ACT_OWNED));
    assign ev_lr   = s2_done_lsu && s2_cache && s2_is_lr;
    assign ev_sc   = s2_done_lsu && s2_is_sc;            // an SC completed (cacheable or not: a non-cacheable SC also ends the LR/SC pair)
    // A2: tell the top that an LR or SC finished here, so the other banks drop their reservation (one reservation per hart).
    assign rsv_event_pulse = ev_lr || ev_sc;
    assign ev_wr   = s2_done_lsu && s2_cache && s2_store && !s2_is_sc;
    // cache_v2_bank_pipe: the AMO is one request computed in the cache; there is no AMO lock any more (no lock set or released).
    // An AMO is store class (rd_wr = 1), so its completion is an ev_wr for the reservation unit (a write to the line).
    assign ev_amor = 1'b0;
    assign ev_amow = 1'b0;

    // ======================================================================
    // AMO: S2 computes, S3 writes
    // ======================================================================
    // ---- An AMO hit (line E or M) reads its line at the grant like every request; S1 cuts the old word (s2_words) and makes the line
    //      M; S2 answers the OLD value and the AMO unit computes the new one; S3 merges the new value into the old line and writes the
    //      whole line on the write port. No lock: same-set requests are ordered by the tracker (an AMO is store class) and a fresh
    //      request that read the line before the S3 write is cancelled in R (amo_hit / wl_hit). A CE in the hit way takes the scrub
    //      path instead (no AMO write, the AMO re-issues). ----
    wire [WRITE_DATA_WIDTH-1:0] amo_new;                  // new = f(old, operand), aligned to bit 0, 64 bits
    // TIMING OPT (S2-I1): the way select moves from IN FRONT OF the AMO unit to BEHIND it.
    // Measured before (post-synthesis, ASAP7, 200 ps clock): s2_way[1] -> buffer tree 169 ps -> hit_data way AND-OR
    // (AO21 + OA21, 39 ps) -> l1d_amo_alu (adder / compare / op select, 294 ps) -> s3_new D at 502.5 ps (slack -322.9).
    // The AMO unit's inputs other than the old word (op, size, operand) are early S2 registers, and the old word of every
    // way is already cut per way in S1 (s2_words, 64 bits per way). So the AMO arithmetic runs once per way on
    // f(word of way w, operand) straight from the S2 registers, in parallel with the s2_way buffer tree, and the one-hot
    // hit way picks the result. Measured after: 369.8 ps (kept).
    // Same function: old amo_new = f(hit_data) with hit_data = word of way s2_way (0 for a code >= WAYS);
    // new amo_new = f(word of the selected way), or f(0, operand) for a code >= WAYS (the extra unit below, only when
    // WAYS is not a power of 2; none at WAYS = 4).
    //
    // TIMING OPT (S2-I3): the AMO arithmetic is written INLINE per way (no l1d_amo_alu instance at WAYS = 4) and every
    // late signal enters exactly ONE AND level before the final OR.
    // Measured before (S2-I1, post-synthesis): 369.8 ps, path s2_req[43] (operand) -> INV (fan-out 10, 28 ps) -> HAxp5
    // (32 ps: Yosys extract_fa paired the compare leaf ~old | operand with the old ^ operand of the ADD / XOR arms) ->
    // compare tree (AND4 ... AO221, 7 cells) -> lt at 249 ps -> fan-out buffers 39 ps -> ALU output AND-OR (AO21) -> OR4
    // -> way AND-OR (AO32) -> s3_new D at 350.6 ps (+ setup and uncertainty = 369.8 ps).
    // Changes (all combined, one cycle, no new flop):
    //   (1) no 2-input XOR on (old bit, operand bit) is left anywhere: old ^ operand is formed from the QUALIFIED inputs
    //       ((q & old) ^ (q & operand), q = way hit & (ADD or XOR)), so extract_fa finds no half adder to share with the
    //       compare leaves and the Kogge-Stone leaves (old & operand, old | operand); the compare leaves are single gates
    //       ~old & operand and ~old | operand (the inversion on the per-way old word only, the operand pin in true
    //       polarity, its ideal pin drive feeds all four ways). Expected gain: the INV + HA head (60 ps) becomes about
    //       20 to 25 ps, about 35 ps.
    //   (2) ONE compare per way instead of two (signed and unsigned): the sign flip (signed MIN / MAX) and the access
    //       width (is64) are folded into early terms around two 31-bit balanced trees (bits 62:32 and 30:0, 5 levels each,
    //       the same depth as the iteration-6 32-bit trees), so the compare result is one AO221 after the trees:
    //         64-bit: lt = g63' | p63' & G[62:32] | p63' & P[62:32] & (g31 | p31 & G[30:0])
    //         32-bit: lt = g31' | p31' & G[30:0]
    //       (g / p primed = bit 63 or 31 with the sign flip for MIN / MAX: g = old & ~operand, p = old | ~operand).
    //       lt is only read by the MIN / MAX / MINU / MAXU arms, so a signed compare for MIN / MAX and an unsigned one for
    //       MINU / MAXU gives the same result bit for bit; for every other code the arms are zero.
    //       Each bit now has 2 compare AND terms per way (lt & pick, ~lt & pick) instead of 4 (lt_s and lt_u).
    //   (3) the ADD sum as (sum for carry 0) & ~carry | (sum for carry 1) & carry, with the hit way's carry picked first
    //       (the carry tree is about 100 ps earlier than the compare): one AO22 per bit on the late carry, not XOR then AND.
    //   (4) all arms are pre-qualified with the early one-hot hit way and op decode (way_min, way_max, way_and, way_or,
    //       way_logic): per bit the final logic is 8 compare terms + 2 ADD terms + 1 early term, one AND level (AO22 /
    //       AO222) and one OR level (OR4), instead of AO21 -> OR4 -> AO32 (3 levels, 62 ps). Expected gain 10 to 20 ps.
    // Expected after: lt about 210 ps, buffers about 39 ps, final 2 levels about 45 ps -> about 295 ps at the D pin,
    // about 315 to 335 ps with setup and uncertainty (estimate, not measured; ABC may restructure).
    // Same function: per way w the arms are those of l1d_amo_alu (iteration 6) ANDed with (s2_way == w); exactly one way
    // is selected at WAYS = 4, so the OR over the ways is the old AND-OR of the per-way results. SWAP / any other code
    // gives the operand for every way and for the no-way case alike, so its arm needs no way qualifier.
    // Written for WRITE_DATA_WIDTH = 64 (bit numbers 63 / 62:32 / 31 / 30:0 and 6 carry stages), like l1d_amo_alu.
    // TIMING OPT (S2-I4): Ling pseudo carries (one leaf level less) for the ADD and the compare, the hit way folded into
    // the last carry stage, the ADD sum chosen by the pseudo carry between two early arms, fewer operand pin loads.
    // Measured before (S2-I3, post-synthesis): 329.5 ps. Worst path s2_req[2] (operand bit 2) -> BUFx2 + BUFx3 (operand
    // fan-out, 39 ps) -> AO21, AO221, MAJx2, OA221, OA31, AO21, AO221 (the Kogge-Stone carry, 7 cells, 162 ps; the MAJ is
    // Yosys / ABC seeing g | p & g' with g = old & operand, p = old | operand as a full-adder carry) -> AND2, AND3, OR5
    // (the hit way's carry pick and the ADD qualifier, 62 ps) -> XOR2 (ABC merges ~c & x | c & ~x into c ^ x, because
    // amo_add_hit_way equals amo_op_add at WAYS = 4) -> AO221 -> s3_new[53] D at 304.2 ps. Second family (-121 ps):
    // s2_words (old word) -> BUF + INV -> compare tree (9 cells) -> lt at 237 ps -> BUFx6f -> AO221 -> OR4 -> s3_new.
    // Changes (all combined, one cycle, no new flop):
    //   (1) Ling pseudo carry instead of the carry, for the ADD and for the compare. With OR-propagate t = a | b and
    //       g = a & b (g implies t) the carry out of bits i..0 is C(i) = t(i) & H(i), where H(i) is the prefix of the pairs
    //       (g(i), t(i-1)): the propagate of every bit is SHIFTED by one. The first stage then simplifies:
    //         H over bits (i, i-1) = g(i) | t(i-1) & g(i-1) = g(i) | g(i-1)     (one AO22 straight from the pins)
    //         propagate over (i, i-1) = t(i-1) & t(i-2)                          (one OA22 straight from the pins)
    //       instead of g(i) | p(i) & g(i-1) = MAJ(a(i), b(i), g(i-1)) behind g(i-1) = a & b (2 cells). One cell less on the
    //       whole tree, and no 3-input cut is a full-adder carry any more (no MAJ chain).
    //       ADD: 5 stages (2, 4, 8, 16, 32 bits) per way, the 6th (64 bits) folded with the way (2). The sum needs no
    //       AND with t(i-1) behind the tree: sum(i) = x(i) ^ (t(i-1) & H(i-1)) = H(i-1) ? (x(i) ^ t(i-1)) : x(i), and both
    //       arms are early (x = old ^ operand), so the late H(i-1) meets one AO22-type level.
    //       Compare: the two 31-bit trees (bits 62:32, 30:0) are Ling trees of 16 pair leaves (4 levels behind the pair
    //       leaf instead of 5 behind the bit leaf); the t of bit 63 (or 31) that C = t & H needs is early and goes into
    //       the early factors:
    //         64-bit: lt = g63' | (t63' & t62) & U | t63' & P[62:32] & g31 | (t63' & P[62:30]) & L
    //         32-bit: lt = g31' | (t31' & t30) & L
    //       (U = H over bits 62:32, L = H over bits 30:0, primed = with the sign flip of MIN / MAX, P = AND of the t).
    //   (2) the hit way is folded into the last ADD stage: per way onehot & X32 | onehot & Y32 & X32(-32) (AO32 type),
    //       and the OR over the ways, instead of the 6th stage AO21 then AND2 / AND3 / OR5. About 2 cells less.
    //   (3) the ADD sum: carry 1 arm = way-qualified x ^ (t << 1), carry 0 arm = way-qualified x, both early; the last
    //       level is the pseudo carry choosing between them (pc & arm1 | ~pc & arm0) inside the final AND-OR.
    //   (4) operand pin loads (fan-out) cut from about 29 to about 20 per bit: old ^ operand is formed per way from
    //       (op & old) ^ (op & operand) with ONE shared op & operand net (amo_logic_operand) instead of four way-qualified
    //       copies (the way is applied after the XOR, still no XOR on a raw pin pair); the MIN / MAX arms use two shared
    //       nets op & operand (amo_min_operand, amo_max_operand) instead of eight way-qualified ones; the AND / OR arms
    //       reuse the ADD leaves old & operand and old | operand.
    // Expected after (estimate, not measured; ABC may restructure): ADD about 39 + 20 + 4 x 22 + 2 x 23 + 22 + 22 =
    // about 237 ps at the D pin (about 260 to 280 ps with setup and uncertainty); compare one leaf level less (about
    // 22 ps) and the operand side unchanged, about 295 to 300 ps: the compare (s2_words -> lt -> s3_new) is the next
    // family. Still over the 200 ps goal: a known gap (GR-1), about 95 to 100 ps.
    // Same function: (1) H is an exact rewrite of the carry (C = t & H because g implies t); (2), (3), (4) only move the
    // way and op qualifiers: exactly one way is selected at WAYS = 4 and every per-way arm is zero for the other ways;
    // for a code >= WAYS (WAYS not a power of 2) every per-way arm is zero and amo_new_no_way gives f(0, operand), as
    // before. The upper bits of a 32-bit ADD / AND / OR / XOR / SWAP and the MIN / MAX pick of whole 64-bit words are
    // those of l1d_amo_alu (iteration 6), bit for bit.
    // ---- early decodes (S2 register bits only) ----
    wire [AMOOP_W-1:0] amo_op_code;                        // the AMO operation, AMOOP_W = 4 bits
    wire               amo_size_is64;                      // a 64-bit access (size 3); else the compare uses 32 bits
    wire               amo_op_add;                         // AMO_ADD
    wire               amo_op_xor;                         // AMO_XOR
    wire               amo_op_and;                         // AMO_AND
    wire               amo_op_or;                          // AMO_OR
    wire               amo_op_min_any;                     // AMO_MIN or AMO_MINU: new = the smaller value
    wire               amo_op_max_any;                     // AMO_MAX or AMO_MAXU: new = the larger value
    wire               amo_op_signed;                      // AMO_MIN or AMO_MAX: the compare is signed
    wire               amo_op_swap_other;                  // AMO_SWAP and every code that is none of the 8 others: new = operand
    wire               amo_op_logic;                       // TIMING OPT (S2-I4): AMO_ADD or AMO_XOR (both need old ^ operand)
    wire [WRITE_DATA_WIDTH-1:0] amo_logic_operand;        // TIMING OPT (S2-I4): operand if ADD or XOR, else 0; shared by the ways, 64 bits
    wire [WRITE_DATA_WIDTH-1:0] amo_min_operand;          // TIMING OPT (S2-I4): operand if MIN or MINU, else 0; shared by the ways, 64 bits
    wire [WRITE_DATA_WIDTH-1:0] amo_max_operand;          // TIMING OPT (S2-I4): operand if MAX or MAXU, else 0; shared by the ways, 64 bits
    assign amo_op_code       = s2_req[REQ_AMOOP_LSB +: AMOOP_W];                           // AMOOP_W = 4 bits
    assign amo_size_is64     = (s2_size == 2'd3);
    assign amo_op_add        = (amo_op_code == AMO_ADD);
    assign amo_op_xor        = (amo_op_code == AMO_XOR);
    assign amo_op_and        = (amo_op_code == AMO_AND);
    assign amo_op_or         = (amo_op_code == AMO_OR);
    assign amo_op_min_any    = (amo_op_code == AMO_MIN) | (amo_op_code == AMO_MINU);
    assign amo_op_max_any    = (amo_op_code == AMO_MAX) | (amo_op_code == AMO_MAXU);
    assign amo_op_signed     = (amo_op_code == AMO_MIN) | (amo_op_code == AMO_MAX);
    assign amo_op_swap_other = ~(amo_op_add | amo_op_xor | amo_op_and | amo_op_or | amo_op_min_any | amo_op_max_any);
    assign amo_op_logic      = amo_op_add | amo_op_xor;
    assign amo_logic_operand = {WRITE_DATA_WIDTH{amo_op_logic}}   & s2_wdata;                // 64 bits
    assign amo_min_operand   = {WRITE_DATA_WIDTH{amo_op_min_any}} & s2_wdata;                // 64 bits
    assign amo_max_operand   = {WRITE_DATA_WIDTH{amo_op_max_any}} & s2_wdata;                // 64 bits
    // ---- per way: compare, pseudo-carry tree and the pre-qualified arms ----
    wire [WAYS*WRITE_DATA_WIDTH-1:0] amo_new_way;         // way w: its compare and AND / OR arms, already qualified by way and op, at [w*64 +: 64], 256 bits
    wire [WAYS*WRITE_DATA_WIDTH-1:0] amo_xor_way;         // way w: (old ^ operand) qualified by way & (ADD or XOR), 256 bits
    wire [WAYS*WRITE_DATA_WIDTH-1:0] amo_sum_if_carry_way; // TIMING OPT (S2-I4): way w: the ADD sum bit if the carry into it is 1, (old ^ operand) ^ (t << 1), qualified by the way (by ADD in the final), 256 bits
    wire [WAYS*WRITE_DATA_WIDTH-1:0] amo_carry_way;       // way w: the Ling pseudo carry of the bits below every bit (H(i-1)), qualified by the way, 256 bits
    wire [WRITE_DATA_WIDTH-1:0]      amo_new_no_way;      // f(0, operand): the result when s2_way names no way, 64 bits
    wire                             hit_way_none;        // s2_way names no way (a code >= WAYS; constant 0 at WAYS = 4)
    generate
        genvar amo_way_idx;                               // loop variable: way
        for (amo_way_idx = 0; amo_way_idx < WAYS; amo_way_idx = amo_way_idx + 1) begin : g_amo_way
            wire [WRITE_DATA_WIDTH-1:0] old_word;         // the registered old word of this way (no way select in front), 64 bits
            wire way_and;                                 // this way hits and the op is AND
            wire way_or;                                  // this way hits and the op is OR
            wire way_min;                                 // this way hits and the op is MIN or MINU
            wire way_max;                                 // this way hits and the op is MAX or MAXU
            assign old_word  = s2_words[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];   // 64 bits
            assign way_and   = hit_way_onehot[amo_way_idx] & amo_op_and;
            assign way_or    = hit_way_onehot[amo_way_idx] & amo_op_or;
            assign way_min   = hit_way_onehot[amo_way_idx] & amo_op_min_any;
            assign way_max   = hit_way_onehot[amo_way_idx] & amo_op_max_any;
            // ---- compare old < operand: carry of operand + ~old, unsigned leaves g = ~old & operand, p = ~old | operand ----
            wire [WRITE_DATA_WIDTH-1:0] cmp_gen;          // per bit: old 0, operand 1 (this bit decides "less"), 64 bits
            wire [WRITE_DATA_WIDTH-1:0] cmp_prop;         // per bit: old 0 or operand 1 (a lower decision passes), 64 bits
            wire cmp_gen_top;                             // bit 63 leaf g with the sign flip of MIN / MAX (old 1, operand 0)
            wire cmp_prop_top;                            // bit 63 leaf p with the sign flip of MIN / MAX
            wire cmp_gen_half;                            // bit 31 leaf g with the sign flip (the sign bit of a 32-bit access)
            wire cmp_prop_half;                           // bit 31 leaf p with the sign flip
            wire cmp_upper_gen;                           // TIMING OPT (S2-I4): Ling U = H over bits 62:32 (pair leaves g(i) | g(i-1), 4 levels)
            wire cmp_upper_prop;                          // P[62:32]: bits 62:32 are all "pass"
            wire cmp_lower_gen;                           // TIMING OPT (S2-I4): Ling L = H over bits 30:0 (pair leaves g(i) | g(i-1), 4 levels)
            wire cmp_early_gen;                           // the early terms of lt that need no tree output
            wire cmp_early_upper;                         // the early factor of U in lt
            wire cmp_early_lower;                         // the early factor of L in lt
            wire old_less;                                // old < operand (signed for MIN / MAX, unsigned else; 32 or 64 bits)
            assign cmp_gen         = ~old_word & s2_wdata;                                   // 64 bits
            assign cmp_prop        = ~old_word | s2_wdata;                                   // 64 bits
            assign cmp_gen_top     = (amo_op_signed & old_word[63] & ~s2_wdata[63]) | (~amo_op_signed & cmp_gen[63]);
            assign cmp_prop_top    = (amo_op_signed & (old_word[63] | ~s2_wdata[63])) | (~amo_op_signed & cmp_prop[63]);
            assign cmp_gen_half    = (amo_op_signed & old_word[31] & ~s2_wdata[31]) | (~amo_op_signed & cmp_gen[31]);
            assign cmp_prop_half   = (amo_op_signed & (old_word[31] | ~s2_wdata[31])) | (~amo_op_signed & cmp_prop[31]);
            // Ling trees: H[hi:lo] = H[hi:mid] | (t(hi-1) & ... & t(mid-1)) & H[mid-1:lo], pair leaf H[i:i-1] = g(i) | g(i-1)
            assign cmp_upper_gen   = ((((cmp_gen[62] | cmp_prop[61] & (cmp_gen[61] | cmp_gen[60])) | &cmp_prop[61:59] & ((cmp_gen[59] | cmp_gen[58]) | &cmp_prop[58:57] & (cmp_gen[57] | cmp_gen[56]))) | &cmp_prop[61:55] & (((cmp_gen[55] | cmp_gen[54]) | &cmp_prop[54:53] & (cmp_gen[53] | cmp_gen[52])) | &cmp_prop[54:51] & ((cmp_gen[51] | cmp_gen[50]) | &cmp_prop[50:49] & (cmp_gen[49] | cmp_gen[48])))) | &cmp_prop[61:47] & ((((cmp_gen[47] | cmp_gen[46]) | &cmp_prop[46:45] & (cmp_gen[45] | cmp_gen[44])) | &cmp_prop[46:43] & ((cmp_gen[43] | cmp_gen[42]) | &cmp_prop[42:41] & (cmp_gen[41] | cmp_gen[40]))) | &cmp_prop[46:39] & (((cmp_gen[39] | cmp_gen[38]) | &cmp_prop[38:37] & (cmp_gen[37] | cmp_gen[36])) | &cmp_prop[38:35] & ((cmp_gen[35] | cmp_gen[34]) | &cmp_prop[34:33] & (cmp_gen[33] | cmp_gen[32])))));
            assign cmp_upper_prop  = &cmp_prop[62:32];
            assign cmp_lower_gen   = ((((cmp_gen[30] | cmp_prop[29] & (cmp_gen[29] | cmp_gen[28])) | &cmp_prop[29:27] & ((cmp_gen[27] | cmp_gen[26]) | &cmp_prop[26:25] & (cmp_gen[25] | cmp_gen[24]))) | &cmp_prop[29:23] & (((cmp_gen[23] | cmp_gen[22]) | &cmp_prop[22:21] & (cmp_gen[21] | cmp_gen[20])) | &cmp_prop[22:19] & ((cmp_gen[19] | cmp_gen[18]) | &cmp_prop[18:17] & (cmp_gen[17] | cmp_gen[16])))) | &cmp_prop[29:15] & ((((cmp_gen[15] | cmp_gen[14]) | &cmp_prop[14:13] & (cmp_gen[13] | cmp_gen[12])) | &cmp_prop[14:11] & ((cmp_gen[11] | cmp_gen[10]) | &cmp_prop[10:9] & (cmp_gen[9] | cmp_gen[8]))) | &cmp_prop[14:7] & (((cmp_gen[7] | cmp_gen[6]) | &cmp_prop[6:5] & (cmp_gen[5] | cmp_gen[4])) | &cmp_prop[6:3] & ((cmp_gen[3] | cmp_gen[2]) | &cmp_prop[2:1] & (cmp_gen[1] | cmp_gen[0])))));
            // 64-bit: lt = g63' | (t63' & t62) & U | t63' & P[62:32] & g31 | (t63' & P[62:30]) & L (bits 62:0 unflipped)
            // 32-bit: lt = g31' | (t31' & t30) & L (bit 31 flipped for a signed compare)
            assign cmp_early_gen   = (amo_size_is64 & cmp_gen_top) | (amo_size_is64 & cmp_prop_top & cmp_upper_prop & cmp_gen[31]) |
                                     (~amo_size_is64 & cmp_gen_half);
            assign cmp_early_upper = amo_size_is64 & cmp_prop_top & cmp_prop[62];
            assign cmp_early_lower = (amo_size_is64 & cmp_prop_top & (&cmp_prop[62:30])) | (~amo_size_is64 & cmp_prop_half & cmp_prop[30]);
            assign old_less        = cmp_early_gen | (cmp_early_upper & cmp_upper_gen) | (cmp_early_lower & cmp_lower_gen);
            // ---- old + operand: Ling pseudo-carry tree (Kogge-Stone form), OR-propagate, from the pins ----
            // TIMING OPT (S2-I4): span N gen = H over the N bits ending at bit i, span N prop = t(i-1) & ... & t(i-N)
            wire [WRITE_DATA_WIDTH-1:0] add_bit_gen;      // per bit: old & operand (also the AND arm), 64 bits
            wire [WRITE_DATA_WIDTH-1:0] add_bit_prop;     // per bit: old | operand (also the OR arm), 64 bits
            wire [WRITE_DATA_WIDTH-1:0] add_span2_gen;    // pseudo carry of the 2 bits ending at bit i: g(i) | g(i-1), 64 bits
            wire [WRITE_DATA_WIDTH-1:0] add_span2_prop;   // t(i-1) & t(i-2), 64 bits
            wire [WRITE_DATA_WIDTH-1:0] add_span4_gen;    // ... 4 bits, 64 bits
            wire [WRITE_DATA_WIDTH-1:0] add_span4_prop;   // ... 4 bits, 64 bits
            wire [WRITE_DATA_WIDTH-1:0] add_span8_gen;    // ... 8 bits, 64 bits
            wire [WRITE_DATA_WIDTH-1:0] add_span8_prop;   // ... 8 bits, 64 bits
            wire [WRITE_DATA_WIDTH-1:0] add_span16_gen;   // ... 16 bits, 64 bits
            wire [WRITE_DATA_WIDTH-1:0] add_span16_prop;  // ... 16 bits, 64 bits
            wire [WRITE_DATA_WIDTH-1:0] add_span32_gen;   // ... 32 bits, 64 bits
            wire [WRITE_DATA_WIDTH-1:0] add_span32_prop;  // ... 32 bits, 64 bits
            wire [WRITE_DATA_WIDTH-1:0] old_xor_operand;  // TIMING OPT (S2-I4): old ^ operand if ADD or XOR, else 0, 64 bits
            assign add_bit_gen     = old_word & s2_wdata;                                    // 64 bits
            assign add_bit_prop    = old_word | s2_wdata;                                    // 64 bits
            assign add_span2_gen   = add_bit_gen     | (add_bit_gen      << 1);              // g(i) | t(i-1) & g(i-1) = g(i) | g(i-1)
            assign add_span2_prop  = (add_bit_prop   << 1) & (add_bit_prop << 2);
            assign add_span4_gen   = add_span2_gen   | (add_span2_prop   & (add_span2_gen   << 2));
            assign add_span4_prop  = add_span2_prop  & (add_span2_prop   << 2);
            assign add_span8_gen   = add_span4_gen   | (add_span4_prop   & (add_span4_gen   << 4));
            assign add_span8_prop  = add_span4_prop  & (add_span4_prop   << 4);
            assign add_span16_gen  = add_span8_gen   | (add_span8_prop   & (add_span8_gen   << 8));
            assign add_span16_prop = add_span8_prop  & (add_span8_prop   << 8);
            assign add_span32_gen  = add_span16_gen  | (add_span16_prop  & (add_span16_gen  << 16));
            assign add_span32_prop = add_span16_prop & (add_span16_prop  << 16);
            // old ^ operand from op-qualified inputs (no XOR on the raw pin pair; one shared operand net, see (4))
            assign old_xor_operand = ({WRITE_DATA_WIDTH{amo_op_logic}} & old_word) ^ amo_logic_operand;    // 64 bits
            // ---- outputs of the way (all qualified by this way) ----
            // compare and AND / OR arms, pre-qualified (old_less is the only late signal: one AND level per bit)
            assign amo_new_way[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH] =                                   // 64 bits
                       ({WRITE_DATA_WIDTH{ old_less}} & (({WRITE_DATA_WIDTH{way_min}} & old_word) | ({WRITE_DATA_WIDTH{hit_way_onehot[amo_way_idx]}} & amo_max_operand)))
                     | ({WRITE_DATA_WIDTH{~old_less}} & (({WRITE_DATA_WIDTH{way_max}} & old_word) | ({WRITE_DATA_WIDTH{hit_way_onehot[amo_way_idx]}} & amo_min_operand)))
                     | ({WRITE_DATA_WIDTH{way_and}} & add_bit_gen)
                     | ({WRITE_DATA_WIDTH{way_or}}  & add_bit_prop);
            assign amo_xor_way[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH] =                                   // 64 bits
                       {WRITE_DATA_WIDTH{hit_way_onehot[amo_way_idx]}} & old_xor_operand;
            assign amo_sum_if_carry_way[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH] =                          // 64 bits
                       {WRITE_DATA_WIDTH{hit_way_onehot[amo_way_idx]}} & (old_xor_operand ^ (add_bit_prop << 1));
            // the 6th (64-bit) stage with the way folded in: H(i-1) = X32(i-1) | Y32(i-1) & X32(i-33), each term AND the way
            assign amo_carry_way[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH] =                                 // 64 bits
                       ({WRITE_DATA_WIDTH{hit_way_onehot[amo_way_idx]}} & (add_span32_gen << 1))
                     | ({WRITE_DATA_WIDTH{hit_way_onehot[amo_way_idx]}} & (add_span32_prop << 1) & (add_span32_gen << 33));
        end
        if (WAYS < (1 << WAY_W)) begin : g_amo_no_way
            // WAYS not a power of 2: a code >= WAYS selects no word, the old unit then saw old value 0
            l1d_amo_alu #(.W(WRITE_DATA_WIDTH), .MEM_SIZE_BITS(MEM_SIZE_BITS)) u_amo_zero (
                .amo_op      (s2_req[REQ_AMOOP_LSB +: AMOOP_W]),                                    // AMOOP_W = 4 bits
                .access_size (s2_size),                                                             // MEM_SIZE_BITS = 2 bits
                .old_value   ({WRITE_DATA_WIDTH{1'b0}}),                                            // 64 bits
                .operand     (s2_wdata),                                                            // 64 bits
                .new_value   (amo_new_no_way)                                                       // 64 bits
            );
            assign hit_way_none = ~|hit_way_onehot;
        end else begin : g_amo_all_ways
            // WAYS a power of 2 (the default 4): every code names a way
            assign amo_new_no_way = {WRITE_DATA_WIDTH{1'b0}};
            assign hit_way_none   = 1'b0;
        end
    endgenerate
    // ---- the OR over the ways (every term is already qualified by its way) ----
    reg  [WRITE_DATA_WIDTH-1:0] amo_new_hit_way;          // OR of the ways' compare / AND / OR arms (and f(0, operand) for no way), 64 bits
    reg  [WRITE_DATA_WIDTH-1:0] amo_xor_hit_way;          // old ^ operand of the hit way (zero unless ADD or XOR), 64 bits
    reg  [WRITE_DATA_WIDTH-1:0] amo_sum_if_carry_hit_way; // TIMING OPT (S2-I4): the hit way's ADD sum if the carry into the bit is 1, 64 bits
    reg  [WRITE_DATA_WIDTH-1:0] amo_carry_hit_way;        // TIMING OPT (S2-I4): the hit way's Ling pseudo carry H(i-1) of every bit, 64 bits
    integer amo_sel_way;                                  // loop variable: way
    always @* begin
        amo_new_hit_way          = {WRITE_DATA_WIDTH{hit_way_none}} & amo_new_no_way;
        amo_xor_hit_way          = {WRITE_DATA_WIDTH{1'b0}};
        amo_sum_if_carry_hit_way = {WRITE_DATA_WIDTH{1'b0}};
        amo_carry_hit_way        = {WRITE_DATA_WIDTH{1'b0}};
        for (amo_sel_way = 0; amo_sel_way < WAYS; amo_sel_way = amo_sel_way + 1) begin
            amo_new_hit_way          = amo_new_hit_way          | amo_new_way[amo_sel_way*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];
            amo_xor_hit_way          = amo_xor_hit_way          | amo_xor_way[amo_sel_way*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];
            amo_sum_if_carry_hit_way = amo_sum_if_carry_hit_way | amo_sum_if_carry_way[amo_sel_way*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];
            amo_carry_hit_way        = amo_carry_hit_way        | amo_carry_way[amo_sel_way*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];
        end
    end
    // final: early arms (XOR, SWAP / other), the ADD sum as pc ? arm1 : arm0 on the late pseudo carry pc (sum(i) =
    // x(i) ^ (t(i-1) & H(i-1)) = H(i-1) ? x(i) ^ t(i-1) : x(i)), the ways' compare / AND / OR arms
    assign amo_new = ({WRITE_DATA_WIDTH{amo_op_xor}}        & amo_xor_hit_way)                            // XOR
                   | ({WRITE_DATA_WIDTH{amo_op_swap_other}} & s2_wdata)                                   // SWAP and other codes
                   | (~amo_carry_hit_way & ({WRITE_DATA_WIDTH{amo_op_add}} & amo_xor_hit_way))            // ADD, carry 0
                   | ( amo_carry_hit_way & ({WRITE_DATA_WIDTH{amo_op_add}} & amo_sum_if_carry_hit_way))   // ADD, carry 1
                   | amo_new_hit_way;                                                                     // MIN / MAX / AND / OR

    always @(posedge clk) begin : s2_to_s3_proc
        if (rst) s3_amo_v <= 1'b0;
        else     s3_amo_v <= s2_live && s2_hit && s2_amo1 && !ecc_data_fix;   // served AMO hit (not cancelled, not an upgrade, not scrubbed)
        s3_line <= s2_line;
        s3_way  <= s2_way;
        s3_old  <= line_q;                                // the hit way's line, read at the AMO's grant
        s3_be   <= s2_st_be;                              // the AMO word's bytes (offset, size)
        s3_off  <= s2_off;
        s3_new  <= amo_new;
    end // always s2_to_s3_proc

    // ======================================================================
    // Training record, eviction event, prefetch feedback, ECC pulses (registered: one flop stage before the top-level wires)
    // ======================================================================
    // the record goes once per request: the first S2 pass that is not a ghost and not tag-blocked (a tag-blocked pass has no
    // reliable hit / miss); the tracker remembers it (s2_train_sent -> trained bit)
    wire train_now;                                              // the record of the entry in S2 goes now
    wire evict_now;                                              // a line leaves the array now
    wire [LINE_ADDR_W-1:0] evict_line_now;                       // its line address, 58 bits
    assign train_now      = s2_live && !s2_tag_blocked && !s2_seat_trained;
    assign s2_train_sent  = train_now;
    assign evict_now      = (s2_refill && s2_vvalid) || (s2_snp && s2_snp_hit && s2_sinv) || (s2_maint && !s2_held && s2_vvalid);
    assign evict_line_now = s2_snp ? s2_line : s2_vline;
    always @(posedge clk) begin : records_proc
        if (rst) begin
            train_valid       <= 1'b0;
            evict_valid       <= 1'b0;
            pf_fb_valid       <= 1'b0;
            ecc_correctable   <= 1'b0;
            ecc_uncorrectable <= 1'b0;
        end else begin
            train_valid       <= train_now;
            evict_valid       <= evict_now;
            pf_fb_valid       <= s2_pf_op;
            ecc_correctable   <= ecc_data_fix || ecc_tag_fix_wr;
            ecc_uncorrectable <= ecc_data_ue || ecc_tag_ue;
        end
        // the wide fields load every cycle (no enable on wide data); their valid bits qualify them
        train_port      <= s2_port;
        train_pc        <= s2_seat_pc;
        train_addr      <= s2_addr;
        train_is_store  <= s2_store;
        train_miss      <= !s2_hit;
        train_pf_hit    <= s2_pf_hit;
        train_no_train  <= s2_amo || !s2_cache || (s2_mstid != MID_LSU);
        evict_line_addr <= evict_line_now;
        pf_fb_line_addr <= s2_line;
        pf_fb_result    <= pf_result;
    end // always records_proc

    // ======================================================================
    // Stage 2 decision for a snoop
    // ======================================================================
    // ---- Stage 2 decisions for a snoop: answer now (snp_push), with present / dirty / data from the cache line or the victim cache,
    //      or wait (snp_act_*). ----
    reg  snp_push;                 // answer goes to the snoop response FIFO
    reg  snp_present;              // the cache held the line
    reg  snp_dirty;                // the answer carries a dirty line
    reg  snp_from_vc;              // the answer data comes from the victim cache

    always @* begin
        snp_push      = 1'b0;
        snp_present   = 1'b0;
        snp_dirty     = 1'b0;
        snp_from_vc   = 1'b0;
        vc_snp        = 1'b0;
        snp_act_valid = 1'b0;
        snp_act_code  = SACT_DONE;
        snp_act_id    = {WID_W{1'b0}};
        if (s2_snp) begin
            snp_act_valid = 1'b1;
            if (s2_snp_hit) begin
                snp_push    = 1'b1;                                // state changed in S1
                snp_present = 1'b1;
                snp_dirty   = s2_hand;                             // M: line read in S1
            end else if (mshr_hit) begin
                if (mshr_hasd) begin
                    snp_act_code = SACT_WMSHR;                     // line about to be installed: wait
                    snp_act_id   = mshr_hid;
                end else begin
                    snp_push = 1'b1;                               // still on its way: not ours yet
                end
            end else if (vc_hit && vc_state == VC_VALID) begin
                snp_push    = 1'b1;
                snp_present = 1'b1;
                snp_dirty   = vc_dirty;
                snp_from_vc = 1'b1;
                vc_snp      = 1'b1;                                // slot -> FREE, or shared + clean
            end else if (vc_hit && vc_state == VC_WBPEND) begin
                snp_act_code = SACT_WVC;                           // wait for the B response
                snp_act_id   = vc_slot;
            end else if (vc_hit) begin
                snp_act_code = SACT_WMSHR;                         // MOVING: cannot happen without an
                snp_act_id   = mshr_hid;                           // MSHR entry (caught above)
            end else begin
                snp_push = 1'b1;                                   // nowhere: not present
            end
        end
    end

    // the snoop answer packet = { present, dirty, line address, line data } (the line leaves raw, without ECC correction: O-5)
    assign snoop_response_valid  = snp_push;
    assign snoop_response_packet = {snp_present, snp_dirty, s2_line, (snp_from_vc ? vc_data : line_q)};

    // ======================================================================
    // Victim-cache insert and release
    // ======================================================================
    // victim insert: a refill's victim (VALID) or a flush/clean line (WBPEND).  vc_rel: a swap that installed into an empty way
    // leaves its MOVING slot without a victim, so the slot is released.
    assign vc_ins = (s2_refill && s2_vvalid) || (s2_maint && !s2_held && s2_vvalid);
    // a swap that installed into an empty way leaves its MOVING slot empty
    assign vc_rel = s2_refill && s2_swap && !s2_vvalid;

// ---- simulation-only trace (compiled only when L1D_DEBUG is defined): the S2 result of each operation ----
`ifdef L1D_DEBUG
    function [8*4-1:0] dbg_cls;                           // request class name
        input [REQ_W-1:0] r;
        begin
            if (r[REQ_PF_BIT])        dbg_cls = "PF  ";
            else if (!r[REQ_AMO_BIT]) dbg_cls = r[REQ_RDWR_BIT] ? "ST  " : "LD  ";
            else if (!r[REQ_SEQ_BIT]) dbg_cls = r[REQ_RDWR_BIT] ? "SC  " : "LR  ";
            else                      dbg_cls = r[REQ_RDWR_BIT] ? "AMOW" : "AMOR";
        end
    endfunction

    always @(posedge clk) begin
        if (!rst) begin
            if (s2_pf_op)
                $display("[%0t] L1D B%0d S2 PF   line=%h unique=%b -> %s", $time, BANK_ID, {s2_line, OFF0}, s2_pfu,
                         (pf_result == PF_FB_ISSUED) ? "ISSUED (prefetch MSHR entry)" :
                         (pf_result == PF_FB_IN_CACHE) ? "dropped: in the cache" :
                         (pf_result == PF_FB_IN_FLIGHT) ? "dropped: already in the MSHR" : "dropped: no room");
            if (s2_cpu && s2_kill)
                $display("[%0t] L1D B%0d S2 LSU  entry=%0d idx=%0d ghost (cancelled in R): entry READY again", $time, BANK_ID, s2_seat, s2_idx);
            if (s2_live) begin
                if (s2_tag_blocked)
                    $display("[%0t] L1D B%0d S2 LSU  entry=%0d idx=%0d %0s addr=%h -> TAG ECC %s way %0d, re-issue (WECC)",
                             $time, BANK_ID, s2_seat, s2_idx, dbg_cls(s2_req), s2_addr,
                             tag_fix_valid ? "corrected" : "UNCORRECTABLE", tag_fix_way);
                else if (!s2_cache)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d NC %0s addr=%h -> %s",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr,
                             s3_scfail ? "SC fail (non-cacheable)" :
                             (n_alloc ? "NC queue" : "NC queue full, entry waits (WNC)"));
                else if (s2_scfail)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d SC addr=%h -> FAIL (answer 1)", $time, BANK_ID, s2_idx, s2_addr);
                else if (s2_hit && ecc_data_fix)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> HIT way=%0d DATA ECC corrected (granules %b), re-issue (WECC)",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr, s2_way, hit_fix);
                else if (s2_hit)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> HIT way=%0d data=%h%s",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr, s2_way, s2_rdata,
                             ecc_data_ue ? " (DATA ECC UNCORRECTABLE)" : "");
                else if (mshr_hit)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> MISS, line already in MSHR %0d: entry waits (WMSHR)",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr, mshr_hid);
                else if (vc_hit)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> victim-cache hit slot=%0d state=%0d: answer=%b swap=%b act=%0d",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr, vc_slot, vc_state,
                             s3_load, m_swap, act_code);
                else if (m_alloc)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> MISS: MSHR entry %0d allocated, %s fetch of line %h",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr, mshr_aid,
                             s2_uniq ? "unique" : "shared", {s2_line, OFF0});
                else
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> MISS, all MSHR entries busy: entry waits (WANY)",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr);
            end
            if (train_now)
                $display("[%0t] L1D B%0d S2 TRAIN port=%0d pc=%h addr=%h %s miss=%b pf_hit=%b%s", $time, BANK_ID, s2_port, s2_seat_pc,
                         s2_addr, s2_store ? "ST" : "LD", !s2_hit, s2_pf_hit,
                         (s2_amo || !s2_cache || (s2_mstid != MID_LSU)) ? " (no train)" : "");
            if (evict_now)
                $display("[%0t] L1D B%0d S2 EVICT line=%h", $time, BANK_ID, {evict_line_now, OFF0});
            if (s2_valid && s2_kind == K_REFILL)
                $display("[%0t] L1D B%0d S2 REFILL mshr=%0d line=%h %s",
                         $time, BANK_ID, s2_mid, {s2_line, OFF0},
                         s2_held ? "held, token goes back to the MSHR" : "INSTALLED (line + tag written), entry freed");
            if (s2_refill && s2_vvalid)
                $display("[%0t] L1D B%0d S2 victim line %h (dirty=%b shared=%b) -> victim-cache slot %0d",
                         $time, BANK_ID, {s2_vline, OFF0}, s2_vdirty, s2_vshared, s2_slot);
            if (s2_maint)
                $display("[%0t] L1D B%0d S2 MAINT %s", $time, BANK_ID,
                         s2_held ? "held (retry)" : (s2_vvalid ? "dirty line -> victim cache (WBPEND)" : "done"));
            if (s2_snp) begin
                if (snp_push)
                    $display("[%0t] L1D B%0d S2 SNOOP %s line=%h -> answer present=%b dirty=%b%s",
                             $time, BANK_ID, s2_sinv ? "INV" : "READ", {s2_line, OFF0}, snp_present, snp_dirty,
                             snp_from_vc ? " (from victim-cache slot)" : (mshr_hit ? " (MSHR still waiting for data)" : ""));
                else
                    $display("[%0t] L1D B%0d S2 SNOOP %s line=%h -> waits for %s %0d",
                             $time, BANK_ID, s2_sinv ? "INV" : "READ", {s2_line, OFF0},
                             snp_act_code == SACT_WVC ? "victim-cache writeback, slot" : "install, MSHR entry",
                             snp_act_id);
                if (vc_hit && vc_state == VC_MOVING && !mshr_hit)
                    $display("[%0t] L1D B%0d S2 SNOOP ERROR MOVING slot without an MSHR entry", $time, BANK_ID);
            end
        end
    end
`endif

endmodule

`default_nettype wire
`endif // L1D_BANK_S2_V
