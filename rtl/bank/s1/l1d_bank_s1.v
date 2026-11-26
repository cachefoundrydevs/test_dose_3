// ============================================================================
// l1d_bank_s1.v  -  bank stage S1: tag compare, MESI / PLRU update, victim choice, SC check, ECC syndromes, the S1 -> S2 registers
// ============================================================================
// IN SIMPLE WORDS
//   S1 looks at the registered tags of all 4 ways and decides "hit or miss, and in which way". In the same cycle it reads
//   and writes the line states (MESI), the PLRU bits and the "prefetched, not yet used" bits of the set (flip-flops,
//   l1d_meta_flops), chooses the victim way for a refill (never the AMO-locked line) and a victim-cache slot for it, checks an
//   SC against the reservation, makes the byte enables and shifted data of a store, and cuts the load word out of the line of
//   EVERY way (beside the compare, so Stage 2 only has to pick one). Everything it decided is registered into the s2_* registers.
//   cache_v6 ECC: beside the tag compare, 32 syndrome units (4 ways x 8 granules, l1d_ecc_syn) and 4 tag syndrome units check the
//   registered SRAM words. Their results go to S2 as flops: S2 answers only when the hit way is clean, else it writes the corrected
//   word and re-issues the request. A TAG error in any valid way of the set makes the hit / miss decision unreliable, so the CPU
//   request is BLOCKED in S1 (no state change, as a cancelled request) and S2 scrubs the tag.
//   cache_v6 prefetch: a prefetch (REQ_PF_BIT) runs like a load but changes no state and no PLRU; S2 gets its raw tag hit.
//
//   tag_words_q, s1_words (from l1d_bank_r) --> l1d_s1_lookup (hit, hit_way, victim, PLRU) --+--> s2_* registers (to S2)
//   l1d_meta_flops (states, PLRU, pf bits) <--> l1d_mesi_next (next state)                  |
//   per-way load word (l1d_resp_gen x 4), store byte enables, SC check, ECC syndromes --------+
//
// STAGE / CYCLE
//   Cycle c0+2 of a hit. Inputs: the S1 registers (from l1d_bank_r), the reservation / lock state, the MSHR entry being
//   installed and the free victim-cache slot. Outputs: the S2 registers, and three combinational signals used in this
//   same cycle by the MSHR (r_held) and the reservation unit (rsv_clear_valid / rsv_clear_line).
//   Estimated depth (not measured): tag compare 50 bits + hit + MESI next + state write about 12 levels (unchanged from the
//   lean S0, plus 1 level for the tag-error gate); a 72-input syndrome XOR tree about 10 gate equivalents into flops (PLAN_v6 O-6).
//
// INSTANTIATED BY
//   l1d_bank, instance u_s1 (one per bank).
//
// PARAMETERS
//   `L1D_PARAMS   all cache parameters (l1d_defines.vh); used here: SETS (64), WAYS (4), LINE_BYTES (64), WRITE_DATA_WIDTH (64),
//                 MEM_SIZE_BITS (2), RSV_CLR_ON_EVICT (1), HAS_ECC and the widths of l1d_fields.vh
//   BANK_ID       default 0: number of this bank (its bank bits in a victim's line address)
//
// PORTS (name, direction, width at the defaults, meaning)
//   clk, rst                     in   1      clock; synchronous reset, active high
//   s1_valid ... s1_sid, s1_kill in   regs   the S1 registers (from l1d_bank_r)
//   tag_words_q, s1_words        in   228/2304  registered coded tags and lines of all ways
//   lock_valid, lock_line        in   1/58   the AMO-locked line (never chosen as a victim)
//   q1_state, q1_swap, q1_slot   in   2/1/3  the MSHR entry being installed: its state, swap or not, its reserved VC slot
//   q1_is_prefetch               in   1      ... it is a prefetch (the installed line gets the pf bit)
//   vc_al_ok, vc_al_slot         in   1/3    a free victim-cache slot exists, and its number
//   rsv_sc_match                 in   1      the reservation covers the line in S1 (SC check)
//   r_held                       out  1      a refill token in S1 has no victim-cache slot: it is held (to the MSHR)
//   rsv_clear_valid/_line        out  1/58   a snoop or an eviction clears the reservation of this line (to the unit)
//   vec_read_slot                out  1      (C-20) data place of the vector store in S1 (to l1d_vec_store_buf)
//   vec_read_line_data           in   512    (C-20) its line: the store data of a vector store (all 64 byte enables on)
//   s2_valid ... s2_slot         out  regs   the S2 registers (see their comments)
//   s2_raw_hit, s2_pf_hit        out  regs   the tags hit (any CPU op, prefetch too); a demand hit used a prefetched line
//   s2_data_synd/_fix/_ce/_ue    out  regs   ECC: per way and granule the syndrome (7 bits) and "correctable"; per way any CE / UE
//   s2_tag_synd/_fix/_ce/_ue, s2_tag_blocked  out regs  ECC of the tags: per way; the CPU request was blocked by a tag error
//   s2_tags                      out  200    the raw tags of all ways (S2 corrects the erroneous one and writes it back)
//
// WORKED EXAMPLE (load hit in way 2, line in state E)
//   tag way 2 = the load's tag -> hit = 1, hit_way = 2, hit_st = E; m_ev = EV_LOAD_HIT -> state stays E, PLRU points
//   away from way 2 (pl_we = 1). s1_words holds the load word of every way. At the edge: s2_hit = 1, s2_way = 2,
//   s2_words = the 4 words; Stage 2 then answers with word 2 if s2_data_ce[2] = 0.
// ============================================================================
`ifndef L1D_BANK_S1_V
`define L1D_BANK_S1_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_bank_s1 #(
    `L1D_PARAMS,                                       // all cache parameters (l1d_defines.vh)
    parameter BANK_ID = 0                              // default 0: number of this bank (its bank bits in a line address)
) (
    clk, rst,
    s1_valid, s1_kind, s1_seat, s1_req, s1_mid, s1_line, s1_mway, s1_mop, s1_sinv, s1_sid, s1_kill,
    tag_words_q, s1_words,
    lock_valid, lock_line,
    q1_state, q1_swap, q1_slot, q1_is_prefetch, vc_al_ok, vc_al_slot,
    rsv_sc_match,
    r_held, rsv_clear_valid, rsv_clear_line,
    vec_read_slot, vec_read_line_data,
    s2_valid, s2_kind, s2_seat, s2_req, s2_mid, s2_line, s2_sinv, s2_sid,
    s2_hit, s2_scfail, s2_snp_hit, s2_hand, s2_way, s2_held, s2_swap, s2_kill,
    s2_words, s2_lines, s2_st_be, s2_st_din,
    s2_wbpend, s2_vvalid, s2_vdirty, s2_vshared, s2_vline, s2_slot,
    s2_raw_hit, s2_pf_hit,
    s2_data_synd, s2_data_fix, s2_data_ce, s2_data_ue,
    s2_tag_synd, s2_tag_fix, s2_tag_ce, s2_tag_ue, s2_tag_blocked,
    s2_tags
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // ---- local constants ----
    localparam [BANK_W-1:0]   BANK_C = BANK_ID;                  // this bank's number in a line address, BANK_W = 2 bits
    localparam [OFFSET_W-1:0] OFF0   = {OFFSET_W{1'b0}};         // zero line offset (debug messages print byte addresses), 6 bits
    localparam SYND_W   = WAYS * GRANULES * DATA_HAM_W;          // all data syndromes, 4*8*7 = 224 bits
    localparam TSYND_W  = WAYS * TAG_HAM_W;                      // all tag syndromes, 4*6 = 24 bits
    localparam FIX_W    = WAYS * GRANULES;                       // "correctable" per way and granule, 32 bits

    // ======================================================================
    // Ports
    // ======================================================================
    input  wire                        clk;                      // clock: all registers change on its rising edge
    input  wire                        rst;                      // synchronous reset, active high
    // ---- S1 registers (from l1d_bank_r) ----
    input  wire                        s1_valid;                 // stage S1 holds an operation
    input  wire [1:0]                  s1_kind;                  // its kind (K_LSU, K_REFILL, K_MAINT, K_SNOOP)
    input  wire [SEAT_W-1:0]           s1_seat;                  // its tracker entry, SEAT_W = 3 bits
    input  wire [REQ_W-1:0]            s1_req;                   // its request packet, REQ_W = 148 bits
    input  wire [MSHR_ID_W-1:0]        s1_mid;                   // MSHR entry of the refill token, 3 bits
    input  wire [LINE_ADDR_W-1:0]      s1_line;                  // its line address, 58 bits
    input  wire [WAY_W-1:0]            s1_mway;                  // way of the flush/clean token, WAY_W = 2 bits
    input  wire                        s1_mop;                   // operation of the flush/clean token (0 clean, 1 flush)
    input  wire                        s1_sinv;                  // snoop: 1 = invalidate
    input  wire [SNP_ID_W-1:0]         s1_sid;                   // snoop buffer entry, 1 bit
    input  wire                        s1_kill;                  // a fresh request cancelled in R
    input  wire [WAYS*TWORD_W-1:0]     tag_words_q;              // the coded tags of all ways, WAYS*TWORD_W = 228 bits
    input  wire [WAYS*DWORD_W-1:0]     s1_words;                 // the coded lines of all ways, WAYS*DWORD_W = 2304 bits
    // ---- side inputs ----
    input  wire                        lock_valid;               // a line of this bank is AMO-locked
    input  wire [LINE_ADDR_W-1:0]      lock_line;                // the locked line, 58 bits
    input  wire [1:0]                  q1_state;                 // MSHR entry being installed (refill token in S1): state to install
    input  wire                        q1_swap;                  // ... it is a swap back from the victim cache
    input  wire [VC_ID_W-1:0]          q1_slot;                  // ... its victim-cache slot (for a swap), VC_ID_W = 3 bits
    input  wire                        q1_is_prefetch;           // ... it is a prefetch: the installed line gets the pf bit
    input  wire                        vc_al_ok;                 // a victim-cache slot is free for a victim
    input  wire [VC_ID_W-1:0]          vc_al_slot;               // its number, 3 bits
    input  wire                        rsv_sc_match;             // the reservation covers the line in S1
    // ---- combinational results of S1 used in this cycle ----
    output wire                        r_held;                   // the refill token in S1 is held (no victim-cache slot)
    output wire                        rsv_clear_valid;          // a snoop or an eviction clears the reservation of rsv_clear_line
    output wire [LINE_ADDR_W-1:0]      rsv_clear_line;           // that line, 58 bits
    // ---- cache_v5 C-20: the line of a vector store (l1d_vec_store_buf) ----
    output wire [VEC_SLOT_W-1:0]       vec_read_slot;            // data place of the vector store in S1, VEC_SLOT_W = 1 bit
    input  wire [LINE_W-1:0]           vec_read_line_data;       // its line, LINE_W = 512 bits
    // ---- S1 -> S2 registers ----
    output reg                         s2_valid;                 // stage S2 holds an operation
    output reg  [1:0]                  s2_kind;                  // its kind
    output reg  [SEAT_W-1:0]           s2_seat;                  // its tracker entry, 3 bits
    output reg  [REQ_W-1:0]            s2_req;                   // its request packet, 148 bits
    output reg  [MSHR_ID_W-1:0]        s2_mid;                   // MSHR entry of the refill token being installed, 3 bits
    output reg  [LINE_ADDR_W-1:0]      s2_line;                  // its line address, 58 bits
    output reg                         s2_sinv;                  // snoop: 1 = invalidate
    output reg  [SNP_ID_W-1:0]         s2_sid;                   // snoop buffer entry, 1 bit
    output reg                         s2_hit;                   // the CPU request was a served cache hit in S1
    output reg                         s2_scfail;                // an SC whose reservation check failed in S1
    output reg                         s2_snp_hit;               // snoop: the tags hit
    output reg                         s2_hand;                  // snoop: the line was M, its data is handed over
    output reg  [WAY_W-1:0]            s2_way;                   // the way used in S1 (hit, victim or maintenance way), 2 bits
    output reg                         s2_held;                  // the refill / maintenance token was held in S1
    output reg                         s2_swap;                  // the refill is a swap back from the victim cache
    output reg                         s2_kill;                  // the fresh request was cancelled in R
    output reg  [WAYS*WRITE_DATA_WIDTH-1:0] s2_words;            // the load word of every way, WAYS*64 = 256 bits
    output reg  [WAYS*LINE_W-1:0]      s2_lines;                 // the data lines of all ways (victim / snoop / merge), 2048 bits
    output reg  [LINE_BYTES-1:0]       s2_st_be;                 // byte enables of a store hit, LINE_BYTES = 64 bits
    output reg  [LINE_W-1:0]           s2_st_din;                // store data at its byte position, LINE_W = 512 bits
    output reg                         s2_wbpend;                // the victim is a flush/clean line (inserted as WBPEND)
    output reg                         s2_vvalid;                // a line is being evicted
    output reg                         s2_vdirty;                // ... it is dirty
    output reg                         s2_vshared;               // ... it is shared
    output reg  [LINE_ADDR_W-1:0]      s2_vline;                 // ... its line address, 58 bits
    output reg  [VC_ID_W-1:0]          s2_slot;                  // victim-cache slot that takes it, 3 bits
    output reg                         s2_raw_hit;               // the tags hit for a cacheable CPU op or prefetch (no state check)
    output reg                         s2_pf_hit;                // a demand hit on a line that a prefetch brought in (first use)
    output reg  [SYND_W-1:0]           s2_data_synd;             // ECC: syndromes, way w granule g at [(w*GRANULES+g)*7 +: 7], 224 bits
    output reg  [FIX_W-1:0]            s2_data_fix;              // ECC: correctable per way and granule, 32 bits
    output reg  [WAYS-1:0]             s2_data_ce;               // ECC: some granule of the way is correctable, 4 bits
    output reg  [WAYS-1:0]             s2_data_ue;               // ECC: some granule of the way is uncorrectable, 4 bits
    output reg  [TSYND_W-1:0]          s2_tag_synd;              // ECC: tag syndromes, way w at [w*6 +: 6], 24 bits
    output reg  [WAYS-1:0]             s2_tag_fix;               // ECC: tag correctable per way, 4 bits
    output reg  [WAYS-1:0]             s2_tag_ce;                // ECC: tag correctable AND the way is valid, 4 bits
    output reg  [WAYS-1:0]             s2_tag_ue;                // ECC: tag uncorrectable AND the way is valid, 4 bits
    output reg                         s2_tag_blocked;           // the CPU request in S2 was blocked by a tag error (scrub, re-issue)
    output reg  [WAYS*TAG_W-1:0]       s2_tags;                  // the raw tags of all ways (the tag scrub of S2 corrects one), 200 bits

    // ======================================================================
    // The coded words split into tags / lines and their check bits
    // ======================================================================
    wire [WAYS*TAG_W-1:0]    tag_q;                              // the tags of all ways, 4*50 = 200 bits
    wire [WAYS*LINE_W-1:0]   s1_lines;                           // the data lines of all ways, 2048 bits
    genvar sw;                                                   // loop variable: way
    generate
        for (sw = 0; sw < WAYS; sw = sw + 1) begin : g_split
            assign tag_q[sw*TAG_W +: TAG_W]      = tag_words_q[sw*TWORD_W +: TAG_W];
            assign s1_lines[sw*LINE_W +: LINE_W] = s1_words[sw*DWORD_W +: LINE_W];
        end
    endgenerate

    // ======================================================================
    // Fields of the operation in S1
    // ======================================================================
    wire [INDEX_W-1:0] s1_set;                                   // set number of the operation in S1, 6 bits
    wire [TAG_W-1:0]   s1_tag;                                   // tag of the operation in S1, 50 bits
    wire               s1_cache;                                 // the request is cacheable
    wire               s1_store;                                 // write type: store, SC or AMO write
    wire               s1_amo;                                   // the request is an atomic (LR / SC / AMO)
    wire               s1_is_sc;                                 // the request is an SC
    wire               s1_pf;                                    // the operation is a prefetch
    wire [OFFSET_W-1:0] s1_off;                                  // byte offset of the access in the line, 6 bits
    wire [MEM_SIZE_BITS-1:0] s1_size;                            // access size, 2 bits
    wire [WRITE_DATA_WIDTH-1:0] s1_wdata;                        // store data, 64 bits
    wire               s1_lsu, s1_refill, s1_maint, s1_snp;      // the kind of the operation in S1
    wire               s1_line_op;                               // a whole-line VPU access
    wire               s2_lsu;                                   // a CPU request is in S2 (the SC check needs it)
    assign s1_set     = s1_line[LA_SET_LSB +: INDEX_W];
    assign s1_tag     = s1_line[LA_TAG_LSB +: TAG_W];
    assign s1_cache   = s1_req[REQ_CACHE_BIT];
    assign s1_store   = (s1_req[REQ_RDWR_BIT] == OP_WRITE);
    assign s1_amo     = s1_req[REQ_AMO_BIT];
    assign s1_is_sc   = s1_amo && !s1_req[REQ_SEQ_BIT] && s1_store;
    assign s1_pf      = s1_req[REQ_PF_BIT];
    assign s1_off     = s1_req[REQ_ADDR_LSB +: OFFSET_W];
    assign s1_size    = s1_req[REQ_SIZE_LSB +: MEM_SIZE_BITS];
    assign s1_wdata   = s1_req[REQ_DATA_LSB +: WRITE_DATA_WIDTH];
    assign s1_lsu     = s1_valid && (s1_kind == K_LSU);
    assign s1_refill  = s1_valid && (s1_kind == K_REFILL);
    assign s1_maint   = s1_valid && (s1_kind == K_MAINT);
    assign s1_snp     = s1_valid && (s1_kind == K_SNOOP);
    assign s1_line_op = s1_req[REQ_LINE_BIT];
    assign s2_lsu     = s2_valid && (s2_kind == K_LSU);
    assign vec_read_slot = s1_req[REQ_DATA_LSB +: VEC_SLOT_W];

    // ======================================================================
    // Line states, PLRU bits and pf bits (flops): read in S1 with s1_set, written in S1
    // ======================================================================
    wire [2*WAYS-1:0] m_states;                                  // state of every way, 8 bits
    wire [WAYS-2:0]   m_plru;                                    // PLRU bits, 3 bits
    wire [WAYS-1:0]   m_pf_bits;                                 // pf bit of every way, 4 bits
    wire              st_we;                                     // write a state
    wire [WAY_W-1:0]  st_way;                                    // ... of this way
    wire [1:0]        st_val;                                    // ... this value
    wire              pl_we;                                     // write the PLRU bits
    wire [WAYS-2:0]   plru_new;                                  // ... this value
    wire              pf_we;                                     // write a pf bit
    wire              pf_val;                                    // ... this value
    wire [WAY_W-1:0]  s1_way;                                    // the way the operation in S1 uses (declared below)
    l1d_meta_flops #(.SETS(SETS), .WAYS(WAYS), .INDEX_W(INDEX_W), .WAY_W(WAY_W)) u_meta (
        .clk                  (clk),
        .rst                  (rst),
        .read_set             (s1_set),
        .read_way_states_flat (m_states),
        .read_plru_bits       (m_plru),
        .read_pf_bits         (m_pf_bits),
        .state_write_enable   (st_we),
        .state_write_set      (s1_set),
        .state_write_way      (st_way),
        .state_write_value    (st_val),
        .plru_write_enable    (pl_we),
        .plru_write_set       (s1_set),
        .plru_write_value     (plru_new),
        .pf_write_enable      (pf_we),
        .pf_write_set         (s1_set),
        .pf_write_way         (s1_way),
        .pf_write_value       (pf_val)
    );

    // ======================================================================
    // ECC syndromes of all ways (HAS_ECC): data per granule, tags per way; a tag error of a VALID way blocks the CPU request
    // ======================================================================
    wire [SYND_W-1:0]  data_synd;                                // 224 bits
    wire [FIX_W-1:0]   data_fix;                                 // correctable per way and granule, 32 bits
    wire [FIX_W-1:0]   data_ue_g;                                // uncorrectable per way and granule, 32 bits
    wire [WAYS-1:0]    data_ce;                                  // per way, 4 bits
    wire [WAYS-1:0]    data_ue;                                  // per way, 4 bits
    wire [TSYND_W-1:0] tag_synd;                                 // 24 bits
    wire [WAYS-1:0]    tag_fix;                                  // tag correctable per way, 4 bits
    wire [WAYS-1:0]    tag_ue_raw;                               // tag uncorrectable per way, 4 bits
    wire [WAYS-1:0]    way_valid;                                // the way holds a line (state != I), 4 bits
    wire [WAYS-1:0]    tag_ce;                                   // correctable tag error of a valid way, 4 bits
    wire [WAYS-1:0]    tag_ue;                                   // uncorrectable tag error of a valid way, 4 bits
    wire               tag_err_any;                              // some valid way of the set has a tag error
    generate
        if (HAS_ECC != 0) begin : g_ecc
            genvar ew, eg;                                       // loop variables: way, granule
            for (ew = 0; ew < WAYS; ew = ew + 1) begin : g_ecc_way
                for (eg = 0; eg < GRANULES; eg = eg + 1) begin : g_ecc_gran
                    wire po_unused;                              // parity flag (correctable is used)
                    l1d_ecc_syn #(.DATA_W(GRAN_BITS), .HAM_W(DATA_HAM_W)) u_syn (
                        .data          (s1_words[ew*DWORD_W + eg*GRAN_BITS +: GRAN_BITS]),
                        .check         (s1_words[ew*DWORD_W + LINE_W + eg*DATA_ECC_W +: DATA_ECC_W]),
                        .syndrome      (data_synd[(ew*GRANULES + eg)*DATA_HAM_W +: DATA_HAM_W]),
                        .parity_odd    (po_unused),
                        .correctable   (data_fix[ew*GRANULES + eg]),
                        .uncorrectable (data_ue_g[ew*GRANULES + eg])
                    );
                end
                assign data_ce[ew] = |data_fix[ew*GRANULES +: GRANULES];
                assign data_ue[ew] = |data_ue_g[ew*GRANULES +: GRANULES];
                wire tpo_unused;                                 // tag parity flag
                l1d_ecc_syn #(.DATA_W(TAG_W), .HAM_W(TAG_HAM_W)) u_tag_syn (
                    .data          (tag_words_q[ew*TWORD_W +: TAG_W]),
                    .check         (tag_words_q[ew*TWORD_W + TAG_W +: TAG_ECC_W]),
                    .syndrome      (tag_synd[ew*TAG_HAM_W +: TAG_HAM_W]),
                    .parity_odd    (tpo_unused),
                    .correctable   (tag_fix[ew]),
                    .uncorrectable (tag_ue_raw[ew])
                );
            end
        end else begin : g_no_ecc
            assign data_synd  = {SYND_W{1'b0}};
            assign data_fix   = {FIX_W{1'b0}};
            assign data_ue_g  = {FIX_W{1'b0}};
            assign data_ce    = {WAYS{1'b0}};
            assign data_ue    = {WAYS{1'b0}};
            assign tag_synd   = {TSYND_W{1'b0}};
            assign tag_fix    = {WAYS{1'b0}};
            assign tag_ue_raw = {WAYS{1'b0}};
        end
    endgenerate
    genvar vw;                                                   // loop variable: way
    generate
        for (vw = 0; vw < WAYS; vw = vw + 1) begin : g_way_valid
            assign way_valid[vw] = (m_states[2*vw +: 2] != ST_I);
            assign tag_ce[vw]    = tag_fix[vw] & way_valid[vw];
            assign tag_ue[vw]    = tag_ue_raw[vw] & way_valid[vw];
        end
    endgenerate
    assign tag_err_any = |(tag_ce | tag_ue);

    // ======================================================================
    // Tag compare, victim choice, PLRU update
    // ======================================================================
    wire             hit;                                        // the tag of a valid way matches
    wire [WAY_W-1:0] hit_way, victim_way, vict_way;              // hit way; PLRU victim; victim with the lock rule, 2 bits each
    wire [WAY_W-1:0] acc_way;                                    // the way the PLRU update is about, 2 bits
    assign acc_way = (s1_kind == K_LSU) ? hit_way : vict_way;
    l1d_s1_lookup #(.WAYS(WAYS), .TAG_W(TAG_W), .WAY_W(WAY_W)) u_lookup (
        .way_tags_flat   (tag_q),
        .way_states_flat (m_states),
        .plru_bits       (m_plru),
        .lookup_tag      (s1_tag),
        .accessed_way    (acc_way),
        .hit             (hit),
        .hit_way         (hit_way),
        .victim_way      (victim_way),
        .plru_bits_next  (plru_new)
    );
    wire [1:0] hit_st;                                           // MESI state of the hit way
    assign hit_st = m_states[2*hit_way +: 2];

    // ---- SC check: the reservation must cover the line, and no LR / SC may be completing in S2 this cycle ----
    wire s2_lrsc;                                                // an LR / SC completes in S2 now
    wire sc_ok;                                                  // the SC may succeed
    wire s1_live;                                                // a CPU request that counts: not a ghost, not a prefetch, not tag-blocked
    wire s1_scfail;                                              // an SC that fails
    wire s1_lsu_th;                                              // a cacheable tag hit that is served or upgraded
    assign s2_lrsc   = s2_lsu && s2_req[REQ_AMO_BIT] && !s2_req[REQ_SEQ_BIT];
    assign sc_ok     = rsv_sc_match && !s2_lrsc;
    assign s1_live   = s1_lsu && !s1_kill && !s1_pf && !tag_err_any;
    assign s1_scfail = s1_live && s1_cache && s1_is_sc && !sc_ok;
    assign s1_lsu_th = s1_live && s1_cache && hit && !s1_scfail;

    // ---- refill: victim way and its line; the AMO-locked line is never evicted ----
    wire [LINE_ADDR_W-1:0] v0_line;                              // line of the PLRU victim, 58 bits
    wire                   v_locked;                             // ... it is the locked line
    wire [1:0]             v_st;                                 // state of the victim way
    wire                   v_valid, v_dirty;                     // the victim holds a line; it is dirty
    wire [LINE_ADDR_W-1:0] v_line;                               // line of the victim way, 58 bits
    wire [VC_ID_W-1:0]     r_slot;                               // victim-cache slot the victim takes, 3 bits
    assign v0_line  = {tag_q[victim_way*TAG_W +: TAG_W], s1_set, BANK_C};
    assign v_locked = lock_valid && (v0_line == lock_line);
    assign vict_way = v_locked ? (victim_way ^ {{(WAY_W-1){1'b0}}, 1'b1}) : victim_way;
    assign v_st     = m_states[2*vict_way +: 2];
    assign v_valid  = (v_st != ST_I);
    assign v_dirty  = (v_st == ST_M);
    assign v_line   = {tag_q[vict_way*TAG_W +: TAG_W], s1_set, BANK_C};
    assign r_slot   = q1_swap ? q1_slot : vc_al_slot;
    assign r_held   = s1_refill && v_valid && !q1_swap && !vc_al_ok;

    // ---- flush/clean token: the state and line of the way it examines ----
    wire [1:0]             mt_st;                                // state of the examined way
    wire                   mt_dirty;                             // ... it is dirty
    wire [LINE_ADDR_W-1:0] mt_vline;                             // its line, 58 bits
    wire                   m_held;                               // the token is held (no victim-cache slot for the dirty line)
    assign mt_st    = m_states[2*s1_mway +: 2];
    assign mt_dirty = (mt_st == ST_M);
    assign mt_vline = {tag_q[s1_mway*TAG_W +: TAG_W], s1_set, BANK_C};
    assign m_held   = s1_maint && mt_dirty && !vc_al_ok;

    // ======================================================================
    // MESI state logic: ONE event per cycle (served CPU hit, refill install, maintenance walk, snoop hit)
    // ======================================================================
    reg  [3:0] m_ev;                                             // the event (EV_*)
    reg  [1:0] m_cur;                                            // the state it applies to
    wire [1:0] m_nxt;                                            // the next state
    wire       m_we, m_upg, m_hand;                              // write it; upgrade needed; snoop hands data over
    always @* begin
        m_ev  = EV_NONE;
        m_cur = hit_st;
        if (s1_lsu_th)
            m_ev = s1_store ? EV_WRITE_HIT : (s1_amo ? EV_UNIQ_HIT : EV_LOAD_HIT);
        else if (s1_refill && !r_held)
            m_ev = EV_INSTALL;
        else if (s1_maint && !m_held) begin
            m_ev  = (s1_mop == MAINT_FLUSH) ? EV_FLUSH : EV_CLEAN;
            m_cur = mt_st;
        end else if (s1_snp && hit)
            m_ev = s1_sinv ? EV_SNP_INV : EV_SNP_READ;
    end
    l1d_mesi_next u_mesi (
        .event_code            (m_ev),
        .current_state         (m_cur),
        .install_state         (q1_state),
        .next_state            (m_nxt),
        .state_write_enable    (m_we),
        .upgrade_needed        (m_upg),
        .snoop_hands_over_data (m_hand)
    );
    wire s1_hit;                                                 // a served CPU hit (an upgrade is not a hit)
    assign s1_hit = s1_lsu_th && !m_upg;
    assign s1_way = (s1_kind == K_REFILL) ? vict_way :
                    (s1_kind == K_MAINT)  ? s1_mway : hit_way;
    assign st_we  = m_we;
    assign st_way = s1_way;
    assign st_val = m_nxt;
    assign pl_we  = s1_hit || (s1_refill && !r_held);

    // ---- the pf bit: set at a prefetch install; cleared at the first demand hit, a demand install, a snoop invalidate, a flush ----
    wire pf_demand_hit;                                          // a demand hit used a prefetched line (its first use)
    assign pf_demand_hit = s1_hit && m_pf_bits[hit_way];
    assign pf_we  = pf_demand_hit || (s1_refill && !r_held) || (s1_snp && hit && s1_sinv) ||
                    (s1_maint && !m_held && (s1_mop == MAINT_FLUSH));
    assign pf_val = s1_refill && q1_is_prefetch;

    // ======================================================================
    // Store bytes and per-way load words
    // ======================================================================
    reg  [LINE_BYTES-1:0] st_be;                                 // byte enables of the store, 64 bits
    wire [LINE_W-1:0]     st_wext;                               // store data zero-extended to a line, 512 bits
    wire [LINE_W-1:0]     st_din;                                // store data at its byte position, 512 bits
    integer bi;                                                  // loop variable: byte of the line
    integer nby;                                                 // bytes of the access
    assign st_wext = {{(LINE_W-WRITE_DATA_WIDTH){1'b0}}, s1_wdata};
    assign st_din  = s1_line_op ? vec_read_line_data : (st_wext << {s1_off, 3'b000});
    always @* begin
        nby = 1 << s1_size;
        for (bi = 0; bi < LINE_BYTES; bi = bi + 1)
            st_be[bi] = s1_line_op || ((bi >= s1_off) && (bi < s1_off + nby));
    end
    wire [WAYS*WRITE_DATA_WIDTH-1:0] s1_words_ld;                // the load word of every way, 256 bits
    genvar gwd;                                                  // loop variable: way
    generate
        for (gwd = 0; gwd < WAYS; gwd = gwd + 1) begin : g_word
            l1d_resp_gen #(.IN_BYTES(LINE_BYTES), .WRITE_DATA_WIDTH(WRITE_DATA_WIDTH),
                           .MEM_SIZE_BITS(MEM_SIZE_BITS), .OFF_W(OFFSET_W)) u_word (
                .source_block (s1_lines[gwd*LINE_W +: LINE_W]), .byte_offset (s1_off), .access_size (s1_size),
                .load_data    (s1_words_ld[gwd*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH])
            );
        end
    endgenerate

    // ======================================================================
    // Reservation clear: a snoop, or the eviction of the reserved line (when enabled)
    // ======================================================================
    wire s1_evict;                                               // a valid line is evicted by the refill
    assign s1_evict        = (RSV_CLR_ON_EVICT != 0) && s1_refill && !r_held && v_valid;
    assign rsv_clear_valid = s1_snp || s1_evict;
    assign rsv_clear_line  = s1_snp ? s1_line : v_line;

    // ======================================================================
    // S1 -> S2 registers
    // ======================================================================
    always @(posedge clk) begin : s1_to_s2_proc
        if (rst) begin
            s2_valid <= 1'b0;
        end else begin
            s2_valid       <= s1_valid;
            s2_kind        <= s1_kind;
            s2_seat        <= s1_seat;
            s2_req         <= s1_req;
            s2_mid         <= s1_mid;
            s2_line        <= s1_line;
            s2_sinv        <= s1_sinv;
            s2_sid         <= s1_sid;
            s2_hit         <= s1_hit;
            s2_scfail      <= s1_scfail;
            s2_snp_hit     <= s1_snp && hit;
            s2_hand        <= m_hand;
            s2_way         <= s1_way;
            s2_held        <= r_held || m_held;
            s2_swap        <= q1_swap;
            s2_kill        <= s1_kill;
            s2_words       <= s1_words_ld;
            s2_lines       <= s1_lines;
            s2_st_be       <= st_be;
            s2_st_din      <= st_din;
            s2_wbpend      <= s1_maint;
            s2_raw_hit     <= s1_lsu && s1_cache && hit;
            s2_pf_hit      <= pf_demand_hit;
            s2_data_synd   <= data_synd;
            s2_data_fix    <= data_fix;
            s2_data_ce     <= data_ce;
            s2_data_ue     <= data_ue;
            s2_tag_synd    <= tag_synd;
            s2_tag_fix     <= tag_fix;
            s2_tag_ce      <= tag_ce;
            s2_tag_ue      <= tag_ue;
            s2_tag_blocked <= s1_lsu && !s1_kill && !s1_pf && tag_err_any;
            s2_tags        <= tag_q;
            if (s1_maint) begin
                s2_vvalid  <= mt_dirty;
                s2_vdirty  <= 1'b1;
                s2_vshared <= 1'b0;
                s2_vline   <= mt_vline;
                s2_slot    <= vc_al_slot;
            end else begin
                s2_vvalid  <= v_valid;
                s2_vdirty  <= v_dirty;
                s2_vshared <= (v_st == ST_S);
                s2_vline   <= v_line;
                s2_slot    <= r_slot;
            end
        end
    end // always s1_to_s2_proc

// ---- simulation-only trace (compiled only when L1D_DEBUG is defined): the S1 lookup of each operation ----
`ifdef L1D_DEBUG
    always @(posedge clk) begin
        if (!rst) begin
            if (s1_lsu)
                $display("[%0t] L1D B%0d S1 %s entry=%0d idx=%0d %s %s addr=%h set=%0d -> %s way=%0d state=%0d%s%s%s%s",
                         $time, BANK_ID, s1_pf ? "PF " : "LSU", s1_seat, s1_req[REQ_IDX_LSB +: IDX_BITS],
                         (s1_store ? "ST" : "LD"), (s1_cache ? "C " : "NC"), s1_req[REQ_ADDR_LSB +: ADDR_W], s1_set,
                         (!s1_cache ? "non-cacheable" : (hit ? "HIT" : "MISS")), hit_way, hit_st,
                         s1_kill ? " (ghost)" : "", m_upg ? " (upgrade)" : "", s1_scfail ? " (SC fail)" : "",
                         tag_err_any ? " (TAG ECC ERROR: blocked)" : "");
            if (s1_refill)
                $display("[%0t] L1D B%0d S1 REFILL mshr=%0d line=%h set=%0d victim way=%0d state=%0d swap=%b slot=%0d install=%0d pf=%b%s",
                         $time, BANK_ID, s1_mid, {s1_line, OFF0}, s1_set, vict_way, v_st, q1_swap, r_slot, q1_state, q1_is_prefetch,
                         r_held ? " -> HELD" : "");
            if (s1_maint)
                $display("[%0t] L1D B%0d S1 MAINT set=%0d way=%0d state=%0d%s", $time, BANK_ID, s1_set, s1_mway, mt_st, m_held ? " -> HELD" : "");
            if (s1_snp)
                $display("[%0t] L1D B%0d S1 SNOOP %s line=%h -> %s way=%0d", $time, BANK_ID, s1_sinv ? "INV" : "READ", {s1_line, OFF0},
                         hit ? "HIT" : "tag miss", hit_way);
            if (s1_valid && (|data_ce || |data_ue || tag_err_any))
                $display("[%0t] L1D B%0d S1 ECC set=%0d data_ce=%b data_ue=%b tag_ce=%b tag_ue=%b", $time, BANK_ID, s1_set, data_ce, data_ue, tag_ce, tag_ue);
        end
    end
`endif

endmodule

`default_nettype wire
`endif // L1D_BANK_S1_V
