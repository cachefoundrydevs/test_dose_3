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
//   reservation_line_addr        in   58     (CB-3) the reservation line (flop of l1d_reservation_unit)
//   clear_eq_s2_line/_rsv_line   out  1/1    (CB-3) rsv_clear_line == s2_line / == reservation line, combinational
//   s2_way_onehot_copy           out  48     (CB-3) 12 registered one-hot copies of s2_way (to l1d_bank_s2)
//   s2_mid_onehot_copy           out  64     (CB-3) 8 registered one-hot copies of s2_mid (to l1d_mshr)
//   s2_kind_onehot_copy          out  4      (CB-4c) registered: bit k = (s2_valid && s2_kind == k) (to l1d_bank_s2)
//   s2_live_copy                 out  10     (CB-4c) 10 registered copies of s2_live (valid CPU op, not PF, not a ghost)
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
    rsv_sc_match, reservation_line_addr,
    r_held, rsv_clear_valid, rsv_clear_line, clear_eq_s2_line, clear_eq_rsv_line,
    vec_read_slot, vec_read_line_data,
    s2_valid, s2_kind, s2_seat, s2_req, s2_mid, s2_line, s2_sinv, s2_sid,
    s2_hit, s2_scfail, s2_snp_hit, s2_hand, s2_way, s2_held, s2_swap, s2_kill,
    s2_words, s2_lines, s2_st_be, s2_st_din,
    s2_wbpend, s2_vvalid, s2_vdirty, s2_vshared, s2_vline, s2_slot,
    s2_raw_hit, s2_pf_hit,
    s2_data_synd, s2_data_fix, s2_data_ce, s2_data_ue,
    s2_tag_synd, s2_tag_fix, s2_tag_ce, s2_tag_ue, s2_tag_blocked,
    s2_tags, s2_way_onehot_copy, s2_mid_onehot_copy,
    s2_kind_onehot_copy, s2_live_copy
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // ---- local constants ----
    localparam [BANK_W-1:0]   BANK_C = BANK_ID;                  // this bank's number in a line address, BANK_W = 2 bits
    localparam [OFFSET_W-1:0] OFF0   = {OFFSET_W{1'b0}};         // zero line offset (debug messages print byte addresses), 6 bits
    localparam SYND_W   = WAYS * GRANULES * DATA_HAM_W;          // all data syndromes, 4*8*7 = 224 bits
    localparam TSYND_W  = WAYS * TAG_HAM_W;                      // all tag syndromes, 4*6 = 24 bits
    localparam FIX_W    = WAYS * GRANULES;                       // "correctable" per way and granule, 32 bits
    localparam S2_WAY_COPIES = GRANULES + 4;                     // CB-3: copies of s2_way, 8 granule + 4 = 12 (same value in l1d_bank_s2, l1d_bank)
    localparam S2_LIVE_COPIES = GRANULES + 2;                    // CB-4c: copies of s2_live, 8 granule + decision + write port = 10 (same value in l1d_bank_s2, l1d_bank)

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
    input  wire [LINE_ADDR_W-1:0]      reservation_line_addr;    // CB-3: the reservation line (flop of u_rsv), 58 bits
    // ---- combinational results of S1 used in this cycle ----
    output wire                        r_held;                   // the refill token in S1 is held (no victim-cache slot)
    output wire                        rsv_clear_valid;          // a snoop or an eviction clears the reservation of rsv_clear_line
    output wire [LINE_ADDR_W-1:0]      rsv_clear_line;           // that line, 58 bits
    output wire                        clear_eq_s2_line;         // CB-3: rsv_clear_line == s2_line, combinational
    output wire                        clear_eq_rsv_line;        // CB-3: rsv_clear_line == reservation_line_addr, combinational
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
    output reg  [S2_WAY_COPIES*WAYS-1:0] s2_way_onehot_copy;     // CB-3: 12 one-hot copies of s2_way, copy c at [c*4 +: 4], 48 bits, registered
    output reg  [GRANULES*MSHR_ENTRIES-1:0] s2_mid_onehot_copy;  // CB-3: 8 one-hot copies of s2_mid, copy g at [g*8 +: 8], 64 bits, registered
    // CB-4c: (* keep = "true" *) on the declarations too (CB-3 review: keep on the always block alone may not stop a
    // tool from merging flops with equal D inputs; all 10 live copies have the same D)
    (* keep = "true" *) output reg [K_SNOOP:K_LSU] s2_kind_onehot_copy;      // CB-4c: bit k = (s2_valid && s2_kind == k), K_SNOOP:K_LSU = 4 bits, registered
    (* keep = "true" *) output reg [S2_LIVE_COPIES-1:0] s2_live_copy;        // CB-4c: each bit = s2_live of l1d_bank_s2, S2_LIVE_COPIES = 10 bits, registered

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
    // tm_opt s1 iter 3/4 (new signals): u_meta reads with a one-hot set (READ_ONEHOT) and gives the valid bits of the set
    // (m_valid, made before the read); the state and pf writes go to u_meta with one enable and one value PER WAY
    // (WRITE_ONEHOT), so no way number is encoded here and decoded again in u_meta. The v6 write ports keep their values
    // (the simulation trace of u_meta reads them).
    wire [WAYS-1:0]   m_valid;                                   // way w of the set holds a line (state != I), WAYS = 4 bits
    wire [WAYS-1:0]   st_we_way;                                 // write the state of way w, WAYS = 4 bits
    wire [2*WAYS-1:0] st_val_way;                                // ... with this value at [2*w +: 2], 2 x WAYS = 8 bits
    wire [WAYS-1:0]   pf_we_way;                                 // write pf_val to the pf bit of way w, WAYS = 4 bits
    // tm_opt s1 iter 14 (new signal): the PLRU write per way too: way w's enable writes plru_cand of way w (PLRU_ONEHOT)
    wire [WAYS-1:0]   pl_we_way;                                 // write the PLRU bits of an access of way w, WAYS = 4 bits
    // tm_opt s1 iter 23: u_meta WRITE_BEHIND = 1: the per-way writes of this cycle go into u_meta's pending register (~50
    // flops) instead of the 64-set storage; the storage takes them one cycle later from flops, and every read sees them
    // (forwarding after the read). Same values on every read and in u_meta.states / plru / pf_bit, cycle by cycle.
    l1d_meta_flops #(.SETS(SETS), .WAYS(WAYS), .INDEX_W(INDEX_W), .WAY_W(WAY_W),
                     .READ_ONEHOT(1), .WRITE_ONEHOT(1), .PLRU_ONEHOT(1), .WRITE_BEHIND(1)) u_meta (
        .clk                    (clk),
        .rst                    (rst),
        .read_set               (s1_set),
        .read_way_states_flat   (m_states),
        .read_way_valid         (m_valid),
        .read_plru_bits         (m_plru),
        .read_pf_bits           (m_pf_bits),
        .state_write_enable     (st_we),
        .state_write_set        (s1_set),
        .state_write_way        (st_way),
        .state_write_value      (st_val),
        .state_write_way_enable (st_we_way),
        .state_write_way_value  (st_val_way),
        .plru_write_enable      (pl_we),
        .plru_write_set         (s1_set),
        .plru_write_value       (plru_new),
        .plru_write_way_enable  (pl_we_way),
        .plru_write_way_value   (plru_cand),
        .pf_write_enable        (pf_we),
        .pf_write_set           (s1_set),
        .pf_write_way           (s1_way),
        .pf_write_value         (pf_val),
        .pf_write_way_enable    (pf_we_way)
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
            assign way_valid[vw] = m_valid[vw];                  // tm_opt s1 iter 3: = (m_states[2*vw +: 2] != ST_I), made before the read
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
    wire [WAYS-2:0]  plru_next_lookup;                           // u_lookup's PLRU bits after the access (tm_opt s1 iter 2: unused), WAYS-1 = 3 bits
    wire             hit_lookup;                                 // u_lookup's hit (tm_opt s1 iter 3: unused; hit = |way_tag_match, same value)
    wire [WAY_W-1:0] hit_way_lookup;                             // u_lookup's hit way (tm_opt s1 iter 13: unused; hit_way from hit_onehot)
    l1d_s1_lookup #(.WAYS(WAYS), .TAG_W(TAG_W), .WAY_W(WAY_W)) u_lookup (
        .way_tags_flat   (tag_q),
        .way_states_flat (m_states),
        .plru_bits       (m_plru),
        .lookup_tag      (s1_tag),
        .accessed_way    (acc_way),
        .hit             (hit_lookup),
        .hit_way         (hit_way_lookup),
        .victim_way      (victim_way),
        .plru_bits_next  (plru_next_lookup)
    );

    // ---- tm_opt s1 iter 2: the hit way as a ONE-HOT vector, and the PLRU update of every way in parallel ----
    // Before: match -> l1d_pri_enc (binary hit_way) -> acc_way mux -> l1d_plru update -> plru_new, and every "value of the hit
    // way" (state, next state, pf bit) a binary-indexed mux after hit_way. Now hit_onehot is built from the per-way match in one
    // parallel level (the lowest matching way, as l1d_pri_enc picks it; way 0 when no way matches, as l1d_pri_enc gives 0), so
    // every select by the hit way is one AND-OR. The PLRU bits after an access are computed for all WAYS accessed ways at once
    // from the PLRU bits read (u_plru_way, early), and the one-hot accessed way picks one at the end. Same values for all inputs:
    // hit_onehot has its 1 exactly at hit_way, acc_onehot exactly at acc_way. u_lookup keeps acc_way, so its own PLRU output
    // (plru_next_lookup) equals plru_new; it is not used any more.
    wire [WAYS-1:0]   way_tag_match;                             // way w is valid and its tag is the lookup tag, WAYS = 4 bits
    wire [WAYS-1:0]   hit_onehot;                                // 1 at hit_way: the lowest matching way, or way 0, WAYS = 4 bits
    wire [WAYS-1:0]   vict_onehot;                               // 1 at vict_way, WAYS = 4 bits
    wire [WAYS-1:0]   acc_onehot;                                // 1 at acc_way, WAYS = 4 bits
    wire [WAYS*(WAYS-1)-1:0] plru_cand;                          // PLRU bits after an access of way w, WAYS x (WAYS-1) = 12 bits
    // tm_opt s1 iter 4: hit_first = hit_onehot without the "no match -> way 0" default (used where a hit is required anyway)
    wire [WAYS-1:0]   hit_first;                                 // 1 at the lowest matching way, all 0 without a match, WAYS = 4 bits
    // tm_opt s1 iter 6: the victim as a one-hot vector, every way in parallel (was: valid -> l1d_pri_enc -> victim_way ->
    // lock compare select -> vict_way -> vict_onehot compare)
    wire [WAYS*WAY_W-1:0] plru_vict_way;                         // PLRU victim from every u_plru_way (all equal), WAYS x WAY_W = 8 bits
    wire [WAYS-1:0]   inv_first;                                 // way w is the lowest invalid way, WAYS = 4 bits
    wire              any_invalid;                               // some way of the set is invalid
    wire [WAYS-1:0]   plru_vict_hot;                             // 1 at the PLRU victim, WAYS = 4 bits
    wire [WAYS-1:0]   victim_onehot;                             // 1 at victim_way (before the lock rule), WAYS = 4 bits
    wire [WAYS-1:0]   lock_way;                                  // way w holds the locked line (lock_valid and lock_match[w]), WAYS = 4 bits
    wire [WAYS-1:0]   partner_valid;                             // way w ^ 1 (the lock rule's other way) is valid, WAYS = 4 bits
    assign any_invalid = !(&way_valid);
    genvar hw;                                                   // loop variable: way
    generate
        for (hw = 0; hw < WAYS; hw = hw + 1) begin : g_hit_onehot
            wire [WAY_W-1:0] cand_way;                           // way number hw, WAY_W = 2 bits
            assign cand_way           = hw;
            assign way_tag_match[hw]  = way_valid[hw] && (tag_q[hw*TAG_W +: TAG_W] == s1_tag);
            if (hw == 0) begin : g_first
                assign hit_onehot[hw] = way_tag_match[hw] || !(|way_tag_match);   // lowest match, or no match at all
                assign hit_first[hw]  = way_tag_match[hw];
                assign inv_first[hw]  = !way_valid[hw];
            end else begin : g_rest
                assign hit_onehot[hw] = way_tag_match[hw] && !(|way_tag_match[hw-1:0]);
                assign hit_first[hw]  = hit_onehot[hw];
                assign inv_first[hw]  = !way_valid[hw] && (&way_valid[hw-1:0]);
            end
            assign acc_onehot[hw]     = (s1_kind == K_LSU) ? hit_onehot[hw] : vict_onehot[hw];
            l1d_plru #(.WAYS(WAYS), .WAY_W(WAY_W), .NB(WAYS-1)) u_plru_way (
                .plru_bits      (m_plru),
                .accessed_way   (cand_way),
                .victim_way     (plru_vict_way[hw*WAY_W +: WAY_W]),
                .plru_bits_next (plru_cand[hw*(WAYS-1) +: WAYS-1])
            );
        end
    endgenerate
    // one AND-OR per bit by a one-hot select (exactly one bit of the select is 1)
    assign hit = |way_tag_match;                                 // tm_opt s1 iter 3: from the valid bits made before the read
    reg  [WAYS-2:0] plru_pick;                                   // the PLRU bits of the accessed way, WAYS-1 = 3 bits
    reg  [1:0]      hit_st_pick;                                 // the state of the hit way
    reg  [WAY_W-1:0] hit_way_pick;                               // tm_opt s1 iter 13: = l1d_pri_enc of the match (0 without a match), 2 bits
    integer         pk;                                          // loop variable: way
    always @* begin
        plru_pick   = {(WAYS-1){1'b0}};
        hit_st_pick = 2'b00;
        hit_way_pick = {WAY_W{1'b0}};
        for (pk = 0; pk < WAYS; pk = pk + 1) begin
            hit_way_pick = hit_way_pick | ({WAY_W{hit_onehot[pk]}} & pk[WAY_W-1:0]);
            plru_pick   = plru_pick   | ({(WAYS-1){acc_onehot[pk]}} & plru_cand[pk*(WAYS-1) +: WAYS-1]);
            hit_st_pick = hit_st_pick | ({2{hit_onehot[pk]}} & m_states[2*pk +: 2]);
        end
    end
    assign plru_new = plru_pick;
    wire [1:0] hit_st;                                           // MESI state of the hit way
    assign hit_st = hit_st_pick;                                 // = m_states[2*hit_way +: 2]
    // tm_opt s1 iter 13: hit_onehot has its 1 exactly at the lowest matching way (way 0 without a match), so its number is
    // u_lookup's hit_way; nothing reads u_lookup any more (synthesis removes it), its read-side copy of valid / match /
    // victim no longer loads the meta read
    assign hit_way = hit_way_pick;

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
    // tm_opt s1 iter 1 (new signal): the same served-or-upgraded CPU tag hit WITHOUT the tag-error gate. Equal to s1_lsu_th
    // whenever tag_err_any = 0 (s1_lsu_th = s1_lsu_th_early && !tag_err_any). The late tag syndrome (4 x l1d_ecc_syn on
    // tag_words_q, then the valid-way mask) now enters only the last AND of the state write / hit terms below.
    wire s1_lsu_th_early;                                        // cacheable tag hit, served or upgraded, before the tag-error gate
    // tm_opt s1 iter 4 (new signal): the same without the hit (the per-way writes AND it with the way's own first match)
    wire s1_lsu_go;                                              // a cacheable CPU request that is served or upgraded if it hits
    assign s1_lsu_go       = s1_lsu && !s1_kill && !s1_pf && s1_cache && !(s1_is_sc && !sc_ok);
    assign s1_lsu_th_early = s1_lsu_go && hit;

    // ---- refill: victim way and its line; the AMO-locked line is never evicted ----
    wire [LINE_ADDR_W-1:0] v0_line;                              // line of the PLRU victim, 58 bits
    wire                   v_locked;                             // ... it is the locked line
    wire [1:0]             v_st;                                 // state of the victim way
    wire                   v_valid, v_dirty;                     // the victim holds a line; it is dirty
    wire [LINE_ADDR_W-1:0] v_line;                               // line of the victim way, 58 bits
    wire [VC_ID_W-1:0]     r_slot;                               // victim-cache slot the victim takes, 3 bits
    assign v0_line  = {tag_q[victim_way*TAG_W +: TAG_W], s1_set, BANK_C};
    // tm_opt s1 iter 1 (new signal): the 58-bit compare with the locked line runs for EVERY way in parallel with the PLRU
    // victim choice; the late victim_way only selects one compare result (was: select the victim's tag, then compare).
    // Same value: lock_match[victim_way] == (v0_line == lock_line).
    wire [WAYS-1:0] lock_match;                                  // way w's line is the locked line, WAYS = 4 bits
    genvar lw;                                                   // loop variable: way
    generate
        for (lw = 0; lw < WAYS; lw = lw + 1) begin : g_lock_match
            assign lock_match[lw] = ({tag_q[lw*TAG_W +: TAG_W], s1_set, BANK_C} == lock_line);
        end
    endgenerate
    // tm_opt s1 iter 6: victim_way = the lowest invalid way if there is one, otherwise the PLRU victim (as u_lookup); the lock
    // rule moves the victim to way ^ 1 when the victim holds the locked line. So way vh is the final victim when it is the
    // victim and not locked, or when its partner vh ^ 1 is the victim and locked: one AND-OR, no binary way number.
    // Same value as (vict_way == cand_way).
    genvar vh;                                                   // loop variable: way
    generate
        for (vh = 0; vh < WAYS; vh = vh + 1) begin : g_vict_onehot
            wire [WAY_W-1:0] vict_cand;                          // way number vh, WAY_W = 2 bits
            assign vict_cand          = vh;
            assign plru_vict_hot[vh]  = (plru_vict_way[WAY_W-1:0] == vict_cand);
            assign victim_onehot[vh]  = any_invalid ? inv_first[vh] : plru_vict_hot[vh];
            assign lock_way[vh]       = lock_valid && lock_match[vh];
            assign partner_valid[vh]  = way_valid[vh ^ 1];
            assign vict_onehot[vh]    = (victim_onehot[vh] && !lock_way[vh]) || (victim_onehot[vh ^ 1] && lock_way[vh ^ 1]);
        end
    endgenerate
    assign v_locked = |(victim_onehot & lock_way);               // tm_opt s1 iter 6: = lock_valid && lock_match[victim_way]
    // tm_opt s1 iter 10: the victim's way number, state and tag are picked by the one-hot vict_onehot (one AND-OR each)
    // instead of a 4:1 select by the binary vict_way (was: u_lookup's binary victim_way, then the lock flip, then the select).
    // Same values: vict_onehot has its 1 exactly at v_locked ? victim_way ^ 1 : victim_way.
    reg  [WAY_W-1:0] vict_way_pick;                              // = vict_way, WAY_W = 2 bits
    reg  [1:0]       vict_st_pick;                               // = m_states[2*vict_way +: 2]
    reg  [TAG_W-1:0] vict_tag_pick;                              // = tag_q[vict_way*TAG_W +: TAG_W], TAG_W = 50 bits
    integer          vp;                                         // loop variable: way
    always @* begin
        vict_way_pick = {WAY_W{1'b0}};
        vict_st_pick  = 2'b00;
        vict_tag_pick = {TAG_W{1'b0}};
        for (vp = 0; vp < WAYS; vp = vp + 1) begin
            vict_way_pick = vict_way_pick | ({WAY_W{vict_onehot[vp]}} & vp[WAY_W-1:0]);
            vict_st_pick  = vict_st_pick  | ({2{vict_onehot[vp]}}     & m_states[2*vp +: 2]);
            vict_tag_pick = vict_tag_pick | ({TAG_W{vict_onehot[vp]}} & tag_q[vp*TAG_W +: TAG_W]);
        end
    end
    assign vict_way = vict_way_pick;
    assign v_st     = vict_st_pick;
    // tm_opt s1 iter 6: = (v_st != ST_I) without the victim select: with every way valid the victim is valid whatever the
    // lock rule picks; otherwise the victim is the lowest invalid way, valid only when the lock rule moves it to its partner
    assign v_valid  = !any_invalid || |(inv_first & lock_way & partner_valid);
    assign v_dirty  = (v_st == ST_M);
    assign v_line   = {vict_tag_pick, s1_set, BANK_C};          // tm_opt s1 iter 10: one-hot tag pick
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
    // tm_opt s1 iter 1: m_ev / m_cur above are kept for the debug print only. The next state is no longer computed AFTER the
    // tag compare (hit_way -> hit_st -> m_ev -> l1d_mesi_next) but in parallel with it:
    //   - hit-type events (CPU hit, snoop hit) apply to the hit way: one l1d_mesi_next per way (u_mesi_way) runs on the
    //     way's own state with the event of the request kind (early: from the S1 registers only), and hit_way selects;
    //   - the flush / clean walk applies to way s1_mway (a flop): u_mesi runs on mt_st;
    //   - the install writes q1_state.
    // The case of the old priority chain (s1_lsu_th, refill and not held, maint and not held, snoop hit, none) then selects.
    // The kinds exclude each other (s1_kind), so at most one case is open. Every output is the same as before for all inputs.
    reg  [3:0] hit_ev;                                           // the event a hit of this request kind would make (EV_*)
    always @* begin
        if (s1_snp)
            hit_ev = s1_sinv ? EV_SNP_INV : EV_SNP_READ;
        else
            hit_ev = s1_store ? EV_WRITE_HIT : (s1_amo ? EV_UNIQ_HIT : EV_LOAD_HIT);
    end
    wire [2*WAYS-1:0] way_nxt;                                   // next state of every way for hit_ev, 2 x WAYS = 8 bits
    wire [WAYS-1:0]   way_we, way_upg, way_hand;                 // ... write, upgrade, hand over data, WAYS = 4 bits each
    genvar mw;                                                   // loop variable: way
    generate
        for (mw = 0; mw < WAYS; mw = mw + 1) begin : g_mesi_way
            l1d_mesi_next u_mesi_way (
                .event_code            (hit_ev),
                .current_state         (m_states[2*mw +: 2]),
                .install_state         (q1_state),
                .next_state            (way_nxt[2*mw +: 2]),
                .state_write_enable    (way_we[mw]),
                .upgrade_needed        (way_upg[mw]),
                .snoop_hands_over_data (way_hand[mw])
            );
        end
    endgenerate
    wire [1:0] hit_nxt;                                          // next state of the hit way
    wire       hit_we, hit_upg, hit_hand;                        // ... write, upgrade, hand over data
    // tm_opt s1 iter 2: selected by the one-hot hit way (one AND-OR), same values as the binary-indexed selects by hit_way
    reg  [1:0] hit_nxt_pick;                                     // next state of the hit way
    reg        hit_we_pick, hit_upg_pick, hit_hand_pick;         // ... write, upgrade, hand over data
    integer    hp;                                               // loop variable: way
    always @* begin
        hit_nxt_pick  = 2'b00;
        hit_we_pick   = 1'b0;
        hit_upg_pick  = 1'b0;
        hit_hand_pick = 1'b0;
        for (hp = 0; hp < WAYS; hp = hp + 1) begin
            hit_nxt_pick  = hit_nxt_pick  | ({2{hit_onehot[hp]}} & way_nxt[2*hp +: 2]);
            hit_we_pick   = hit_we_pick   | (hit_onehot[hp] & way_we[hp]);
            hit_upg_pick  = hit_upg_pick  | (hit_onehot[hp] & way_upg[hp]);
            hit_hand_pick = hit_hand_pick | (hit_onehot[hp] & way_hand[hp]);
        end
    end
    assign hit_nxt  = hit_nxt_pick;
    assign hit_we   = hit_we_pick;
    assign hit_upg  = hit_upg_pick;
    assign hit_hand = hit_hand_pick;
    wire [1:0] mt_nxt;                                           // next state of the walked way (flush / clean)
    wire       mt_we, mt_upg_unused, mt_hand_unused;             // ... write; upgrade and hand-over are always 0 for a walk
    l1d_mesi_next u_mesi (
        .event_code            ((s1_mop == MAINT_FLUSH) ? EV_FLUSH : EV_CLEAN),
        .current_state         (mt_st),
        .install_state         (q1_state),
        .next_state            (mt_nxt),
        .state_write_enable    (mt_we),
        .upgrade_needed        (mt_upg_unused),
        .snoop_hands_over_data (mt_hand_unused)
    );
    wire case_cpu, case_inst, case_walk, case_snp;               // which event of the old priority chain is open
    assign case_cpu  = s1_lsu_th;                                // (kinds exclude each other: at most one is 1)
    assign case_inst = s1_refill && !r_held;
    assign case_walk = s1_maint && !m_held;
    assign case_snp  = s1_snp && hit;
    // the tag-error gate (inside s1_lsu_th) is the last term of the CPU leg
    assign m_we   = (s1_lsu_th_early && hit_we && !tag_err_any) || case_inst || (case_walk && mt_we) || (case_snp && hit_we);
    assign m_upg  = s1_lsu_th_early && hit_upg && !tag_err_any;  // only a CPU hit can need an upgrade
    assign m_hand = case_snp && hit_hand;                        // only a snoop hit hands data over
    assign m_nxt  = (case_cpu || case_snp) ? hit_nxt :
                    case_inst              ? q1_state :
                    case_walk              ? mt_nxt : hit_st;    // no event: the state is unchanged (m_cur = hit_st)
    wire s1_hit;                                                 // a served CPU hit (an upgrade is not a hit)
    // tm_opt s1 iter 5: = s1_lsu_th_early && !hit_upg && !tag_err_any (= s1_lsu_th && !m_upg), as an OR over the ways of
    // (first match AND no upgrade): the per-way upgrade flag no longer waits for the select by the hit way
    assign s1_hit = s1_lsu_go && !tag_err_any && |(hit_first & ~way_upg);
    assign s1_way = (s1_kind == K_REFILL) ? vict_way :
                    (s1_kind == K_MAINT)  ? s1_mway : hit_way;
    assign st_we  = m_we;
    assign st_way = s1_way;
    assign st_val = m_nxt;
    assign pl_we  = s1_hit || (s1_refill && !r_held);

    // ---- the pf bit: set at a prefetch install; cleared at the first demand hit, a demand install, a snoop invalidate, a flush ----
    wire pf_demand_hit;                                          // a demand hit used a prefetched line (its first use)
    assign pf_demand_hit = s1_hit && |(hit_onehot & m_pf_bits);   // tm_opt s1 iter 2: = m_pf_bits[hit_way], one-hot select
    assign pf_we  = pf_demand_hit || (s1_refill && !r_held) || (s1_snp && hit && s1_sinv) ||
                    (s1_maint && !m_held && (s1_mop == MAINT_FLUSH));
    assign pf_val = s1_refill && q1_is_prefetch;

    // ---- tm_opt s1 iter 4: the state and pf writes PER WAY, every way in parallel (u_meta WRITE_ONEHOT) ----
    // Before: the legs were ORed into one enable (m_we, pf_we), the way number s1_way was chosen by the kind (binary hit_way
    // from l1d_pri_enc, vict_way or s1_mway), the value m_nxt was selected by the hit way, and u_meta decoded the way number
    // again. Now way pw gets its own enable from its own first match, victim bit or walk bit, and its own next state:
    //   CPU hit or snoop hit : (s1_lsu_go and no tag error, or a snoop) and hit_first[pw] and way_we[pw]  -> way_nxt of way pw
    //   refill install       : s1_refill and vict_onehot[pw] and not held (victim invalid, or swap / free slot) -> q1_state
    //   flush / clean walk   : case_walk and mt_we and s1_mway == pw                                         -> mt_nxt
    // The kinds exclude each other, so at most one leg is open and it names exactly the way and the value of the v6 write
    // (st_we, st_way, st_val; pf_we, s1_way). For the pf bit the legs are those of pf_we, at the same ways.
    wire       refill_ok;                                        // a valid victim may leave: swap, or a free victim-cache slot
    wire [WAYS-1:0] inst_way;                                    // the refill installs into way w now (case_inst at vict_way), WAYS = 4 bits
    wire [WAYS-1:0] mway_hot;                                    // 1 at s1_mway (the walked way), WAYS = 4 bits
    assign refill_ok = q1_swap || vc_al_ok;
    genvar pw;                                                   // loop variable: way
    generate
        for (pw = 0; pw < WAYS; pw = pw + 1) begin : g_write_way
            wire [WAY_W-1:0] write_cand;                         // way number pw, WAY_W = 2 bits
            assign write_cand    = pw;
            assign mway_hot[pw]  = (s1_mway == write_cand);
            assign inst_way[pw]  = s1_refill && vict_onehot[pw] && (!way_valid[pw] || refill_ok);   // = case_inst at way pw
            assign st_we_way[pw] = (((s1_lsu_go && !tag_err_any) || s1_snp) && hit_first[pw] && way_we[pw]) ||
                                   inst_way[pw] || (case_walk && mt_we && mway_hot[pw]);
            assign st_val_way[2*pw +: 2] = (s1_kind == K_REFILL) ? q1_state :
                                           (s1_kind == K_MAINT)  ? mt_nxt : way_nxt[2*pw +: 2];
            // iter 14: = pl_we at way acc_way: a served CPU hit (s1_hit) at its first match, or the refill install at its way
            assign pl_we_way[pw] = (s1_lsu_go && !tag_err_any && hit_first[pw] && !way_upg[pw]) || inst_way[pw];
            assign pf_we_way[pw] = (s1_lsu_go && !tag_err_any && hit_first[pw] && !way_upg[pw] && m_pf_bits[pw]) ||
                                   inst_way[pw] || (s1_snp && s1_sinv && hit_first[pw]) ||
                                   (case_walk && (s1_mop == MAINT_FLUSH) && mway_hot[pw]);
        end
    endgenerate

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
    // TIMING OPT (CB-3, 2026-11-26), part b: the two compares of the clear line for l1d_reservation_unit,
    // from flops, with the late victim one-hot LAST. Before, u_rsv compared rsv_clear_line, i.e. after the
    // victim pick v_line (F3: meta read -> victim one-hot -> 50-bit tag pick -> 58-bit compare).
    // Way w's line is {tag_q of way w, s1_set, BANK_C} (as lock_match). vict_onehot is exactly one-hot
    // for every input: victim_onehot has one bit v (inv_first when a way is invalid, else the decode of
    // the PLRU victim); the lock rule keeps it at v (lock_way[v] = 0) or moves it to v ^ 1, all other
    // bits are 0. So v_line == X is |(vict_onehot & (way w's line == X)). The snoop leg is folded into
    // the same AND-OR (the way terms are 0 for a snoop):
    //   (rsv_clear_line == X) = s1_snp ? (s1_line == X) : (v_line == X), X = s2_line or the reservation line.
    // Worked example: an eviction, vict_onehot = 0100, way 2's line = s2_line and not the reservation
    //   line -> way_line_eq_s2 = 0100, way_line_eq_rsv = 0000 -> clear_eq_s2_line = 1,
    //   clear_eq_rsv_line = 0 (old: rsv_clear_line = v_line = s2_line). A snoop on line L: way terms 0,
    //   clear_eq_s2_line = (L == s2_line).
    wire [WAYS-1:0] way_line_eq_s2;                              // no snoop, and way w's line == s2_line, WAYS = 4 bits
    wire [WAYS-1:0] way_line_eq_rsv;                             // no snoop, and way w's line == reservation line, 4 bits
    wire            snoop_line_eq_s2;                            // a snoop, and s1_line == s2_line
    wire            snoop_line_eq_rsv;                           // a snoop, and s1_line == reservation line
    genvar clear_way_idx;                                        // loop variable: way
    generate
        for (clear_way_idx = 0; clear_way_idx < WAYS; clear_way_idx = clear_way_idx + 1) begin : g_clear_match
            assign way_line_eq_s2[clear_way_idx]  = !s1_snp &&
                   ({tag_q[clear_way_idx*TAG_W +: TAG_W], s1_set, BANK_C} == s2_line);                 // 58-bit compare
            assign way_line_eq_rsv[clear_way_idx] = !s1_snp &&
                   ({tag_q[clear_way_idx*TAG_W +: TAG_W], s1_set, BANK_C} == reservation_line_addr);   // 58-bit compare
        end
    endgenerate
    assign snoop_line_eq_s2  = s1_snp && (s1_line == s2_line);
    assign snoop_line_eq_rsv = s1_snp && (s1_line == reservation_line_addr);
    assign clear_eq_s2_line  = snoop_line_eq_s2  || |(vict_onehot & way_line_eq_s2);
    assign clear_eq_rsv_line = snoop_line_eq_rsv || |(vict_onehot & way_line_eq_rsv);
    // ======================================================================
    // TIMING OPT (CB-3, 2026-11-26), part c: one-hot flop copies of s2_way and s2_mid (rule T-6)
    // ======================================================================
    // s2_way (2 flops) selected the 512-bit lines, syndromes, words, AMO arms and the write way: about
    // 2300 loads per bit, 7 to 15 BUFx2 in series in the CB-2 report (F2: -352.3 ps into the data macro
    // b_wdata, -348.8 ps into b_en / b_we). Each consumer group in l1d_bank_s2 now reads its own copy
    // (copy g < GRANULES: line_q granule g and its write select; then word, ECC, control, AMO). s2_mid
    // gets one copy per granule for the install line in l1d_mshr.
    // Exact, cycle by cycle:
    //   - rst = 0: s2_way <= s1_way and copy <= s1_way_onehot; s1_way_onehot has its one 1 at s1_way for
    //     EVERY input (hit_onehot: lowest match or way 0, = hit_way; vict_onehot: exactly one-hot,
    //     = vict_way; mway_hot = decode of s1_mway), also when S1 holds no valid op (same stale value);
    //   - rst = 1: s2_way holds (enable !rst); the copy loads decode(s2_way), so it still equals it.
    //   So one edge after ANY state copy = decode(s2_way) (likewise s2_mid). No constant reset on purpose
    //   (s2_way has none). Before the first edge both are uninitialised (X in simulation).
    // (* keep *): the copies have equal D inputs; without it opt_merge folds them back into one flop
    // (same pattern as ar_arb_q / ar_c_q in l1d_mshr_entry, which the CB-2 netlist keeps as separate flops).
    // Worked example: a load hits way 2: hit_onehot = 0100, kind LSU -> s1_way_onehot = 0100; at the edge
    //   s2_way = 2 and every copy = 0100. A refill of MSHR entry 5: every s2_mid copy = 0010_0000.
    //   Reset held with s2_way = 3: every copy = 1000.
    wire [WAYS-1:0]         s1_way_onehot;                       // 1 at s1_way, WAYS = 4 bits
    wire [WAYS-1:0]         s2_way_decoded;                      // decode of the s2_way flop (reset leg, check), 4 bits
    wire [MSHR_ENTRIES-1:0] s1_mid_onehot;                       // decode of s1_mid, MSHR_ENTRIES = 8 bits
    wire [MSHR_ENTRIES-1:0] s2_mid_decoded;                      // decode of the s2_mid flop (reset leg, check), 8 bits
    assign s1_way_onehot = (s1_kind == K_REFILL) ? vict_onehot :
                           (s1_kind == K_MAINT)  ? mway_hot : hit_onehot;   // the kind select of s1_way
    genvar decode_way_idx;                                       // loop variable: way
    generate
        for (decode_way_idx = 0; decode_way_idx < WAYS; decode_way_idx = decode_way_idx + 1) begin : g_way_decode
            wire [WAY_W-1:0] way_number;                         // way number decode_way_idx, WAY_W = 2 bits
            assign way_number                     = decode_way_idx;
            assign s2_way_decoded[decode_way_idx] = (s2_way == way_number);
        end
    endgenerate
    genvar decode_entry_idx;                                     // loop variable: MSHR entry
    generate
        for (decode_entry_idx = 0; decode_entry_idx < MSHR_ENTRIES; decode_entry_idx = decode_entry_idx + 1) begin : g_mid_decode
            wire [MSHR_ID_W-1:0] entry_number;                   // entry number decode_entry_idx, MSHR_ID_W = 3 bits
            assign entry_number                     = decode_entry_idx;
            assign s1_mid_onehot[decode_entry_idx]  = (s1_mid == entry_number);
            assign s2_mid_decoded[decode_entry_idx] = (s2_mid == entry_number);
        end
    endgenerate
    // ======================================================================
    // TIMING OPT (CB-4c, 2026-11-26): flop copies of the S2 kind decode and of s2_live (rule T-6)
    // ======================================================================
    // CB-3 ASAP7 run (200 ps): s2_kind[0] QN (fan-out 7, 47.3 ps) -> INV -> OR5 ("not live", 108.1 ps) -> OR5
    // (142.5 ps) -> S2 decision -> s3_load -> lsu_response_valid -> merger FIFO: l1d_top -343.4 ps. In l1d_bank
    // alone the same decode (98.0 ps) and a 2-level buffer tree (138.3 ps) feed the store write select -> S3 AMO
    // mux -> l1d_ecc_enc -> data macro b_wdata: -319.3 ps. l1d_bank_s2 now reads these flops instead:
    //   s2_kind_onehot_copy[k] = s2_valid && (s2_kind == k)                                  (4 bits, k = K_*)
    //   s2_live_copy[c]        = s2_valid && (s2_kind == K_LSU) && !s2_req[REQ_PF_BIT] && !s2_kill  (= s2_live)
    //   live copy c < GRANULES: write select of granule c; c = GRANULES: decision tree and data ECC flags;
    //   c = GRANULES + 1: ecc_tag_blk, store_hit_wr, s3_amo_v, train_now (fan-out 2 to 4 each).
    // Exact, cycle by cycle (s1_to_s2_proc below: s2_valid resets to 0; every other S2 flop holds in reset and
    // loads its S1 flop at every edge out of reset, with no enable):
    //   - rst = 0: s2_valid <= s1_valid, s2_kind <= s1_kind, s2_req <= s1_req, s2_kill <= s1_kill, and the copies
    //     load the same functions of the same S1 flops (s1_lsu = s1_valid && s1_kind == K_LSU, s1_refill,
    //     s1_maint, s1_snp likewise; s1_pf = s1_req[REQ_PF_BIT]). S1 without a valid op: s1_valid = 0 -> every
    //     copy loads 0, and s2_valid = 0 makes every old decode 0;
    //   - rst = 1: s2_valid <= 0 and every copy <= 0; s2_kind / s2_req / s2_kill hold (X before their first load),
    //     but every old decode is ANDed with s2_valid = 0 (0 && X = 0), so it is 0 as well;
    //   - ghost (s1_kill = 1): kind bit K_LSU = 1 (as s2_lsu), live copies 0 (as s2_live); prefetch: the same.
    //   Before the first clock edge both sides are X; the check in l1d_bank_s2 runs only out of reset.
    // Only flops enter the D inputs: s1_valid, s1_kind, s1_req, s1_kill are registers of l1d_bank_r (s1_kill <=
    // r_kill, not late); one decode level and one AND. No S1 decision (s1_hit, r_held) is folded in.
    // Worked example: a load of port 1 in S1 (valid, K_LSU, not a prefetch, not cancelled): at the edge
    //   s2_kind_onehot_copy = 0001 and all 10 live copies = 1. The same load cancelled in R (s1_kill = 1): 0001
    //   and all live copies 0. A refill token: 0010 and 0. A snoop: 1000 and 0. Reset: 0000 and 0.
    (* keep *) always @(posedge clk) begin : onehot_copy_proc
        if (rst) begin
            s2_way_onehot_copy <= {S2_WAY_COPIES{s2_way_decoded}};   // reset: follow the held s2_way
            s2_mid_onehot_copy <= {GRANULES{s2_mid_decoded}};        // reset: follow the held s2_mid
            // CB-4c: s2_valid resets to 0, so every decode of it is 0
            s2_kind_onehot_copy[K_LSU]    <= 1'b0;
            s2_kind_onehot_copy[K_REFILL] <= 1'b0;
            s2_kind_onehot_copy[K_MAINT]  <= 1'b0;
            s2_kind_onehot_copy[K_SNOOP]  <= 1'b0;
            s2_live_copy                  <= {S2_LIVE_COPIES{1'b0}};   // S2_LIVE_COPIES = 10 bits
        end else begin
            s2_way_onehot_copy <= {S2_WAY_COPIES{s1_way_onehot}};    // = decode of s2_way after this edge
            s2_mid_onehot_copy <= {GRANULES{s1_mid_onehot}};         // = decode of s2_mid after this edge
            // CB-4c: bit k = s1_valid && (s1_kind == k) = s2_valid && (s2_kind == k) after this edge
            s2_kind_onehot_copy[K_LSU]    <= s1_lsu;                 // s1_valid && (s1_kind == K_LSU)
            s2_kind_onehot_copy[K_REFILL] <= s1_refill;              // s1_valid && (s1_kind == K_REFILL)
            s2_kind_onehot_copy[K_MAINT]  <= s1_maint;               // s1_valid && (s1_kind == K_MAINT)
            s2_kind_onehot_copy[K_SNOOP]  <= s1_snp;                 // s1_valid && (s1_kind == K_SNOOP)
            // CB-4c: = s2_valid && s2_kind == K_LSU && !s2_req[REQ_PF_BIT] && !s2_kill (s2_live) after this edge
            s2_live_copy                  <= {S2_LIVE_COPIES{s1_lsu && !s1_pf && !s1_kill}};
        end
    end // always onehot_copy_proc

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

`ifndef SYNTHESIS
    // CB-3 always-on check: every one-hot copy equals the decode of its binary flop
    always @(posedge clk) begin
        if (!rst && ((s2_way_onehot_copy != {S2_WAY_COPIES{s2_way_decoded}}) ||
                     (s2_mid_onehot_copy != {GRANULES{s2_mid_decoded}})))
            $display("[%0t] L1D B%0d S1 ERROR CB-3 one-hot copy differs (way %0d: %h, entry %0d: %h)",
                     $time, BANK_ID, s2_way, s2_way_onehot_copy, s2_mid, s2_mid_onehot_copy);
    end
`endif
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
