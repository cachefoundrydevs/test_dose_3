// ==========================================================================
// l1d_pf3_sms_core.v - SMS prefetcher, front end and learning core
//                      (3-cycle version: training record at the unit pins
//                      -> first request in the unit's output register
//                      three clock edges later)
// ==========================================================================
// IN SIMPLE WORDS : the same SMS (Spatial Memory Streaming) learning as
//   l1d_pf_sms_front + l1d_pf_sms_core: the first access to a 2 KB region
//   (the trigger, by PC X at line L) is noted in the FILTER table (FT); a
//   second different line moves the note to the ACCUMULATION table (AT),
//   where every access ticks a box of a 32-box footprint; when a used
//   line leaves the cache (eviction event) the footprint is filed in the
//   PATTERN HISTORY table (PHT) under (X, L); the next trigger of X at L
//   in a new region looks the footprint up and hands it to the expander.
//   What changed is the structure:
//   - the front picks a record and folds its hashes in the same cycle
//     (S1); the FIFOs are only used when the core is busy (bypass);
//   - FT, AT and PHT are compared as CAMs in S2; S2 only registers the
//     decisions and the hit / victim vectors. The FT / AT write happens
//     one cycle later (stage P, all from flops); the next event, now in
//     S2, sees that pending write through a one-level bypass (an entry
//     being set compares with the previous event's key), so it decides
//     exactly as if the table had been written at the end of S2;
//   - each PHT entry keeps its first line, the line after it (second)
//     and the rest of its footprint ready-made (computed once, in the
//     PHT write stages); the lookup registers one hit bit and the read
//     data per way (read by the set, in parallel with the compares), so
//     the footprint comes out of flops in S3 (load_*).
//   - the S1 registers that feed many compares are kept in copies
//     (l1d_pf3_rep, rule T-6).
//
// STAGES (one clock each):
//   S1  record pick (rotating lanes), PC / region / PHT hashes, event
//       select (train or eviction, alternating), FIFO bypass -> e1_*
//   S2  FT / AT / PHT compare (with the bypass of the pending write),
//       decide -> decision flops, hit / victim vectors, PHT way hits and
//       way data (flops)
//   S3  load_* to the expander: way select of the flops (combinational)
//   P   (= the next event's S2) FT / AT write from the flops; read of the
//       PHT write data (AT entry ended or replaced) -> w_*
//   W1  PHT write, part 1: same-tag match per entry, first and second
//       line and rest of the footprint (from w_* flops) -> x_*
//   W2  PHT write, part 2: way pick (same tag, free, ring) and write
//   P / W1 / W2 are a background write, not on the 3-cycle request path;
//   a trigger in the cycle of a PHT write sees the table before that
//   write (as a later event in the old design, see README D-list).
//
// INSTANTIATED BY : l1d_prefetch_unit, instance u_sms_core, 1 copy.
//
// PARAMETERS:
//   name              default unit     meaning
//   NB                4       lanes    training lanes = banks
//   PC_W              64      bits     instruction PC
//   ADDR_W            64      bits     cache byte address
//   PA_W              56      bits     physical address bits used
//   LINE_BYTES        64      bytes    cache line
//   LINE_W            58      bits     cache line address (eviction port)
//   REGION_BYTES      2048    bytes    region (<= 4 KB)
//   FT_ENTRIES        16      entries  filter table
//   AT_ENTRIES        32      entries  accumulation table
//   PHT_SETS          32      sets     pattern history sets
//   PHT_WAYS          4       ways     pattern history ways
//   REGION_TAG_W      16      bits     region tag (XOR fold)
//   PC_HASH_W         12      bits     PC hash
//   PHT_TAG_W         8       bits     PHT tag
//   KEY_COPIES        8       copies   S1 copies of the PHT key (T-6)
//   TAG_COPIES        4       copies   S1 copies of region tag + line
//   TRAIN_FIFO_DEPTH  4       records  training FIFO (power of two)
//   EVT_FIFO_DEPTH    4       records  eviction FIFO (power of two)
//
// PORTS (default widths; b = lane):
//   clk, rst, pf_enable  in  1        clock, reset, on / off
//   train_valid          in  4        training record per lane
//   train_no_train       in  4        do not train
//   train_pc             in  4x64     PC
//   train_addr           in  4x64     byte address
//   evict_valid          in  4        a line left bank b
//   evict_line_addr      in  4x58     its line address
//   load_valid           out 1        S3: a trigger found a footprint
//   load_region          out 45       S3: its region number (flops)
//   load_line            out 5        S3: trigger line in the region
//   load_first           out 5        S3: first line to fetch
//   load_first_bank      out 4        S3: its bank, one-hot
//   load_second          out 5        S3: the line after the first
//   load_rest            out 32       S3: the other lines to fetch
//   (S3 outputs: a 4-way select of flops, 2 levels)
//
// WORKED EXAMPLE (README section 10, PC hash X, lines {0, 3, 6}):
//   X line 0 region R1 -> trigger, FT; line 3 -> AT {0, 3}; line 6 -> AT
//   {0, 3, 6}; evict line 3 -> PHT[(X, 0)]: rest {3, 6}, first 3. Later X
//   line 0 region R2 at the pins in cycle t: S2 (t+1) hits the PHT (way
//   flops), S3 (t+2) load_first = 3 -> the expander's offer; rank sends
//   line R2:3 at the end of t+2, pf_req_valid is 1 in cycle t+3; line 6
//   (load_second) follows from the expander's slot.
//
// LOGIC LEVELS (T-9, estimates): S1 hash folds 5 + lane select 2 +
//   FIFO head 1 + take_evt 2:1 1 = 9 (the lane pick is valid AND-OR with
//   the registered before-masks of the rotation, 3, in parallel; the
//   training and the eviction side are built apart and take_evt, from
//   the record valids, picks last through kept 2:1 selects, one take_evt
//   copy per select group); S2 CAM 4 + per-case bypass 1 + any 3, then the
//   previous decision flops pick the case 2 + decide 2 = 12 to one
//   decision flop (the _none / _keep flops: CAM 4 + bypass 1 + one OR of
//   the hit vectors 4 + e1_evt 2:1 1 = 10; the late decision flops pass
//   only the last 4); PHT compare 4 + set AND-OR 4 = 8 to a way-hit
//   flop; way data: set AND-OR 5; S3 way select 2; P write 2, read 5
//   (the promoted line's one-hot: each FT line is decoded from its
//   flops, then picked by ft_hit_q, so the decode is off the late path;
//   the trigger line is read for both entries, end and victim, and
//   dend_q picks after the masks);
//   W1 tag compare 4 or first / second line prefix 6 + select 2 + encode
//   2; W2 way pick 4 (l1d_pf3_sms_way_pick, 3 kept copies per set, each
//   driving one group of the data flops, rule T-6) + write 1.
//
// ASSUMPTIONS (V-9.2): A1 sizes powers of two, FT/AT >= 2. A2 one event
//   per cycle; the PHT is a hint: a lookup in the cycle of a PHT write
//   sees the old pattern. A3 a promote needs a different second line
//   (open SM-14). A4 FT / AT replacement without a free slot uses a ring
//   pointer (open SM-15). A5 a valid PHT tag is in at most one way of its
//   set (a write of a tag that is present goes to that way; checked under
//   L1D_DEBUG).
// DIFFERENCES FROM l1d_pf_sms (no function removed):
//   (the PHT write is one cycle later than in the first 3-cycle version:
//   a hint, A2)
//   D1 a PHT entry also keeps its trigger line; a lookup needs it to be
//      equal (the 9-stage version accepted an aliased entry of another
//      line and masked it with the event's line: fewer false patterns).
//   D2 the training / eviction FIFOs are bypassed when empty.
// ==========================================================================
`ifndef L1D_PF3_SMS_CORE_V
`define L1D_PF3_SMS_CORE_V
`default_nettype none

module l1d_pf3_sms_core #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter NB               = 4,    // default 4 lanes
    parameter PC_W             = 64,   // default 64 bits: PC
    parameter ADDR_W           = 64,   // default 64 bits: cache address
    parameter PA_W             = 56,   // default 56 bits: physical address
    parameter LINE_BYTES       = 64,   // default 64 bytes: line
    parameter LINE_W           = 58,   // default 58 bits: cache line addr
    parameter REGION_BYTES     = 2048, // default 2048 bytes: region
    parameter FT_ENTRIES       = 16,   // default 16: filter table
    parameter AT_ENTRIES       = 32,   // default 32: accumulation table
    parameter PHT_SETS         = 32,   // default 32: PHT sets
    parameter PHT_WAYS         = 4,    // default 4: PHT ways
    parameter REGION_TAG_W     = 16,   // default 16 bits: region tag
    parameter PC_HASH_W        = 12,   // default 12 bits: PC hash
    parameter PHT_TAG_W        = 8,    // default 8 bits: PHT tag
    parameter TRAIN_FIFO_DEPTH = 4,    // default 4: training FIFO
    parameter EVT_FIFO_DEPTH   = 4,    // default 4: eviction FIFO
    parameter KEY_COPIES       = 8,    // default 8: PHT key copies
    parameter TAG_COPIES       = 4     // default 4: region tag copies
) (
    // ---- clock and reset ----
    input  wire                      clk,             // clock
    input  wire                      rst,             // sync reset, high
    input  wire                      pf_enable,       // 1 = on
    // ---- training records and eviction events ----
    input  wire [NB-1:0]             train_valid,     // 4
    input  wire [NB-1:0]             train_no_train,  // 4
    input  wire [NB*PC_W-1:0]        train_pc,        // 4 x 64
    input  wire [NB*ADDR_W-1:0]      train_addr,      // 4 x 64
    input  wire [NB-1:0]             evict_valid,     // 4
    input  wire [NB*LINE_W-1:0]      evict_line_addr, // 4 x 58
    // ---- to the expander (S3, from flops) ----
    output wire                      load_valid,      // footprint found
    output wire [PA_W-$clog2(LINE_BYTES)-$clog2(REGION_BYTES/LINE_BYTES)-1:0]
                                     load_region,     // 45 bits
    output wire [$clog2(REGION_BYTES/LINE_BYTES)-1:0] load_line,  // 5 bits
    output wire [$clog2(REGION_BYTES/LINE_BYTES)-1:0] load_first, // 5 bits
    output wire [NB-1:0]             load_first_bank, // 4: one-hot
    output wire [$clog2(REGION_BYTES/LINE_BYTES)-1:0] load_second, // 5 bits
    output wire [(REGION_BYTES/LINE_BYTES)-1:0] load_rest // 32 bits
);

    // ---- local constants (value at the default) ----
    localparam OFFSET_W     = $clog2(LINE_BYTES);               // 6 bits
    localparam PLINE_W      = PA_W - OFFSET_W;                  // 50 bits
    localparam LIR_W        = $clog2(REGION_BYTES / LINE_BYTES);// 5 bits
    localparam REGION_NUM_W = PLINE_W - LIR_W;                  // 45 bits
    localparam PATTERN_W    = (1 << LIR_W);                     // 32 bits
    localparam KEY_W        = PC_HASH_W + LIR_W;                // 17 bits
    localparam SET_W        = $clog2(PHT_SETS);                 // 5 bits
    localparam WAY_W        = $clog2(PHT_WAYS);                 // 2 bits
    localparam FT_IDX_W     = $clog2(FT_ENTRIES);               // 4 bits
    localparam AT_IDX_W     = $clog2(AT_ENTRIES);               // 5 bits
    localparam PHT_TOTAL    = PHT_SETS * PHT_WAYS;              // 128
    localparam PORT_W       = $clog2(NB);                       // 2 bits
    localparam BANK_W       = $clog2(NB);                       // 2 bits
    // a training record in the FIFO: {region, line, tag, set, ptag}
    localparam TREC_W = REGION_NUM_W + LIR_W + REGION_TAG_W + SET_W
                        + PHT_TAG_W;                            // 79 bits
    // an eviction event in the FIFO: {region, line, tag}
    localparam EREC_W = REGION_NUM_W + LIR_W + REGION_TAG_W;    // 66 bits
    localparam KEY_PER      = PHT_TOTAL / KEY_COPIES;           // 16 entries
    localparam KEY_CW       = PHT_TAG_W + LIR_W + SET_W;        // 18 bits
    localparam TAG_CW       = REGION_TAG_W + LIR_W;             // 21 bits
    localparam FT_PER       = FT_ENTRIES / TAG_COPIES;          // 4 entries
    localparam AT_PER       = AT_ENTRIES / TAG_COPIES;          // 8 entries
    localparam LOH_COPIES   = TAG_COPIES;                       // 4 copies
    localparam SEL_W        = REGION_NUM_W + LIR_W + REGION_TAG_W
                              + PATTERN_W;                      // 98 bits
    localparam WP_COPIES    = 3;      // way pick copies per PHT set: tag
                                      //   and small fields / pattern low
                                      //   half / pattern high half
    localparam PAT_HALF     = PATTERN_W / 2;                    // 16 bits
    localparam TE_N         = 4;      // copies of take_evt: 3 select groups
                                      //   (region / line + tag / one-hot)
                                      //   and 1 for the FIFO controls

    // ---- S1: lanes ----
    reg  [NB*REGION_TAG_W-1:0] lane_rtag;   // 4 x 16: region tag
    reg  [NB*SET_W-1:0]      lane_set;      // 4 x 5: PHT set
    reg  [NB*PHT_TAG_W-1:0]  lane_ptag;     // 4 x 8: PHT tag
    reg  [NB*REGION_TAG_W-1:0] evl_rtag;    // 4 x 16: eviction region tag
    reg  [KEY_W-1:0]         lane_key;      // scratch: {PC hash, line}
    reg  [PC_HASH_W-1:0]     pch_one;       // scratch
    reg  [REGION_NUM_W-1:0]  region_one;    // scratch
    wire [NB-1:0]            tr_ok;         // 4: lane may train
    wire [NB-1:0]            ev_ok;         // 4: bank has an eviction
    reg  [NB-1:0]            tr_pick;       // 4: picked lane, one-hot
    reg  [NB-1:0]            ev_pick;       // 4: picked eviction, one-hot
    // per lane, the lanes before it in the rotated order (flops, made one
    // cycle ahead from the next rotation): the pick is then one AND-OR
    reg  [NB*NB-1:0]         tr_before_q;   // 4 x 4: lane i at [i*NB +: NB]
    reg  [NB*NB-1:0]         ev_before_q;   // 4 x 4
    reg  [NB*NB-1:0]         tr_before_nx;  // 4 x 4: next value
    reg  [NB*NB-1:0]         ev_before_nx;  // 4 x 4: next value
    reg  [NB*NB-1:0]         rst_before;    // 4 x 4: masks at reset (lane 0
                                            //   first; constant)
    reg  [NB-1:0]            train_rot_nx;  // 4: next rotation, one-hot
    reg  [NB-1:0]            evt_rot_nx;    // 4: next rotation, one-hot
    reg  [PORT_W-1:0]        lane_pos;      // scratch: lane's place in order
    reg  [PORT_W-1:0]        other_pos;     // scratch: other lane's place
    reg  [TREC_W-1:0]        tr_word;       // the picked training record
    reg  [EREC_W-1:0]        ev_word;       // the picked eviction event
    wire                     tr_fresh;      // a training record now
    wire                     ev_fresh;      // an eviction event now
    reg  [NB-1:0]            train_rot_oh;  // 4: first lane tried, one-hot
    reg  [NB-1:0]            evt_rot_oh;    // 4: first bank tried, one-hot
    reg  [NB*PATTERN_W-1:0]  lane_loh;      // 4 x 32: training line, one-hot
    reg  [NB*PATTERN_W-1:0]  evl_loh;       // 4 x 32: eviction line, one-hot

    // ---- S1: FIFOs and event select ----
    wire                     tq_vld;        // training FIFO has a record
    wire [TREC_W-1:0]        tq_head;       // its head
    wire                     eq_vld;        // eviction FIFO has a record
    wire [EREC_W-1:0]        eq_head;       // its head
    wire                     tr_avail;      // a training record waits
    wire                     ev_avail;      // an eviction event waits
    wire                     take_evt;      // the core takes an eviction
    wire                     take_tr;       // the core takes a record
    wire                     tq_push;       // fresh record into the FIFO
    wire                     eq_push;       // fresh event into the FIFO
    wire                     tq_pop;        // FIFO head taken
    wire                     eq_pop;        // FIFO head taken
    wire [PATTERN_W-1:0]     tq_loh;        // 32: FIFO head line, one-hot
    wire [PATTERN_W-1:0]     eq_loh;        // 32: FIFO head line, one-hot
    reg  [SEL_W-1:0]         tr_side;       // 98: {region, line, rtag,
                                            //   line one-hot} if a record
                                            //   is taken
    reg  [SEL_W-1:0]         ev_side;       // 98: the same if an eviction
                                            //   is taken
    wire [SEL_W-1:0]         nx_side;       // 98: the taken one
    wire [TE_N-1:0]          tr_fresh_cp;   // 4: copies of |tr_ok
    wire [TE_N-1:0]          ev_fresh_cp;   // 4: copies of |ev_ok
    wire [TE_N-1:0]          take_evt_cp;   // 4: copies of take_evt (one
                                            //   per select group)
    reg  [SET_W-1:0]         nx_set_r;      // 5
    reg  [PHT_TAG_W-1:0]     nx_ptag_r;     // 8
    reg                      prefer_evt_q;  // alternate when both wait

    // ---- S1 -> S2 registers ----
    reg                      e1_vld;        // event valid
    reg                      e1_evt;        // is an eviction
    reg  [REGION_NUM_W-1:0]  e1_region;     // 45 bits
    reg  [LIR_W-1:0]         e1_line;       // 5 bits
    reg  [REGION_TAG_W-1:0]  e1_rtag;       // 16 bits
    reg  [SET_W-1:0]         e1_set;        // 5 bits
    reg  [PHT_TAG_W-1:0]     e1_ptag;       // 8 bits
    wire [REGION_NUM_W-1:0]  nx_region;     // 45: next S1 value
    wire [LIR_W-1:0]         nx_line;       // 5
    wire [REGION_TAG_W-1:0]  nx_rtag;       // 16
    wire [SET_W-1:0]         nx_set;        // 5
    wire [PHT_TAG_W-1:0]     nx_ptag;       // 8
    wire [PATTERN_W-1:0]     nx_line_oh;    // 32: next line, one-hot
    // copies (T-6): PHT key {ptag, line, set}, {region tag, line}, line
    wire [KEY_COPIES*KEY_CW-1:0]    kc_word; // 8 x 18
    wire [TAG_COPIES*TAG_CW-1:0]    tc_word; // 4 x 21
    wire [LOH_COPIES*PATTERN_W-1:0] lc_oh;   // 4 x 32

    // ---- tables ----
    reg  [FT_ENTRIES-1:0]               ft_vld;   // 16
    reg  [FT_ENTRIES*REGION_TAG_W-1:0]  ft_tag;   // 16 x 16
    reg  [FT_ENTRIES*LIR_W-1:0]         ft_line;  // 16 x 5: trigger line
    reg  [FT_ENTRIES*SET_W-1:0]         ft_pset;  // 16 x 5: PHT set
    reg  [FT_ENTRIES*PHT_TAG_W-1:0]     ft_ptag;  // 16 x 8: PHT tag
    reg  [FT_IDX_W-1:0]                 ft_ring;  // 4: replace pointer
    reg  [AT_ENTRIES-1:0]               at_vld;   // 32
    reg  [AT_ENTRIES*REGION_TAG_W-1:0]  at_tag;   // 32 x 16
    reg  [AT_ENTRIES*LIR_W-1:0]         at_line;  // 32 x 5
    reg  [AT_ENTRIES*SET_W-1:0]         at_pset;  // 32 x 5
    reg  [AT_ENTRIES*PHT_TAG_W-1:0]     at_ptag;  // 32 x 8
    reg  [AT_ENTRIES*PATTERN_W-1:0]     at_pat;   // 32 x 32: footprint
    reg  [AT_IDX_W-1:0]                 at_ring;  // 5: replace pointer
    reg  [PHT_TOTAL-1:0]                pht_vld;  // 128
    reg  [PHT_TOTAL*PHT_TAG_W-1:0]      pht_tag;  // 128 x 8
    reg  [PHT_TOTAL*LIR_W-1:0]          pht_line; // 128 x 5: trigger line
    reg  [PHT_TOTAL*PATTERN_W-1:0]      pht_rest; // 128 x 32: other lines
    reg  [PHT_TOTAL*LIR_W-1:0]          pht_first;// 128 x 5: first line
    reg  [PHT_TOTAL*LIR_W-1:0]          pht_second;// 128 x 5: line after it
    reg  [PHT_TOTAL*NB-1:0]             pht_fbank;// 128 x 4: its bank
    reg  [PHT_TOTAL-1:0]                pht_nz;   // 128: lines to fetch
    reg  [PHT_SETS*WAY_W-1:0]           pht_ptr;  // 32 x 2: ring way

    // ---- P: the write decided by the event before (flops) ----
    reg                      dtrig_q;       // trigger (FT set, PHT lookup)
    // the S3 offer valid from its own two flops (dtrig_q carries the FT /
    // AT decision fanout; the offer goes on to the rank, rule T-6)
    reg                      dtrig_ld_q;    // trigger: copy for load_valid
    reg                      way_any_q;     // a PHT way of the set matched
    reg                      dprom_q;       // promote FT -> AT
    reg                      ddisc_q;       // FT note dropped
    reg                      dend_q;        // generation ended
    reg                      dtick_q;       // AT box ticked
    reg                      dfclr_q;       // FT entry cleared (promote
                                            //   or discard)
    reg                      dft_none_q;    // no FT write
    reg                      dat_keep_q;    // no AT set / clear
    reg                      dat_none_q;    // no AT write at all
    reg  [FT_ENTRIES-1:0]    ft_hit_q;      // 16: FT hit of that event
    reg  [FT_ENTRIES-1:0]    ft_vic_q;      // 16: FT victim
    reg  [AT_ENTRIES-1:0]    at_hit_q;      // 32: AT hit
    reg  [AT_ENTRIES-1:0]    at_vic_q;      // 32: AT victim
    reg  [REGION_TAG_W-1:0]  p_rtag;        // 16: its region tag
    reg  [LIR_W-1:0]         p_line;        // 5: its line
    reg  [SET_W-1:0]         p_set;         // 5: its PHT set
    reg  [PHT_TAG_W-1:0]     p_ptag;        // 8: its PHT tag
    reg  [PATTERN_W-1:0]     p_line_oh;     // 32: its line, one-hot
    wire [FT_ENTRIES-1:0]    p_ft_set;      // 16: FT entry set now
    wire [FT_ENTRIES-1:0]    p_ft_clr;      // 16: FT entry cleared now
    wire [AT_ENTRIES-1:0]    p_at_set;      // 32: AT entry set now
    wire [AT_ENTRIES-1:0]    p_at_clr;      // 32: AT entry cleared now
    wire [AT_ENTRIES-1:0]    p_at_tick;     // 32: AT box ticked now
    reg  [LIR_W-1:0]         pf_line;       // 5: FT read for the promote
    reg  [SET_W-1:0]         pf_pset;       // 5
    reg  [PHT_TAG_W-1:0]     pf_ptag;       // 8
    reg  [PATTERN_W-1:0]     pf_line_oh;    // 32: promoted line, one-hot
                                            //   (each FT line decoded from
                                            //   its flops, then picked)
    reg  [AT_ENTRIES-1:0]    w_rd_sel;      // 32: AT entry read for the PHT
    reg                      w_rd_vld;      // that entry holds a pattern
    reg  [LIR_W-1:0]         w_rd_line;     // 5
    reg  [SET_W-1:0]         w_rd_pset;     // 5
    reg  [PHT_TAG_W-1:0]     w_rd_ptag;     // 8
    reg  [PATTERN_W-1:0]     w_rd_pat;      // 32
    reg  [PATTERN_W-1:0]     w_rd_line_oh;  // 32: its trigger line, one-hot
    reg  [PATTERN_W-1:0]     w_rd_high;     // 32: lines above the trigger
    reg  [LIR_W-1:0]         p_bit_number;  // scratch: P mask bit
    reg  [LIR_W-1:0]         w_end_line;    // 5: line of the AT hit (end)
    reg  [LIR_W-1:0]         w_vic_line;    // 5: line of the AT victim

    // ---- S2 ----
    wire                     same_rtag;     // region tag == E-1's
    wire                     same_line;     // line == E-1's
    wire                     same_pfline;   // line == the promoted line
    wire [FT_ENTRIES-1:0]    ft_vld_e;      // 16: FT valid after P
    wire [AT_ENTRIES-1:0]    at_vld_e;      // 32: AT valid after P
    // the CAM is built for every kind of pending write (from flops), and
    // the decision flops of the event before pick the result last (T-9):
    //   FT: _t = after a trigger (set), _c = after a clear, _o = no write
    //   AT: _p = after a promote (set), _e = after an end (clear),
    //       _k = after a tick, _o = no write
    reg  [FT_ENTRIES-1:0]    ft_raw;        // 16: table hit, no bypass
    reg  [FT_ENTRIES-1:0]    ft_leq_raw;    // 16: table line == line
    reg  [FT_ENTRIES-1:0]    ft_hit_t;      // 16: hit after a trigger
    reg  [FT_ENTRIES-1:0]    ft_leq_t;      // 16: line == after a trigger
    reg  [FT_ENTRIES-1:0]    ft_hit_c;      // 16: hit after a clear
    reg  [AT_ENTRIES-1:0]    at_raw;        // 32: table hit, no bypass
    reg  [AT_ENTRIES-1:0]    at_has_raw;    // 32: table pattern has line
    reg  [AT_ENTRIES-1:0]    at_hit_p;      // 32: hit after a promote
    reg  [AT_ENTRIES-1:0]    at_has_p;      // 32: has line after a promote
    reg  [AT_ENTRIES-1:0]    at_hit_e;      // 32: hit after an end
    reg  [AT_ENTRIES-1:0]    at_has_k;      // 32: has line after a tick
    wire [FT_ENTRIES-1:0]    ft_hit;        // 16: the FT hit (picked)
    wire [AT_ENTRIES-1:0]    at_hit;        // 32: the AT hit (picked)
    wire                     any_ft;        // region in the FT
    wire                     any_at;        // region in the AT
    wire                     ft_same_line;  // FT hit entry has this line
    wire                     at_rd_has;     // AT hit entry has this line
    reg  [FT_ENTRIES-1:0]    ft_vic;        // 16: FT victim
    reg  [AT_ENTRIES-1:0]    at_vic;        // 32: AT victim
    wire [FT_ENTRIES-1:0]    ft_free_one;   // 16: first free FT slot
    wire [AT_ENTRIES-1:0]    at_free_one;   // 32: first free AT slot
    wire                     ft_free_seen;  // a free FT slot exists
    wire                     at_free_seen;  // a free AT slot exists
    // the free-slot search for every kind of pending write
    wire [FT_ENTRIES-1:0]    ft_free_t;     // 16: first free, trigger
    wire [FT_ENTRIES-1:0]    ft_free_c;     // 16: first free, clear
    wire [FT_ENTRIES-1:0]    ft_free_o;     // 16: first free, no write
    wire                     ft_seen_t;     // a free FT slot, trigger
    wire                     ft_seen_c;     // a free FT slot, clear
    wire                     ft_seen_o;     // a free FT slot, no write
    wire [AT_ENTRIES-1:0]    at_free_p;     // 32: first free, promote
    wire [AT_ENTRIES-1:0]    at_free_e;     // 32: first free, end
    wire [AT_ENTRIES-1:0]    at_free_o;     // 32: first free, no set/clear
    wire                     at_seen_p;     // a free AT slot, promote
    wire                     at_seen_e;     // a free AT slot, end
    wire                     at_seen_o;     // a free AT slot, no set/clear
    wire                     do_promote;    // FT -> AT
    wire                     do_trigger;    // new region
    wire                     do_end_gen;    // file the footprint
    wire                     do_ft_discard; // drop the FT note
    wire [FT_ENTRIES-1:0]    ft_same_v;     // 16: FT hit on the same line
    wire                     at_or_same;    // AT hit or FT same-line hit
    wire                     end_none;      // eviction: no footprint end
    wire                     dft_none_nx;   // next dft_none_q
    wire                     dat_keep_nx;   // next dat_keep_q
    wire                     dat_none_nx;   // next dat_none_q
    wire                     dtick_nx;      // next dtick_q
    reg  [PHT_SETS-1:0]      e1_set1h;      // 32: the event's set
    reg  [PHT_TOTAL-1:0]     pht_match;     // 128: entry matches the key
    reg  [PHT_WAYS-1:0]      way_hit;       // 4: per way, the set matches
    reg  [PHT_WAYS*LIR_W-1:0] way_first;    // 4 x 5: per way read
    reg  [PHT_WAYS*LIR_W-1:0] way_second;   // 4 x 5
    reg  [PHT_WAYS*NB-1:0]   way_fbank;     // 4 x 4
    reg  [PHT_WAYS*PATTERN_W-1:0] way_rest; // 4 x 32

    // ---- S2 -> S3 registers (lookup) ----
    reg  [PHT_WAYS-1:0]      way_hit_q;     // 4
    reg  [PHT_WAYS*LIR_W-1:0] way_first_q;  // 4 x 5
    reg  [PHT_WAYS*LIR_W-1:0] way_second_q; // 4 x 5
    reg  [PHT_WAYS*NB-1:0]   way_fbank_q;   // 4 x 4
    reg  [PHT_WAYS*PATTERN_W-1:0] way_rest_q; // 4 x 32
    reg  [REGION_NUM_W-1:0]  e2_region;     // 45
    reg  [LIR_W-1:0]         e2_line;       // 5
    reg  [LIR_W-1:0]         s3_first;      // 5: S3 way select
    reg  [LIR_W-1:0]         s3_second;     // 5
    reg  [NB-1:0]            s3_fbank;      // 4
    reg  [PATTERN_W-1:0]     s3_rest;       // 32

    // ---- W1: PHT write, first stage (way match, first line) ----
    reg                      w_vld;         // a write is waiting (W1)
    reg  [SET_W-1:0]         w_set;         // 5
    reg  [PHT_TAG_W-1:0]     w_ptag;        // 8
    reg  [LIR_W-1:0]         w_line;        // 5: trigger line
    reg  [PATTERN_W-1:0]     w_pat;         // 32: footprint
    reg  [PHT_SETS-1:0]      w_set1h;       // 32: W1 set, one-hot
    reg  [PHT_TOTAL-1:0]     w_tmatch;      // 128: entry holds the same tag
    reg  [PATTERN_W-1:0]     w_line_oh;     // 32: trigger line, one-hot (P)
    reg  [PATTERN_W-1:0]     w_high;        // 32: lines above it (P flops)
    reg  [PATTERN_W-1:0]     w_all;         // 32: footprint without trigger
    reg  [PATTERN_W-1:0]     w_above;       // 32: footprint above trigger
    wire [PATTERN_W-1:0]     w_ffs_above;   // 32: lowest bit of w_above
    wire [PATTERN_W-1:0]     w_ffs2_above;  // 32: second lowest of w_above
    wire [PATTERN_W-1:0]     w_below;       // 32: footprint below trigger
    wire [PATTERN_W-1:0]     w_ffs_below;   // 32: lowest bit of w_below
    wire [PATTERN_W-1:0]     w_ffs2_below;  // 32: second lowest of w_below
    reg  [PATTERN_W-1:0]     w_second_oh;   // 32: second line, one-hot
    reg  [PATTERN_W-1:0]     w_first_oh;    // 32: the first line, one-hot
    wire                     w_seen_a;      // w_above has a line
    wire                     w_two_a;       // w_above has two lines
    wire                     w_seen_b;      // w_below has a line
    reg  [LIR_W-1:0]         w1_bit_number; // scratch: W2 encode bit
    reg  [BANK_W-1:0]        w1_bank_idx;   // scratch: W2 bank of a line
    reg  [LIR_W-1:0]         w2_bit_number; // scratch: W2 second encode bit
    // ---- W2: PHT write, second stage (way pick, second line, write) ----
    reg                      x_vld;         // a write in W2
    reg  [PHT_SETS-1:0]      x_set_go;      // 32: set written now (W1
                                            //   valid AND set, registered)
    reg  [PHT_TAG_W-1:0]     x_ptag;        // 8
    reg  [LIR_W-1:0]         x_line;        // 5
    reg  [PATTERN_W-1:0]     x_rest;        // 32: footprint without trigger
                                            //     and first line
    reg  [PATTERN_W-1:0]     x_first_oh;    // 32: first line, one-hot (W1)
    reg  [PATTERN_W-1:0]     x_second_oh;   // 32: second line, one-hot (W1)
    reg  [LIR_W-1:0]         x_first;       // 5: encoded in W2
    reg  [NB-1:0]            x_fbank;       // 4: its bank (W2)
    reg                      x_nz;          // a line besides the trigger
    reg  [PHT_TOTAL-1:0]     x_tmatch;      // 128: same tag (from W1)
    wire [WP_COPIES*PHT_TOTAL-1:0] x_write_cp; // 3 x 128: entry written
                                            //   now, one copy per flop group
    wire [PHT_TOTAL-1:0]     x_write;       // 128: copy 0 (valid, tag,
                                            //   lines, bank, forwarding)
    wire [PHT_SETS-1:0]      x_adv;         // 32: ring pointer moves
    reg  [LIR_W-1:0]         x_second;      // 5: second line (W2)

    // ======================================================================
    // S1: hashes per lane (XOR folds, as l1d_pf_sms_front / _core)
    // ======================================================================
    always @* begin : hash_proc
        integer lane_idx; // walks the lanes
        integer bit_idx; // walks the fold bits
        lane_rtag = {(NB*REGION_TAG_W){1'b0}};
        lane_set  = {(NB*SET_W){1'b0}};
        lane_ptag = {(NB*PHT_TAG_W){1'b0}};
        evl_rtag  = {(NB*REGION_TAG_W){1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            // PC hash: PC folded to PC_HASH_W
            pch_one = {PC_HASH_W{1'b0}};
            for (bit_idx = 0; bit_idx < PC_W; bit_idx = bit_idx + 1) begin
                pch_one[bit_idx % PC_HASH_W] = pch_one[bit_idx % PC_HASH_W]
                    ^ train_pc[lane_idx*PC_W + bit_idx];
            end // for bit_idx
            // region tag: region number folded to REGION_TAG_W
            region_one = train_addr[lane_idx*ADDR_W + OFFSET_W + LIR_W +: REGION_NUM_W];
            for (bit_idx = 0; bit_idx < REGION_NUM_W; bit_idx = bit_idx + 1) begin
                lane_rtag[lane_idx*REGION_TAG_W + (bit_idx % REGION_TAG_W)] =
                    lane_rtag[lane_idx*REGION_TAG_W + (bit_idx % REGION_TAG_W)]
                    ^ region_one[bit_idx];
            end // for bit_idx
            // PHT set and tag: the key {PC hash, line} folded
            lane_key = {pch_one, train_addr[lane_idx*ADDR_W + OFFSET_W +: LIR_W]};
            for (bit_idx = 0; bit_idx < KEY_W; bit_idx = bit_idx + 1) begin
                lane_set[lane_idx*SET_W + (bit_idx % SET_W)] =
                    lane_set[lane_idx*SET_W + (bit_idx % SET_W)]
                    ^ lane_key[bit_idx];
                // shifted by 3 so the tag is not a copy of the set bits
                lane_ptag[lane_idx*PHT_TAG_W + ((bit_idx + 3) % PHT_TAG_W)] =
                    lane_ptag[lane_idx*PHT_TAG_W + ((bit_idx + 3) % PHT_TAG_W)]
                    ^ lane_key[bit_idx];
            end // for bit_idx
            // eviction region tag
            region_one = evict_line_addr[lane_idx*LINE_W + LIR_W +: REGION_NUM_W];
            for (bit_idx = 0; bit_idx < REGION_NUM_W; bit_idx = bit_idx + 1) begin
                evl_rtag[lane_idx*REGION_TAG_W + (bit_idx % REGION_TAG_W)] =
                    evl_rtag[lane_idx*REGION_TAG_W + (bit_idx % REGION_TAG_W)]
                    ^ region_one[bit_idx];
            end // for bit_idx
        end // for lane_idx
    end // always hash_proc

    // ======================================================================
    // S1: pick one record and one event (first valid from a rotating lane)
    // ======================================================================
    // the pick and the picked data do not wait for pf_enable (a fanout-
    // heavy unit pin); pf_enable gates only "a fresh record" (push, take,
    // e1_vld), so with it off nothing picked is kept (rule T-6)
    assign tr_ok    = train_valid & ~train_no_train;
    assign ev_ok    = evict_valid;
    assign tr_fresh = (|tr_ok) & pf_enable;
    assign ev_fresh = (|ev_ok) & pf_enable;

    always @* begin : pick_proc
        integer lane_idx;  // the lane decided
        // lane i is picked when it is valid and no lane before it in the
        // rotated order is (before-masks are flops: one AND-OR, T-9)
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            tr_pick[lane_idx] = tr_ok[lane_idx]
                & ~(|(tr_ok & tr_before_q[lane_idx*NB +: NB]));
            ev_pick[lane_idx] = ev_ok[lane_idx]
                & ~(|(ev_ok & ev_before_q[lane_idx*NB +: NB]));
        end // for lane_idx
        // the picked records (FIFO data) and the lines as one-hots (decoded
        // before the select, off the select path)
        tr_word = {TREC_W{1'b0}};
        ev_word = {EREC_W{1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_loh[lane_idx*PATTERN_W +: PATTERN_W] =
                {{(PATTERN_W-1){1'b0}}, 1'b1}
                << train_addr[lane_idx*ADDR_W + OFFSET_W +: LIR_W];
            evl_loh[lane_idx*PATTERN_W +: PATTERN_W] =
                {{(PATTERN_W-1){1'b0}}, 1'b1}
                << evict_line_addr[lane_idx*LINE_W +: LIR_W];
            tr_word = tr_word | ({TREC_W{tr_pick[lane_idx]}} & {
                train_addr[lane_idx*ADDR_W + OFFSET_W + LIR_W +: REGION_NUM_W],
                train_addr[lane_idx*ADDR_W + OFFSET_W +: LIR_W],
                lane_rtag[lane_idx*REGION_TAG_W +: REGION_TAG_W],
                lane_set[lane_idx*SET_W +: SET_W],
                lane_ptag[lane_idx*PHT_TAG_W +: PHT_TAG_W]});
            ev_word = ev_word | ({EREC_W{ev_pick[lane_idx]}} & {
                evict_line_addr[lane_idx*LINE_W + LIR_W +: REGION_NUM_W],
                evict_line_addr[lane_idx*LINE_W +: LIR_W],
                evl_rtag[lane_idx*REGION_TAG_W +: REGION_TAG_W]});
        end // for lane_idx
    end // always pick_proc

    // next rotation (first lane tried) and, from it, each lane's
    // before-mask: lane k is before lane i when (k - first) mod NB is
    // smaller than (i - first) mod NB
    always @* begin : before_next_proc
        integer lane_idx;  // the lane whose mask is made
        integer other_idx; // a lane that may be before it
        integer start_idx; // first lane of the next rotation
        train_rot_nx = {train_rot_oh[NB-2:0], train_rot_oh[NB-1]};
        evt_rot_nx   = {evt_rot_oh[NB-2:0], evt_rot_oh[NB-1]};
        tr_before_nx = {(NB*NB){1'b0}};
        ev_before_nx = {(NB*NB){1'b0}};
        rst_before   = {(NB*NB){1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            for (other_idx = 0; other_idx < NB; other_idx = other_idx + 1) begin
                for (start_idx = 0; start_idx < NB; start_idx = start_idx + 1) begin
                    lane_pos  = lane_idx[PORT_W-1:0] - start_idx[PORT_W-1:0];
                    other_pos = other_idx[PORT_W-1:0] - start_idx[PORT_W-1:0];
                    if (other_pos < lane_pos) begin
                        tr_before_nx[lane_idx*NB + other_idx] =
                            tr_before_nx[lane_idx*NB + other_idx]
                            | train_rot_nx[start_idx];
                        ev_before_nx[lane_idx*NB + other_idx] =
                            ev_before_nx[lane_idx*NB + other_idx]
                            | evt_rot_nx[start_idx];
                        // the masks of the reset rotation (lane 0 first)
                        if (start_idx == 0) begin
                            rst_before[lane_idx*NB + other_idx] = 1'b1;
                        end
                    end
                end // for start_idx
            end // for other_idx
        end // for lane_idx
    end // always before_next_proc

    always @(posedge clk) begin : rotation_proc
        if (rst) begin
            train_rot_oh <= {{(NB-1){1'b0}}, 1'b1};
            evt_rot_oh   <= {{(NB-1){1'b0}}, 1'b1};
            tr_before_q  <= rst_before;
            ev_before_q  <= rst_before;
        end
        else begin
            train_rot_oh <= train_rot_nx;
            evt_rot_oh   <= evt_rot_nx;
            tr_before_q  <= tr_before_nx;
            ev_before_q  <= ev_before_nx;
        end // else (rst)
    end // always rotation_proc

    // ---- event select: FIFO head first (older), fresh record bypasses
    //      an empty FIFO; eviction and training alternate when both wait
    assign tr_avail = tq_vld | tr_fresh;
    assign ev_avail = eq_vld | ev_fresh;
    assign take_evt = take_evt_cp[TE_N-1];  // FIFO controls copy
    assign take_tr  = tr_avail & ~take_evt;
    assign tq_pop   = take_tr & tq_vld;
    assign eq_pop   = take_evt & eq_vld;
    assign tq_push  = tr_fresh & ~(take_tr & ~tq_vld);
    assign eq_push  = ev_fresh & ~(take_evt & ~eq_vld);

    // the FIFO writes the record into its free slot every cycle (early,
    // from the record pins); only the push (late) decides whether it stays
    l1d_pf3_fifo #(
        .WIDTH (TREC_W),
        .DEPTH (TRAIN_FIFO_DEPTH)
    ) u_train_fifo (
        .clk        (clk),
        .rst        (rst),
        .stage_data (tr_word),
        .push_valid (tq_push),
        .pop_en     (tq_pop),
        .head_valid (tq_vld),
        .head_data  (tq_head)
    );

    l1d_pf3_fifo #(
        .WIDTH (EREC_W),
        .DEPTH (EVT_FIFO_DEPTH)
    ) u_evict_fifo (
        .clk        (clk),
        .rst        (rst),
        .stage_data (ev_word),
        .push_valid (eq_push),
        .pop_en     (eq_pop),
        .head_valid (eq_vld),
        .head_data  (eq_head)
    );

    // ---- S1 registers: the training side (FIFO head, else the picked
    //      lane) and the eviction side (the same), then take_evt picks;
    //      the line one-hot is selected like the other fields ----
    assign tq_loh = {{(PATTERN_W-1){1'b0}}, 1'b1}
                    << tq_head[REGION_TAG_W+SET_W+PHT_TAG_W +: LIR_W];
    assign eq_loh = {{(PATTERN_W-1){1'b0}}, 1'b1}
                    << eq_head[REGION_TAG_W +: LIR_W];
    // the two sides are built from flops and the lane picks only; the
    // late take_evt (from the record valids) picks last, through kept
    // 2:1 selects, each group with its own take_evt copy (rule T-9, T-6)
    always @* begin : s1_select_proc
        integer lane_idx; // walks the lanes
        tr_side = {SEL_W{1'b0}};
        ev_side = {SEL_W{1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            tr_side = tr_side | ({SEL_W{tr_pick[lane_idx]}} & {
                train_addr[lane_idx*ADDR_W + OFFSET_W + LIR_W +: REGION_NUM_W],
                train_addr[lane_idx*ADDR_W + OFFSET_W +: LIR_W],
                lane_rtag[lane_idx*REGION_TAG_W +: REGION_TAG_W],
                lane_loh[lane_idx*PATTERN_W +: PATTERN_W]});
            ev_side = ev_side | ({SEL_W{ev_pick[lane_idx]}} & {
                evict_line_addr[lane_idx*LINE_W + LIR_W +: REGION_NUM_W],
                evict_line_addr[lane_idx*LINE_W +: LIR_W],
                evl_rtag[lane_idx*REGION_TAG_W +: REGION_TAG_W],
                evl_loh[lane_idx*PATTERN_W +: PATTERN_W]});
        end // for lane_idx
        // the FIFO head (older) goes first; its valid is a flop
        if (tq_vld) begin
            tr_side = {tq_head[TREC_W-1 -: REGION_NUM_W],
                       tq_head[REGION_TAG_W+SET_W+PHT_TAG_W +: LIR_W],
                       tq_head[SET_W+PHT_TAG_W +: REGION_TAG_W],
                       tq_loh};
        end
        if (eq_vld) begin
            ev_side = {eq_head[EREC_W-1 -: REGION_NUM_W],
                       eq_head[REGION_TAG_W +: LIR_W],
                       eq_head[REGION_TAG_W-1:0],
                       eq_loh};
        end
        // set and PHT tag: only a training event uses them
        nx_set_r  = {SET_W{tq_vld}} & tq_head[PHT_TAG_W +: SET_W];
        nx_ptag_r = {PHT_TAG_W{tq_vld}} & tq_head[PHT_TAG_W-1:0];
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            nx_set_r = nx_set_r | ({SET_W{~tq_vld & tr_pick[lane_idx]}}
                       & lane_set[lane_idx*SET_W +: SET_W]);
            nx_ptag_r = nx_ptag_r | ({PHT_TAG_W{~tq_vld & tr_pick[lane_idx]}}
                       & lane_ptag[lane_idx*PHT_TAG_W +: PHT_TAG_W]);
        end // for lane_idx
    end // always s1_select_proc

    // take_evt copies: each from its own kept OR of the record valids
    genvar te_g; // generate index: take_evt copy
    generate
        for (te_g = 0; te_g < TE_N; te_g = te_g + 1) begin : g_take
            l1d_pf3_or_rows #(.W (1), .N (NB)) u_tr_or (
                .rows (tr_ok), .or_out (tr_fresh_cp[te_g])
            );
            l1d_pf3_or_rows #(.W (1), .N (NB)) u_ev_or (
                .rows (ev_ok), .or_out (ev_fresh_cp[te_g])
            );
            assign take_evt_cp[te_g] = (eq_vld | (ev_fresh_cp[te_g] & pf_enable))
                & (~(tq_vld | (tr_fresh_cp[te_g] & pf_enable)) | prefer_evt_q);
        end // for te_g
    endgenerate

    l1d_pf3_mux2 #(.W (REGION_NUM_W)) u_sel_region (
        .sel (take_evt_cp[0]),
        .in1 (ev_side[SEL_W-1 -: REGION_NUM_W]),
        .in0 (tr_side[SEL_W-1 -: REGION_NUM_W]),
        .out (nx_side[SEL_W-1 -: REGION_NUM_W])
    );
    l1d_pf3_mux2 #(.W (LIR_W + REGION_TAG_W)) u_sel_tag (
        .sel (take_evt_cp[1]),
        .in1 (ev_side[PATTERN_W +: LIR_W + REGION_TAG_W]),
        .in0 (tr_side[PATTERN_W +: LIR_W + REGION_TAG_W]),
        .out (nx_side[PATTERN_W +: LIR_W + REGION_TAG_W])
    );
    l1d_pf3_mux2 #(.W (PATTERN_W)) u_sel_loh (
        .sel (take_evt_cp[2]),
        .in1 (ev_side[PATTERN_W-1:0]),
        .in0 (tr_side[PATTERN_W-1:0]),
        .out (nx_side[PATTERN_W-1:0])
    );

    assign nx_region  = nx_side[SEL_W-1 -: REGION_NUM_W];
    assign nx_line    = nx_side[PATTERN_W + REGION_TAG_W +: LIR_W];
    assign nx_rtag    = nx_side[PATTERN_W +: REGION_TAG_W];
    assign nx_set     = nx_set_r;
    assign nx_ptag    = nx_ptag_r;
    assign nx_line_oh = nx_side[PATTERN_W-1:0];

    always @(posedge clk) begin : s1_reg_proc
        if (rst) begin
            prefer_evt_q <= 1'b0;
            e1_vld       <= 1'b0;
        end
        else begin
            if (tr_avail && ev_avail) begin
                prefer_evt_q <= ~prefer_evt_q;
            end
            e1_vld <= (take_tr | take_evt) & pf_enable;
        end // else (rst)
        // data flops need no reset: e1_vld guards them (S-6); set and PHT
        // tag are only used by a training event
        e1_evt    <= take_evt;
        e1_region <= nx_region;
        e1_line   <= nx_line;
        e1_rtag   <= nx_rtag;
        e1_set    <= nx_set;
        e1_ptag   <= nx_ptag;
    end // always s1_reg_proc

    genvar copy_g; // generate index: register copy
    generate
        for (copy_g = 0; copy_g < KEY_COPIES; copy_g = copy_g + 1) begin : g_key_copy
            l1d_pf3_rep #(.W (KEY_CW)) u_key (
                .clk (clk), .rst (rst), .en (1'b1),
                .d   ({nx_ptag, nx_line, nx_set}),
                .q   (kc_word[copy_g*KEY_CW +: KEY_CW])
            );
        end // for copy_g
        for (copy_g = 0; copy_g < TAG_COPIES; copy_g = copy_g + 1) begin : g_tag_copy
            l1d_pf3_rep #(.W (TAG_CW)) u_tag (
                .clk (clk), .rst (rst), .en (1'b1),
                .d   ({nx_rtag, nx_line}),
                .q   (tc_word[copy_g*TAG_CW +: TAG_CW])
            );
            l1d_pf3_rep #(.W (PATTERN_W)) u_line_oh (
                .clk (clk), .rst (rst), .en (1'b1),
                .d   (nx_line_oh),
                .q   (lc_oh[copy_g*PATTERN_W +: PATTERN_W])
            );
        end // for copy_g
    endgenerate

    // ======================================================================
    // P: the write the event before decided (from flops; applied at the
    // end of this cycle, seen by the event now in S2 through the bypass)
    // ======================================================================
    assign p_ft_set  = ft_vic_q & {FT_ENTRIES{dtrig_q}};
    assign p_ft_clr  = ft_hit_q & {FT_ENTRIES{dprom_q | ddisc_q}};
    assign p_at_set  = at_vic_q & {AT_ENTRIES{dprom_q}};
    assign p_at_clr  = at_hit_q & {AT_ENTRIES{dend_q}};
    assign p_at_tick = at_hit_q & {AT_ENTRIES{dtick_q}};

    // FT read of the promoted region (its FT entry is unchanged until the
    // end of this cycle) and the AT read for the PHT write: the entry that
    // ended, or the AT victim that the promote replaces (its old data)
    always @* begin : p_read_proc
        integer ent_idx; // walks table entries
        pf_line = {LIR_W{1'b0}};
        pf_line_oh = {PATTERN_W{1'b0}};
        pf_pset = {SET_W{1'b0}};
        pf_ptag = {PHT_TAG_W{1'b0}};
        for (ent_idx = 0; ent_idx < FT_ENTRIES; ent_idx = ent_idx + 1) begin
            pf_line = pf_line | ({LIR_W{ft_hit_q[ent_idx]}}
                      & ft_line[ent_idx*LIR_W +: LIR_W]);
            // the decode sits on the table flops, off the ft_hit_q path
            pf_line_oh = pf_line_oh | ({PATTERN_W{ft_hit_q[ent_idx]}}
                      & ({{(PATTERN_W-1){1'b0}}, 1'b1}
                         << ft_line[ent_idx*LIR_W +: LIR_W]));
            pf_pset = pf_pset | ({SET_W{ft_hit_q[ent_idx]}}
                      & ft_pset[ent_idx*SET_W +: SET_W]);
            pf_ptag = pf_ptag | ({PHT_TAG_W{ft_hit_q[ent_idx]}}
                      & ft_ptag[ent_idx*PHT_TAG_W +: PHT_TAG_W]);
        end // for ent_idx
        w_rd_sel  = dend_q ? at_hit_q : at_vic_q;
        w_rd_vld  = 1'b0;
        w_rd_line = {LIR_W{1'b0}};
        w_rd_pset = {SET_W{1'b0}};
        w_rd_ptag = {PHT_TAG_W{1'b0}};
        w_rd_pat  = {PATTERN_W{1'b0}};
        w_end_line = {LIR_W{1'b0}};
        w_vic_line = {LIR_W{1'b0}};
        for (ent_idx = 0; ent_idx < AT_ENTRIES; ent_idx = ent_idx + 1) begin
            w_rd_vld  = w_rd_vld | (w_rd_sel[ent_idx] & at_vld[ent_idx]);
            // the trigger line of both possible entries (flops only), so
            // the masks below do not wait for the dend_q select
            w_end_line = w_end_line | ({LIR_W{at_hit_q[ent_idx]}}
                         & at_line[ent_idx*LIR_W +: LIR_W]);
            w_vic_line = w_vic_line | ({LIR_W{at_vic_q[ent_idx]}}
                         & at_line[ent_idx*LIR_W +: LIR_W]);
            w_rd_pset = w_rd_pset | ({SET_W{w_rd_sel[ent_idx]}}
                        & at_pset[ent_idx*SET_W +: SET_W]);
            w_rd_ptag = w_rd_ptag | ({PHT_TAG_W{w_rd_sel[ent_idx]}}
                        & at_ptag[ent_idx*PHT_TAG_W +: PHT_TAG_W]);
            w_rd_pat  = w_rd_pat | ({PATTERN_W{w_rd_sel[ent_idx]}}
                        & at_pat[ent_idx*PATTERN_W +: PATTERN_W]);
        end // for ent_idx
        w_rd_line = dend_q ? w_end_line : w_vic_line;
        // trigger line one-hot and the lines above it, for W1 (registered
        // here so W1 starts from the masks, rule T-9); made for both
        // entries, dend_q picks last
        for (ent_idx = 0; ent_idx < PATTERN_W; ent_idx = ent_idx + 1) begin
            p_bit_number          = ent_idx[LIR_W-1:0];
            w_rd_line_oh[ent_idx] = dend_q ? (p_bit_number == w_end_line)
                                           : (p_bit_number == w_vic_line);
            w_rd_high[ent_idx]    = dend_q ? (p_bit_number > w_end_line)
                                           : (p_bit_number > w_vic_line);
        end // for ent_idx
    end // always p_read_proc

    // ======================================================================
    // S2: compare FT, AT and PHT. The table is the state before the
    // previous event's write (P); the bypass makes the hits as if that
    // write had been done: an entry being set compares with the previous
    // event's key, an entry being cleared does not hit.
    // ======================================================================
    assign same_rtag   = (p_rtag == e1_rtag);
    assign same_line   = (p_line == e1_line);
    assign same_pfline = (pf_line == e1_line);
    assign ft_vld_e    = (ft_vld & ~p_ft_clr) | p_ft_set;
    assign at_vld_e    = (at_vld & ~p_at_clr) | p_at_set;

    always @* begin : s2_cam_proc
        integer ent_idx; // walks table entries
        integer copy_idx; // the register copy an entry uses
        for (ent_idx = 0; ent_idx < FT_ENTRIES; ent_idx = ent_idx + 1) begin
            copy_idx = ent_idx / FT_PER;
            ft_raw[ent_idx] = ft_vld[ent_idx]
                & (ft_tag[ent_idx*REGION_TAG_W +: REGION_TAG_W]
                   == tc_word[copy_idx*TAG_CW + LIR_W +: REGION_TAG_W]);
            ft_leq_raw[ent_idx] = (ft_line[ent_idx*LIR_W +: LIR_W]
                   == tc_word[copy_idx*TAG_CW +: LIR_W]);
            // a trigger sets its victim: it holds the previous event's key
            ft_hit_t[ent_idx] = ft_vic_q[ent_idx] ? same_rtag
                                                  : ft_raw[ent_idx];
            ft_leq_t[ent_idx] = ft_vic_q[ent_idx] ? same_line
                                                  : ft_leq_raw[ent_idx];
            // a promote / discard clears the previous event's hit
            ft_hit_c[ent_idx] = ft_raw[ent_idx] & ~ft_hit_q[ent_idx];
        end // for ent_idx
        for (ent_idx = 0; ent_idx < AT_ENTRIES; ent_idx = ent_idx + 1) begin
            copy_idx = ent_idx / AT_PER;
            at_raw[ent_idx] = at_vld[ent_idx]
                & (at_tag[ent_idx*REGION_TAG_W +: REGION_TAG_W]
                   == tc_word[copy_idx*TAG_CW + LIR_W +: REGION_TAG_W]);
            at_has_raw[ent_idx] = |(at_pat[ent_idx*PATTERN_W +: PATTERN_W]
                                    & lc_oh[copy_idx*PATTERN_W +: PATTERN_W]);
            // a promote sets its victim: footprint {promoted line,
            // previous line}, key of the previous event
            at_hit_p[ent_idx] = at_vic_q[ent_idx] ? same_rtag
                                                  : at_raw[ent_idx];
            at_has_p[ent_idx] = at_vic_q[ent_idx] ? (same_pfline | same_line)
                                                  : at_has_raw[ent_idx];
            // an end clears the previous event's hit
            at_hit_e[ent_idx] = at_raw[ent_idx] & ~at_hit_q[ent_idx];
            // a tick adds the previous event's line to its hit
            at_has_k[ent_idx] = at_has_raw[ent_idx]
                                | (at_hit_q[ent_idx] & same_line);
        end // for ent_idx
        // PHT: the event's set, same tag, same trigger line (D1)
        for (ent_idx = 0; ent_idx < PHT_SETS; ent_idx = ent_idx + 1) begin
            copy_idx = (ent_idx * PHT_WAYS) / KEY_PER;
            e1_set1h[ent_idx] = (kc_word[copy_idx*KEY_CW +: SET_W]
                                 == ent_idx[SET_W-1:0]);
        end // for ent_idx
        for (ent_idx = 0; ent_idx < PHT_TOTAL; ent_idx = ent_idx + 1) begin
            copy_idx = ent_idx / KEY_PER;
            pht_match[ent_idx] = pht_vld[ent_idx] & pht_nz[ent_idx]
                & (pht_tag[ent_idx*PHT_TAG_W +: PHT_TAG_W]
                   == kc_word[copy_idx*KEY_CW + LIR_W + SET_W +: PHT_TAG_W])
                & (pht_line[ent_idx*LIR_W +: LIR_W]
                   == kc_word[copy_idx*KEY_CW + SET_W +: LIR_W]);
        end // for ent_idx
    end // always s2_cam_proc

    // the decisions of the event before are one-hot (with the _none
    // flops), so each result is an AND-OR of the per-case results: the
    // decision flops come in at the last two levels
    assign ft_hit = (ft_hit_t & {FT_ENTRIES{dtrig_q}})
                  | (ft_hit_c & {FT_ENTRIES{dfclr_q}})
                  | (ft_raw   & {FT_ENTRIES{dft_none_q}});
    assign at_hit = (at_hit_p & {AT_ENTRIES{dprom_q}})
                  | (at_hit_e & {AT_ENTRIES{dend_q}})
                  | (at_raw   & {AT_ENTRIES{dat_keep_q}});
    assign any_ft = (dtrig_q & (|ft_hit_t)) | (dfclr_q & (|ft_hit_c))
                  | (dft_none_q & (|ft_raw));
    assign any_at = (dprom_q & (|at_hit_p)) | (dend_q & (|at_hit_e))
                  | (dat_keep_q & (|at_raw));
    assign ft_same_line = (dtrig_q & (|(ft_hit_t & ft_leq_t)))
                        | (dfclr_q & (|(ft_hit_c & ft_leq_raw)))
                        | (dft_none_q & (|(ft_raw & ft_leq_raw)));
    assign at_rd_has = (dprom_q & (|(at_hit_p & at_has_p)))
                     | (dend_q & (|(at_hit_e & at_has_raw)))
                     | (dtick_q & (|(at_raw & at_has_k)))
                     | (dat_none_q & (|(at_raw & at_has_raw)));

    // PHT per way: hit of the event's set, and the set's data (read by the
    // set alone, in parallel with the compares)
    always @* begin : s2_way_proc
        integer set_idx; // walks the PHT sets
        integer way_idx; // walks the ways
        integer ent_idx; // table entry
        way_hit    = {PHT_WAYS{1'b0}};
        way_first  = {(PHT_WAYS*LIR_W){1'b0}};
        way_second = {(PHT_WAYS*LIR_W){1'b0}};
        way_fbank  = {(PHT_WAYS*NB){1'b0}};
        way_rest   = {(PHT_WAYS*PATTERN_W){1'b0}};
        for (set_idx = 0; set_idx < PHT_SETS; set_idx = set_idx + 1) begin
            for (way_idx = 0; way_idx < PHT_WAYS; way_idx = way_idx + 1) begin
                ent_idx = set_idx * PHT_WAYS + way_idx;
                way_hit[way_idx] = way_hit[way_idx]
                    | (e1_set1h[set_idx] & pht_match[ent_idx]);
                way_first[way_idx*LIR_W +: LIR_W] =
                    way_first[way_idx*LIR_W +: LIR_W]
                    | ({LIR_W{e1_set1h[set_idx]}}
                       & pht_first[ent_idx*LIR_W +: LIR_W]);
                way_second[way_idx*LIR_W +: LIR_W] =
                    way_second[way_idx*LIR_W +: LIR_W]
                    | ({LIR_W{e1_set1h[set_idx]}}
                       & pht_second[ent_idx*LIR_W +: LIR_W]);
                way_fbank[way_idx*NB +: NB] = way_fbank[way_idx*NB +: NB]
                    | ({NB{e1_set1h[set_idx]}} & pht_fbank[ent_idx*NB +: NB]);
                way_rest[way_idx*PATTERN_W +: PATTERN_W] =
                    way_rest[way_idx*PATTERN_W +: PATTERN_W]
                    | ({PATTERN_W{e1_set1h[set_idx]}}
                       & pht_rest[ent_idx*PATTERN_W +: PATTERN_W]);
            end // for way_idx
        end // for set_idx
    end // always s2_way_proc

    // victims (from the valid bits after P): the first free slot (log-depth
    // find-first-set), else the ring pointer's slot. The search runs for
    // each kind of pending write (valid bits from flops) and the decision
    // flops pick the result
    l1d_pf3_ffs #(.W (FT_ENTRIES)) u_ft_free_t (
        .in_vec (~(ft_vld | ft_vic_q)), .low_oh (ft_free_t), .low2_oh (),
        .any_set (ft_seen_t), .two_set ()
    );
    l1d_pf3_ffs #(.W (FT_ENTRIES)) u_ft_free_c (
        .in_vec (~(ft_vld & ~ft_hit_q)), .low_oh (ft_free_c), .low2_oh (),
        .any_set (ft_seen_c), .two_set ()
    );
    l1d_pf3_ffs #(.W (FT_ENTRIES)) u_ft_free_o (
        .in_vec (~ft_vld), .low_oh (ft_free_o), .low2_oh (),
        .any_set (ft_seen_o), .two_set ()
    );
    l1d_pf3_ffs #(.W (AT_ENTRIES)) u_at_free_p (
        .in_vec (~(at_vld | at_vic_q)), .low_oh (at_free_p), .low2_oh (),
        .any_set (at_seen_p), .two_set ()
    );
    l1d_pf3_ffs #(.W (AT_ENTRIES)) u_at_free_e (
        .in_vec (~(at_vld & ~at_hit_q)), .low_oh (at_free_e), .low2_oh (),
        .any_set (at_seen_e), .two_set ()
    );
    l1d_pf3_ffs #(.W (AT_ENTRIES)) u_at_free_o (
        .in_vec (~at_vld), .low_oh (at_free_o), .low2_oh (),
        .any_set (at_seen_o), .two_set ()
    );
    assign ft_free_one  = (ft_free_t & {FT_ENTRIES{dtrig_q}})
                        | (ft_free_c & {FT_ENTRIES{dfclr_q}})
                        | (ft_free_o & {FT_ENTRIES{dft_none_q}});
    assign ft_free_seen = (dtrig_q & ft_seen_t) | (dfclr_q & ft_seen_c)
                        | (dft_none_q & ft_seen_o);
    assign at_free_one  = (at_free_p & {AT_ENTRIES{dprom_q}})
                        | (at_free_e & {AT_ENTRIES{dend_q}})
                        | (at_free_o & {AT_ENTRIES{dat_keep_q}});
    assign at_free_seen = (dprom_q & at_seen_p) | (dend_q & at_seen_e)
                        | (dat_keep_q & at_seen_o);

    always @* begin : s2_victim_proc
        ft_vic = ft_free_one;
        at_vic = at_free_one;
        if (!ft_free_seen) begin
            ft_vic[ft_ring] = 1'b1;
        end
        if (!at_free_seen) begin
            at_vic[at_ring] = 1'b1;
        end
    end // always s2_victim_proc

    assign do_promote    = e1_vld & ~e1_evt & ~any_at & any_ft & ~ft_same_line;
    assign do_trigger    = e1_vld & ~e1_evt & ~any_at & ~any_ft;
    assign do_end_gen    = e1_vld & e1_evt & any_at & at_rd_has;
    assign do_ft_discard = e1_vld & e1_evt & ~any_at & any_ft;

    // the _none / _keep decision flops written straight from the hit
    // vectors (same values as ~(do_*), fewer levels, T-9): ft_same_line
    // already means an FT hit, so for a training event "no FT write" is
    // "AT hit OR same line", one OR over the per-entry hits; e1_evt and
    // e1_vld are flops and pick last
    assign ft_same_v   = (ft_hit_t & ft_leq_t   & {FT_ENTRIES{dtrig_q}})
                       | (ft_hit_c & ft_leq_raw & {FT_ENTRIES{dfclr_q}})
                       | (ft_raw   & ft_leq_raw & {FT_ENTRIES{dft_none_q}});
    assign at_or_same  = |{at_hit, ft_same_v};
    assign end_none    = ~any_at | ~at_rd_has;
    assign dft_none_nx = ~e1_vld | (e1_evt ? (any_at | ~any_ft) : at_or_same);
    assign dat_keep_nx = ~e1_vld | (e1_evt ? end_none : (~any_ft | at_or_same));
    assign dat_none_nx = ~e1_vld
                       | (e1_evt ? end_none : (~any_at & (~any_ft | ft_same_line)));
    assign dtick_nx    = e1_vld & ~e1_evt & any_at;

    // ======================================================================
    // S3: the footprint of a trigger (way select of flops)
    // ======================================================================
    always @* begin : s3_proc
        integer way_idx; // walks the ways
        s3_first  = {LIR_W{1'b0}};
        s3_second = {LIR_W{1'b0}};
        s3_fbank  = {NB{1'b0}};
        s3_rest   = {PATTERN_W{1'b0}};
        for (way_idx = 0; way_idx < PHT_WAYS; way_idx = way_idx + 1) begin
            s3_first  = s3_first | ({LIR_W{way_hit_q[way_idx]}}
                        & way_first_q[way_idx*LIR_W +: LIR_W]);
            s3_second = s3_second | ({LIR_W{way_hit_q[way_idx]}}
                        & way_second_q[way_idx*LIR_W +: LIR_W]);
            s3_fbank  = s3_fbank | ({NB{way_hit_q[way_idx]}}
                        & way_fbank_q[way_idx*NB +: NB]);
            s3_rest   = s3_rest | ({PATTERN_W{way_hit_q[way_idx]}}
                        & way_rest_q[way_idx*PATTERN_W +: PATTERN_W]);
        end // for way_idx
    end // always s3_proc

    // the hit entry has at least one line besides the trigger (pht_nz)
    assign load_valid      = dtrig_ld_q & way_any_q;
    assign load_region     = e2_region;
    assign load_line       = e2_line;
    assign load_first      = s3_first;
    assign load_first_bank = s3_fbank;
    assign load_second     = s3_second;
    assign load_rest       = s3_rest;

    // ======================================================================
    // clocked: S2 decisions and vectors, P table write, PHT write request
    // ======================================================================
    always @(posedge clk) begin : table_proc
        integer ent_idx; // walks table entries
        if (rst) begin
            ft_vld  <= {FT_ENTRIES{1'b0}};
            at_vld  <= {AT_ENTRIES{1'b0}};
            ft_ring <= {FT_IDX_W{1'b0}};
            at_ring <= {AT_IDX_W{1'b0}};
            dtrig_q <= 1'b0;
            dtrig_ld_q <= 1'b0;
            dprom_q <= 1'b0;
            ddisc_q <= 1'b0;
            dend_q  <= 1'b0;
            dtick_q <= 1'b0;
            dfclr_q    <= 1'b0;
            dft_none_q <= 1'b1;
            dat_keep_q <= 1'b1;
            dat_none_q <= 1'b1;
            w_vld   <= 1'b0;
        end
        else begin
            // S2: the ring pointers move on each training event while the
            // table is full (as l1d_pf_sms_core)
            if (e1_vld && !e1_evt && !ft_free_seen) begin
                ft_ring <= ft_ring + 1'b1;
            end
            if (e1_vld && !e1_evt && !at_free_seen) begin
                at_ring <= at_ring + 1'b1;
            end
            dtrig_q <= do_trigger;
            dtrig_ld_q <= do_trigger;
            dprom_q <= do_promote;
            ddisc_q <= do_ft_discard;
            dend_q  <= do_end_gen;
            dtick_q <= dtick_nx;
            dfclr_q    <= do_promote | do_ft_discard;
            dft_none_q <= dft_none_nx;
            dat_keep_q <= dat_keep_nx;
            dat_none_q <= dat_none_nx;
            // P: valid bits
            ft_vld <= ft_vld_e;
            at_vld <= at_vld_e;
            // P: PHT write request (the AT entry that ended, or the
            // pattern the promote replaces)
            w_vld  <= dend_q | (dprom_q & w_rd_vld);
        end // else (run)
        // S2: vectors and key of this event (decisions guard them)
        ft_hit_q  <= ft_hit;
        ft_vic_q  <= ft_vic;
        at_hit_q  <= at_hit;
        at_vic_q  <= at_vic;
        p_rtag    <= e1_rtag;
        p_line    <= e1_line;
        p_set     <= e1_set;
        p_ptag    <= e1_ptag;
        p_line_oh <= lc_oh[PATTERN_W-1:0];
        // S2: PHT lookup flops
        way_hit_q    <= way_hit;
        way_any_q    <= |way_hit;
        way_first_q  <= way_first;
        way_second_q <= way_second;
        way_fbank_q  <= way_fbank;
        way_rest_q   <= way_rest;
        e2_region    <= e1_region;
        e2_line      <= e1_line;
        // P: data fields (valid bits guard them, S-6)
        for (ent_idx = 0; ent_idx < FT_ENTRIES; ent_idx = ent_idx + 1) begin
            if (p_ft_set[ent_idx]) begin
                ft_tag[ent_idx*REGION_TAG_W +: REGION_TAG_W] <= p_rtag;
                ft_line[ent_idx*LIR_W +: LIR_W]              <= p_line;
                ft_pset[ent_idx*SET_W +: SET_W]              <= p_set;
                ft_ptag[ent_idx*PHT_TAG_W +: PHT_TAG_W]      <= p_ptag;
            end
        end // for ent_idx
        for (ent_idx = 0; ent_idx < AT_ENTRIES; ent_idx = ent_idx + 1) begin
            if (p_at_set[ent_idx]) begin
                at_tag[ent_idx*REGION_TAG_W +: REGION_TAG_W] <= p_rtag;
                at_line[ent_idx*LIR_W +: LIR_W]              <= pf_line;
                at_pset[ent_idx*SET_W +: SET_W]              <= pf_pset;
                at_ptag[ent_idx*PHT_TAG_W +: PHT_TAG_W]      <= pf_ptag;
                at_pat[ent_idx*PATTERN_W +: PATTERN_W]       <= pf_line_oh | p_line_oh;
            end
            else if (p_at_tick[ent_idx]) begin
                at_pat[ent_idx*PATTERN_W +: PATTERN_W] <=
                    at_pat[ent_idx*PATTERN_W +: PATTERN_W] | p_line_oh;
            end
        end // for ent_idx
        w_set  <= w_rd_pset;
        w_ptag <= w_rd_ptag;
        w_line    <= w_rd_line;
        w_line_oh <= w_rd_line_oh;
        w_high    <= w_rd_high;
        w_pat     <= w_rd_pat;
    end // always table_proc

    // ======================================================================
    // W1 (from the w_* flops): same-tag match of every PHT entry, the first
    // line (nearest after the trigger, else wrapping round) and the rest.
    // The write in W2 this cycle (x_*) lands before this one: its way is
    // compared with x_ptag instead of the table (forwarding).
    // ======================================================================
    always @* begin : w1_match_proc
        integer ent_idx; // walks the PHT entries
        for (ent_idx = 0; ent_idx < PHT_SETS; ent_idx = ent_idx + 1) begin
            w_set1h[ent_idx] = (w_set == ent_idx[SET_W-1:0]);
        end // for ent_idx
        for (ent_idx = 0; ent_idx < PHT_TOTAL; ent_idx = ent_idx + 1) begin
            if (x_vld && x_write[ent_idx]) begin
                w_tmatch[ent_idx] = (x_ptag == w_ptag);
            end
            else begin
                w_tmatch[ent_idx] = pht_vld[ent_idx]
                    & (pht_tag[ent_idx*PHT_TAG_W +: PHT_TAG_W] == w_ptag);
            end // else (from the table)
        end // for ent_idx
    end // always w1_match_proc

    always @* begin : w1_mask_proc
        w_all   = w_pat & ~w_line_oh;
        w_above = w_all & w_high;
    end // always w1_mask_proc

    // first and second line (order: above the trigger, then from line 0):
    // lowest and second lowest of the lines above and of those below, in
    // parallel, then a select
    assign w_below = w_all & ~w_high;
    l1d_pf3_ffs #(.W (PATTERN_W)) u_w_above (
        .in_vec (w_above), .low_oh (w_ffs_above), .low2_oh (w_ffs2_above),
        .any_set (w_seen_a), .two_set (w_two_a)
    );
    l1d_pf3_ffs #(.W (PATTERN_W)) u_w_below (
        .in_vec (w_below), .low_oh (w_ffs_below), .low2_oh (w_ffs2_below),
        .any_set (w_seen_b), .two_set ()
    );

    always @* begin : w1_first_proc
        w_first_oh  = w_seen_a ? w_ffs_above : w_ffs_below;
        w_second_oh = w_two_a  ? w_ffs2_above
                    : (w_seen_a ? w_ffs_below : w_ffs2_below);
    end // always w1_first_proc

    always @(posedge clk) begin : w1_reg_proc
        if (rst) begin
            x_vld    <= 1'b0;
            x_set_go <= {PHT_SETS{1'b0}};
        end
        else begin
            x_vld    <= w_vld;
            x_set_go <= w_set1h & {PHT_SETS{w_vld}};
        end // else (rst)
        x_ptag   <= w_ptag;
        x_line   <= w_line;
        x_rest   <= w_all & ~w_first_oh;
        x_first_oh  <= w_first_oh;
        x_second_oh <= w_second_oh;
        x_nz     <= w_seen_a | w_seen_b;
        x_tmatch <= w_tmatch;
    end // always w1_reg_proc

    // ---- W2: one-hot -> number (bit b of the number is the OR of the
    //      lines whose number has bit b set), the first line's bank;
    //      parallel with the way pick, only the write data ----
    always @* begin : w2_encode_proc
        integer bit_idx; // walks the pattern bits
        x_first  = {LIR_W{1'b0}};
        x_second = {LIR_W{1'b0}};
        for (bit_idx = 0; bit_idx < PATTERN_W; bit_idx = bit_idx + 1) begin
            w1_bit_number = bit_idx[LIR_W-1:0];
            x_first  = x_first | ({LIR_W{x_first_oh[bit_idx]}} & w1_bit_number);
            w2_bit_number = bit_idx[LIR_W-1:0];
            x_second = x_second | ({LIR_W{x_second_oh[bit_idx]}} & w2_bit_number);
        end // for bit_idx
        x_fbank = {NB{1'b0}};
        for (bit_idx = 0; bit_idx < PATTERN_W; bit_idx = bit_idx + 1) begin
            w1_bank_idx = bit_idx[BANK_W-1:0];
            x_fbank[w1_bank_idx] = x_fbank[w1_bank_idx] | x_first_oh[bit_idx];
        end // for bit_idx
    end // always w2_encode_proc

    // ======================================================================
    // W2: per set, the way to write (same tag, else a free way, else the
    // ring way); only the W2 set is written
    // ======================================================================
    // The way that is not a tag match (first free way, else the ring way)
    // depends on flops only and is ready early; the same-tag ways (at most
    // one per set: a written tag always goes to its own way, A5, checked
    // under L1D_DEBUG) pick last: match OR 1 + 2:1 1 + set AND 1, instead
    // of a chain of "seen" gates over 2 x PHT_WAYS ways (rule T-9)
    genvar set_g;  // generate index: PHT set
    genvar wcp_g;  // generate index: way pick copy
    generate
        for (set_g = 0; set_g < PHT_SETS; set_g = set_g + 1) begin : g_way_set
            for (wcp_g = 0; wcp_g < WP_COPIES; wcp_g = wcp_g + 1) begin : g_way_copy
                if (wcp_g == 0) begin : g_main
                    l1d_pf3_sms_way_pick #(.WAYS (PHT_WAYS)) u_pick (
                        .set_go    (x_set_go[set_g]),
                        .tag_match (x_tmatch[set_g*PHT_WAYS +: PHT_WAYS]),
                        .way_valid (pht_vld[set_g*PHT_WAYS +: PHT_WAYS]),
                        .ring_way  (pht_ptr[set_g*WAY_W +: WAY_W]),
                        .way_write (x_write_cp[set_g*PHT_WAYS +: PHT_WAYS]),
                        .ring_adv  (x_adv[set_g])
                    );
                end
                else begin : g_extra
                    l1d_pf3_sms_way_pick #(.WAYS (PHT_WAYS)) u_pick (
                        .set_go    (x_set_go[set_g]),
                        .tag_match (x_tmatch[set_g*PHT_WAYS +: PHT_WAYS]),
                        .way_valid (pht_vld[set_g*PHT_WAYS +: PHT_WAYS]),
                        .ring_way  (pht_ptr[set_g*WAY_W +: WAY_W]),
                        .way_write (x_write_cp[wcp_g*PHT_TOTAL
                                               + set_g*PHT_WAYS +: PHT_WAYS]),
                        .ring_adv  ()
                    );
                end // else (extra copy)
            end // for wcp_g
        end // for set_g
    endgenerate
    assign x_write = x_write_cp[PHT_TOTAL-1:0];


    always @(posedge clk) begin : pht_proc
        integer ent_idx; // walks the PHT entries
        integer set_idx; // walks the PHT sets
        if (rst) begin
            pht_vld <= {PHT_TOTAL{1'b0}};
            pht_ptr <= {(PHT_SETS*WAY_W){1'b0}};
        end
        else begin
            for (ent_idx = 0; ent_idx < PHT_TOTAL; ent_idx = ent_idx + 1) begin
                if (x_write[ent_idx]) begin
                    pht_vld[ent_idx] <= 1'b1;
                end
            end // for ent_idx
            for (set_idx = 0; set_idx < PHT_SETS; set_idx = set_idx + 1) begin
                if (x_adv[set_idx]) begin
                    pht_ptr[set_idx*WAY_W +: WAY_W] <=
                        pht_ptr[set_idx*WAY_W +: WAY_W] + 1'b1;
                end
            end // for set_idx
        end // else (run)
        for (ent_idx = 0; ent_idx < PHT_TOTAL; ent_idx = ent_idx + 1) begin
            if (x_write[ent_idx]) begin
                pht_tag[ent_idx*PHT_TAG_W +: PHT_TAG_W]  <= x_ptag;
                pht_line[ent_idx*LIR_W +: LIR_W]         <= x_line;
                pht_first[ent_idx*LIR_W +: LIR_W]        <= x_first;
                pht_second[ent_idx*LIR_W +: LIR_W]       <= x_second;
                pht_fbank[ent_idx*NB +: NB]              <= x_fbank;
                pht_nz[ent_idx]                          <= x_nz;
            end
        end // for ent_idx
        // the pattern, in two halves, each from its own way pick copy
        for (ent_idx = 0; ent_idx < PHT_TOTAL; ent_idx = ent_idx + 1) begin
            if (x_write_cp[PHT_TOTAL + ent_idx]) begin
                pht_rest[ent_idx*PATTERN_W +: PAT_HALF] <= x_rest[PAT_HALF-1:0];
            end
            if (x_write_cp[2*PHT_TOTAL + ent_idx]) begin
                pht_rest[ent_idx*PATTERN_W + PAT_HALF +: PATTERN_W - PAT_HALF]
                    <= x_rest[PATTERN_W-1:PAT_HALF];
            end
        end // for ent_idx
    end // always pht_proc

`ifdef L1D_DEBUG
    // ---- watch-dogs (V-9.3), simulation only ----
    always @(posedge clk) begin : debug_proc
        integer set_idx; // walks the PHT sets (A5 check)
        if (!rst && x_vld) begin
            for (set_idx = 0; set_idx < PHT_SETS; set_idx = set_idx + 1) begin
                if (x_set_go[set_idx]
                    && ((x_tmatch[set_idx*PHT_WAYS +: PHT_WAYS]
                         & (x_tmatch[set_idx*PHT_WAYS +: PHT_WAYS] - 1'b1))
                        != {PHT_WAYS{1'b0}})) begin
                    $display("[%0t] L1D PF3_SMS ERROR two PHT ways hold one tag (A5)", $time);
                end
            end // for set_idx
        end
        if (!rst && e1_vld) begin
            if ((ft_hit & (ft_hit - 1'b1)) != {FT_ENTRIES{1'b0}}) begin
                $display("[%0t] L1D PF3_SMS ERROR two FT entries", $time);
            end
            if ((at_hit & (at_hit - 1'b1)) != {AT_ENTRIES{1'b0}}) begin
                $display("[%0t] L1D PF3_SMS ERROR two AT entries", $time);
            end
            if (any_ft && any_at) begin
                $display("[%0t] L1D PF3_SMS ERROR region in FT and AT", $time);
            end
            if ((way_hit & (way_hit - 1'b1)) != {PHT_WAYS{1'b0}}) begin
                $display("[%0t] L1D PF3_SMS ERROR two PHT ways", $time);
            end
        end // if (event)
    end // always debug_proc
`endif

endmodule

`default_nettype wire
`endif // L1D_PF3_SMS_CORE_V
