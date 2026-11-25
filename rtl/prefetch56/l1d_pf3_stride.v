// ==========================================================================
// l1d_pf3_stride.v - stride prefetcher, 3-cycle version (training record
//                    at the unit pins -> request in the unit's output
//                    register three clock edges later)
// ==========================================================================
// IN SIMPLE WORDS : a PC-indexed Reference Prediction Table (RPT, 16 sets
//   x 4 ways) with the Chen and Baer trust states, byte strides and the
//   4 KB page limit, exactly as the 9-stage l1d_pf_stride. Every table
//   entry (l1d_pf3_stride_entry) checks the event and updates itself in the
//   same cycle, keeps its next prefetch target and its expected next offset
//   ready-made, and a missing event is written into its set's victim entry
//   in the same cycle. The output then only needs: find the entry (S2) and
//   pick the stored target (one-hot by the hit, early) and the fire flags
//   per set (registered); S3 only ORs 16 flags.
//
// STAGES (one clock each; the S1 and S2 outputs are flops):
//   S1  lowest-bank record pick, PC tag fold, page hash fold, set decode,
//       3 x lo                                 -> E1_COPIES copies of e1
//   S2  64 entries: tag check, stride check, fire flags, update or
//       allocation (inside l1d_pf3_stride_entry); a hit on an entry the
//       event before allocated uses that entry's new contents worked out
//       once per event copy (l1d_pf3_stride_anew, u_anew); victim per set
//       from the entries' classes; per set, the hit entry's target (4:1)
//                                  -> fire_new_q / fire_dup_q / grp_lip_q
//                                     per set, e2_*
//   S3  cand_valid = OR of the set flags (the filter choice by prev_ins_q),
//       built CV_N = 15 times (rank control, rank data x 5, filter registers,
//       one per filter-key copy) so no copy drives a long buffer tree,
//       target = OR of the 4 group flops grp_lip_q (l1d_pf3_or_rows, also one copy
//       per event copy for the next event's duplicate check),
//       filter update; the candidate is ranked by l1d_pf3_rank in the same
//       cycle and lands in the unit's output register at the end of S3.
//
// INSTANTIATED BY : l1d_prefetch_unit, instance u_stride, 1 copy.
//
// PARAMETERS:
//   name        default unit    meaning
//   NB          4       banks   cache banks = training lanes
//   PC_W        64      bits    instruction PC width
//   ADDR_W      64      bits    cache byte address width
//   PA_W        56      bits    physical address bits the table uses
//   LINE_BYTES  64      bytes   cache line
//   PAGE_BYTES  4096    bytes   prefetches never leave this page
//   RPT_SETS    16      sets    table sets (PC bits above PC_LSB)
//   RPT_WAYS    4       ways    table ways (>= 2)
//   TAG_W       12      bits    folded PC tag
//   PC_LSB      1       bit     lowest PC bit used for the set
//   DIST        2       strides prefetch distance (fixed 2: tgt = lo + 3s)
//   FILT_DEPTH  4       entries duplicate filter (>= 2)
//   PH_W        10      bits    page hash (same page check, filter key)
//   E1_COPIES   16      copies  copies of the S1 event register (fanout
//                               split, T-6); divides RPT_SETS
//   LC_COPIES   8       copies  copies of the filter / E-1 target OR
//                               (fanout split); divides RPT_SETS
//   PICK_COPIES 8       copies  copies of the S1 lane pick OR, each
//                               feeding E1_COPIES / PICK_COPIES event
//                               registers (fanout split); divides E1_COPIES
//   FOLD_COPIES 4       copies  copies of the S1 PC tag fold the cards
//                               compare with (fanout split); divides
//                               RPT_SETS
//
// PORTS (default widths; b = bank lane 0..3, bus slice [b*W +: W]):
//   clk, rst            in  1      clock, synchronous reset (high)
//   pf_enable           in  1      1 = training and requests on
//   clear               in  1      invalidate the table (satp write)
//   train_valid         in  4      training record per bank lane
//   train_no_train      in  4      the record must not train
//   train_is_store      in  4      store record (unique prefetch)
//   train_pc            in  4x64   instruction PC
//   train_addr          in  4x64   byte address (bits [PA_W-1:0] used)
//   cand_valid          out 1      S3: the candidate passed the filter
//                                  (copy for the rank's control logic)
//   cand_valid_dp       out 5      S3: the same, NB+1 more copies for the
//                                  rank's data: bit b = bank b's line
//                                  select, bit NB = the hold loads
//   cand_line           out 58     S3: its cache line address (flops)
//   cand_bank           out 4      S3: its bank, one-hot
//   cand_unique         out 1      S3: store stream, ask unique (flop)
//
// WORKED EXAMPLE (loop load at PC X, addresses 0x1000, 0x1080, 0x1100):
//   0x1000: S2 finds no entry -> the set's victim is written (stride 0,
//   Initial). 0x1080: stride 0x80 learned (Transient), tgt = 0x80 + 3 x
//   0x80 = 0x200, exp = 0x100. 0x1100 at the pins in cycle t: S2 (t+1)
//   finds 0x100 == exp -> fire; S3 (t+2) sends line 0x1200 (tgt 0x200 of
//   the same page), pf_req_valid of bank 0 is 1 in cycle t+3.
//
// LOGIC LEVELS (T-9, estimates until place and route measures them):
//   S1 pick 2 + tag fold 3 + mux 2 = 7, 3 x lo adder 6 + mux 2 = 8, then
//   the copies (fanout E1_COPIES);
//   S2 inside the entry about 8 (see l1d_pf3_stride_entry); victim from
//   flops 5, set miss 2 after the hits -> allocation write select;
//   candidate target: hit 4 + 64-input AND-OR 4 = 8; set flags fire 6 +
//   OR4 1 = 7;
//   S3 OR of 16 set flags 3 + filter choice 1 = 4 before the rank logic.
//
// ASSUMPTIONS (V-9.2): A1 one record per cycle is taken (the lowest bank
//   lane with a trainable record), as in the 9-stage version. A2 sizes
//   are powers of two, E1_COPIES and LC_COPIES (filter / target copies) divide
//   RPT_SETS. A3 PA_W <= ADDR_W,
//   PA_W > page bits. A4 DIST = 2 (the entry's stored 3 x stride).
// DIFFERENCES FROM THE 9-STAGE RTL (no function removed):
//   D1 replacement is exactly one-at-a-time (the 9-stage RTL masked the
//      ways written in the 3 cycles before, its D-8).
//   D2 the duplicate filter checks the same FILT_DEPTH last sent keys
//      ({page hash, line in page}); the newest one is checked in S3.
// ==========================================================================
`ifndef L1D_PF3_STRIDE_V
`define L1D_PF3_STRIDE_V
`default_nettype none

module l1d_pf3_stride #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter NB         = 4,    // default 4 banks / training lanes
    parameter PC_W       = 64,   // default 64 bits: PC
    parameter ADDR_W     = 64,   // default 64 bits: byte address
    parameter PA_W       = 56,   // default 56 bits: physical address
    parameter LINE_BYTES = 64,   // default 64 bytes: line
    parameter PAGE_BYTES = 4096, // default 4096 bytes: page
    parameter RPT_SETS   = 16,   // default 16 sets
    parameter RPT_WAYS   = 4,    // default 4 ways
    parameter TAG_W      = 12,   // default 12 bits: PC tag
    parameter PC_LSB     = 1,    // default 1: lowest PC bit of the set
    parameter DIST       = 2,    // default 2 strides ahead (fixed, A4)
    parameter FILT_DEPTH = 4,    // default 4 entries: filter
    parameter PH_W       = 10,   // default 10 bits: page hash
    parameter E1_COPIES  = 16,   // default 16 copies of the event register
    parameter LC_COPIES  = 8,    // default 8 copies of the filter / E-1 target
    parameter PICK_COPIES = 8,   // default 8 copies of the S1 lane pick OR
    parameter FOLD_COPIES = 4    // default 4 copies of the S1 PC tag fold
) (
    // ---- clock, reset, control ----
    input  wire                      clk,             // clock
    input  wire                      rst,             // sync reset, high
    input  wire                      pf_enable,       // 1 = on
    input  wire                      clear,           // invalidate table
    // ---- training records (one lane per bank, unit pins) ----
    input  wire [NB-1:0]             train_valid,     // 4
    input  wire [NB-1:0]             train_no_train,  // 4
    input  wire [NB-1:0]             train_is_store,  // 4
    input  wire [NB*PC_W-1:0]        train_pc,        // 4 x 64
    input  wire [NB*ADDR_W-1:0]      train_addr,      // 4 x 64
    // ---- candidate (S3) ----
    output wire                      cand_valid,      // passed the filter
    output wire [NB:0]               cand_valid_dp,   // NB+1 = 5: same, data
                                                      //   copies (one per
                                                      //   bank + one)
    output wire [ADDR_W-$clog2(LINE_BYTES)-1:0] cand_line, // 58 bits
    output reg  [NB-1:0]             cand_bank,       // 4, one-hot
    output wire                      cand_unique      // store stream
);

    // ---- derived sizes ----
    localparam NS       = RPT_SETS;                  // 16 sets
    localparam NW       = RPT_WAYS;                  // 4 ways
    localparam ENTRIES  = NS * NW;                   // 64 entries
    localparam SET_W    = $clog2(NS);                // 4 bits
    localparam OFFSET_W = $clog2(LINE_BYTES);        // 6 bits
    localparam PAGE_W   = $clog2(PAGE_BYTES);        // 12 bits
    localparam LIP_W    = PAGE_W - OFFSET_W;         // 6 bits: line in page
    localparam PPN_W    = PA_W - PAGE_W;             // 44 bits: page number
    localparam PAD_W    = ADDR_W - PA_W;             // 8 bits of zero
    localparam LO3_W    = PAGE_W + 2;                // 14 bits: 3 x lo
    localparam FOLD_LSB = PC_LSB + SET_W;            // 5: first tag bit
    localparam FOLD_W   = PC_W - FOLD_LSB;           // 59 PC bits folded
    localparam BANK_W   = $clog2(NB);                // 2 bits
    localparam SPC      = NS / E1_COPIES;            // 1 set per copy
    localparam LC_N     = LC_COPIES;                 // 8 copies of the
                                                     //   filter / E-1 target
    localparam CPP      = E1_COPIES / PICK_COPIES;   // 2 event registers
    localparam CV_N     = LC_N + NB + 3;             // 15 copies of the
                                                     //   candidate valid
                                                     //   per lane pick copy
    localparam SPL      = NS / LC_N;                 // 2 sets per such copy
    localparam SPG      = 4;                         // 4 sets per target
                                                     //   group register
    localparam NG       = NS / SPG;                  // 4 target groups
    localparam SPF      = NS / FOLD_COPIES;          // 4 sets per tag fold
                                                     //   copy
    // one copy of the event: {set valid bits, tag, st, lo, lo3, ph}
    localparam EV_W     = TAG_W + 1 + PAGE_W + LO3_W + PH_W; // 49 bits
    localparam FL_W     = FILT_DEPTH * LIP_W;        // 24 bits: filter lips

    // ---- S1: per lane values and the picked record ----
    reg  [NB-1:0]        lane_pick;      // one-hot: lowest trainable lane
    reg                  lane_seen;      // a lower lane was trainable
    reg  [NB*TAG_W-1:0]  lane_tag;       // 4 x 12: folded PC tags
    reg  [NB*PH_W-1:0]   lane_ph;        // 4 x 10: page hashes
    wire [NB*LO3_W-1:0]  lane_lo3;       // 4 x 14: 3 x page offset
    reg                  pick_any;       // a record is picked
    reg  [SET_W-1:0]     pick_set;       // 4 bits: set index
    reg                  pick_st;        // store record
    reg  [PH_W-1:0]      pick_ph;        // 10 bits: page hash
    reg  [PPN_W-1:0]     pick_ppn;       // 44 bits: page number
    reg  [NS-1:0]        pick_set_on;    // 16: set one-hot, valid gated
    wire                 pick_take;      // the record enters S2
    reg  [NB*EV_W-1:0]   lane_event;     // 4 x 49: each lane's event word,
                                         //   masked by its lane_pick bit
    wire [PICK_COPIES*EV_W-1:0] pk_event; // 8 x 49: copies of the picked
                                         //   event word (OR of lane_event)
    wire [FOLD_COPIES*NB*TAG_W-1:0] cp_s1_tag; // 4 x 4 x 12: copies of the
                                         //   lane PC tags for the cards' S1
                                         //   key compares (SPF sets each)
    wire [FOLD_COPIES*NB*PH_W-1:0] cp_s1_ph; // 4 x 4 x 10: copies of the
                                         //   lane page hashes for the cards'
                                         //   S1 same-page compares
    reg  [NB-1:0]        lane_ph_ev;     // S1: lane l has the page hash of
                                         //   the event now in S2 (e1_ph)
    reg  [NB-1:0]        lane_ph_p;      // S1: lane l has the page hash of
                                         //   the event before it (e2_ph)

    // ---- S1 -> S2 registers ----
    reg                  e1_vld;         // event in S2 (central)
    reg  [PH_W-1:0]      e1_ph;          // 10 bits (central copy)
    reg                  e1_st;          // store event (central copy)
    reg  [PPN_W-1:0]     e1_ppn;         // 44 bits: page number
    wire [E1_COPIES*SPC-1:0] cp_set_on;  // 16 x 1: set on bits, per copy
    wire [E1_COPIES*EV_W-1:0] cp_event;  // 16 x 49: event word, per copy
    wire [E1_COPIES*EV_W-1:0] cp_pevent; // 16 x 49: the event before (alloc)
    wire [E1_COPIES*NB-1:0]  cp_lane;    // 16 x 4: the S2 event's lane
    wire [E1_COPIES*NB-1:0]  cp_p2_key;  // 16 x 4: lane l had E-2's key
    wire [E1_COPIES*NB-1:0]  cp_ph_ev;   // 16 x 4: lane l had E-1's page
                                         //   hash (the S2 event of then)
    wire [E1_COPIES*NB-1:0]  cp_ph_p;    // 16 x 4: lane l had E-2's page
                                         //   hash
    reg  [NB-1:0]            lane_p2_key; // S1: lane l has the key of the
                                          //   event now in P (E-2 of the
                                          //   S1 event); the S2 card picks
                                          //   the event's lane
    // new contents of a card E-1 allocated if E hits it (l1d_pf3_stride_anew)
    wire [E1_COPIES-1:0]            cp_a_key;     // 16: E tag == E-1 tag
    wire [E1_COPIES-1:0]            cp_a_correct; // 16: correct such hit
    wire [E1_COPIES*(PAGE_W+1)-1:0] cp_a_stride;  // 16 x 13
    wire [E1_COPIES*(PAGE_W+1)-1:0] cp_a_exp;     // 16 x 13
    wire [E1_COPIES*(PAGE_W+3)-1:0] cp_a_s3;      // 16 x 15
    wire [E1_COPIES*(PAGE_W+3)-1:0] cp_a_tgt;     // 16 x 15
    wire [E1_COPIES-1:0]            cp_a_nz;      // 16: stride != 0

    // ---- S2 -> S3 registers ----
    reg                  e2_vld;         // event in S3
    reg                  e2_st;          // store event: unique
    reg  [PH_W-1:0]      e2_ph;          // 10 bits: filter key
    reg  [PPN_W-1:0]     e2_ppn;         // 44 bits: candidate page
    reg  [NS-1:0]        fire_new_q;     // 16: a way of the set fired,
                                         //     target not filter slot 3
    reg  [NS-1:0]        fire_dup_q;     // 16: fired, not E-1's key
    wire [CV_N-1:0]      cand_cp;        // 15: copies of the candidate valid
    wire [LIP_W-1:0]     cand_lip_q;     // 6 bits: hit entry's target (S3,
                                         // OR of the per-set flops)
    // the hit's target and bank are registered per group of SPG sets
    // (the OR of the group's sets is made in S2, before the flop), so S3
    // ORs NG = 4 group flops instead of 16 set flops (2 levels less on
    // every S3 path through the stride target, rule T-9)
    wire [NS*LIP_W-1:0]  set_lip_nx;     // 16 x 6: per set, its hit's
                                         //   target (S2, 0 when no hit)
    wire [NS*NB-1:0]     set_bank_nx;    // 16 x 4: per set, its hit's
                                         //   bank, one-hot (S2)
    reg  [NG*LIP_W-1:0]  grp_lip_q;      // 4 x 6: per group, the OR of its
                                         //   sets' targets
    // the same OR in two more kept copies (rule T-6): the line to the rank
    // and the bank decode, apart from the filter / key copies
    wire [LIP_W-1:0]     out_lip;        // 6 bits: copy for cand_line
    reg  [NG*NB-1:0]     grp_bank_oh_q;  // 4 x 4: per group, the hit
                                         //   way's target bank, one-hot
                                         //   (0 when no way hits)
    wire [NB-1:0]        bank_oh;        // 4: OR of the sets = cand_bank
    wire [LC_N*LIP_W-1:0]      cp_prev_lip; // 8 x 6: copies of cand_lip_q

    // ---- entry wires (S2) ----
    wire [ENTRIES-1:0]       ent_hit;    // 64: event is the entry's
    wire [ENTRIES*3-1:0]     ent_class;  // 64 x 3: replacement class
    wire [ENTRIES-1:0]       ent_fire_new; // 64
    wire [ENTRIES-1:0]       ent_fire_dup; // 64
    wire [ENTRIES*LIP_W-1:0] ent_tgt_lip;  // 64 x 6
    wire [ENTRIES-1:0]       ent_alloc;  // 64: allocation write

    // ---- filter (S3 writes, S2 reads) ----
    reg  [FILT_DEPTH-1:0]       filt_vld;  // 4: slot holds a key
    reg  [FILT_DEPTH*PH_W-1:0]  filt_ph;   // 4 x 10: page hash
    wire [FL_W-1:0]             filt_lip_next; // 24: shifted lips
    wire [LC_N*FL_W-1:0]        cp_filt_lip;   // 8 x 24: copies
    reg                         prev_ins_q;    // E-1 sent a candidate

    // ======================================================================
    // S1: fold the PC tag and the page hash of every lane
    // ======================================================================
    always @* begin : lane_fold_proc
        integer lane_idx; // training lane
        integer bit_idx;  // bit being folded
        lane_tag = {(NB*TAG_W){1'b0}};
        lane_ph  = {(NB*PH_W){1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            // PC bits above the set bits, XOR-folded into TAG_W bits
            for (bit_idx = 0; bit_idx < FOLD_W; bit_idx = bit_idx + 1) begin
                lane_tag[lane_idx*TAG_W + (bit_idx % TAG_W)] =
                    lane_tag[lane_idx*TAG_W + (bit_idx % TAG_W)]
                    ^ train_pc[lane_idx*PC_W + FOLD_LSB + bit_idx];
            end // for bit_idx
            // page number XOR-folded into PH_W bits
            for (bit_idx = 0; bit_idx < PPN_W; bit_idx = bit_idx + 1) begin
                lane_ph[lane_idx*PH_W + (bit_idx % PH_W)] =
                    lane_ph[lane_idx*PH_W + (bit_idx % PH_W)]
                    ^ train_addr[lane_idx*ADDR_W + PAGE_W + bit_idx];
            end // for bit_idx
        end // for lane_idx
    end // always lane_fold_proc

    // 3 x lo per lane (lo + 2 lo), before the pick so the adder runs in
    // parallel with the pick logic
    genvar lane_g; // training lane
    generate
        for (lane_g = 0; lane_g < NB; lane_g = lane_g + 1) begin : g_lo3
            wire [LO3_W-1:0] lo_x1; // 14 bits: lo
            wire [LO3_W-1:0] lo_x2; // 14 bits: 2 x lo
            assign lo_x1 = {2'b00, train_addr[lane_g*ADDR_W +: PAGE_W]};
            assign lo_x2 = {1'b0, train_addr[lane_g*ADDR_W +: PAGE_W], 1'b0};
            l1d_pf3_ks_add #(.W(LO3_W), .SUB(0)) u_lo3 (
                .a   (lo_x1),
                .b   (lo_x2),
                .sum (lane_lo3[lane_g*LO3_W +: LO3_W])
            );
        end // for lane_g
    endgenerate

    // ---- S1: lowest trainable lane, then one-hot AND-OR of its values ----
    always @* begin : lane_pick_proc
        integer lane_idx; // training lane
        integer set_idx;  // table set
        lane_seen = 1'b0;
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_pick[lane_idx] = train_valid[lane_idx]
                                  & ~train_no_train[lane_idx] & ~lane_seen;
            lane_seen = lane_seen
                        | (train_valid[lane_idx] & ~train_no_train[lane_idx]);
        end // for lane_idx
        pick_any = 1'b0;
        pick_set = {SET_W{1'b0}};
        pick_st  = 1'b0;
        pick_ph  = {PH_W{1'b0}};
        pick_ppn = {PPN_W{1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            pick_any = pick_any | lane_pick[lane_idx];
            pick_set = pick_set | ({SET_W{lane_pick[lane_idx]}}
                       & train_pc[lane_idx*PC_W + PC_LSB +: SET_W]);
            pick_st  = pick_st | (lane_pick[lane_idx] & train_is_store[lane_idx]);
            pick_ph  = pick_ph | ({PH_W{lane_pick[lane_idx]}}
                       & lane_ph[lane_idx*PH_W +: PH_W]);
            pick_ppn = pick_ppn | ({PPN_W{lane_pick[lane_idx]}}
                       & train_addr[lane_idx*ADDR_W + PAGE_W +: PPN_W]);
        end // for lane_idx
        // each lane's event word, masked by its pick bit; the OR of the
        // rows is done by PICK_COPIES kept copies of l1d_pf3_or_rows
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_event[lane_idx*EV_W +: EV_W] = {EV_W{lane_pick[lane_idx]}} & {
                lane_tag[lane_idx*TAG_W +: TAG_W],
                train_is_store[lane_idx],
                train_addr[lane_idx*ADDR_W +: PAGE_W],
                lane_lo3[lane_idx*LO3_W +: LO3_W],
                lane_ph[lane_idx*PH_W +: PH_W]};
        end // for lane_idx
        // set one-hot, already gated with "the record enters S2"
        for (set_idx = 0; set_idx < NS; set_idx = set_idx + 1) begin
            pick_set_on[set_idx] = pick_take & (pick_set == set_idx[SET_W-1:0]);
        end // for set_idx
    end // always lane_pick_proc

    assign pick_take  = pick_any & pf_enable & ~clear;

    // S1: does lane l have the key of the event now in P (the event two
    // before it)? An entry written with that key at the end of this cycle
    // uses this instead of its own S1 compare. Per lane, not gated by the
    // pick: the S2 card ANDs it with the event's one-hot lane in the OR it
    // already has, so the late pick bits do not join the compare here
    always @* begin : p2_key_proc
        integer lane_idx; // training lane
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_p2_key[lane_idx] =
                (lane_tag[lane_idx*TAG_W +: TAG_W]
                 == cp_pevent[EV_W-1 -: TAG_W])
                & (train_is_store[lane_idx] == cp_pevent[EV_W-1-TAG_W]);
        end // for lane_idx
    end // always p2_key_proc

    // S1: does lane l have the page hash of the event now in S2 / of the
    // event before it? A card whose page hash is written with one of
    // those at the end of this cycle uses this instead of its own S1
    // compare (e1_ph / e2_ph hold the same values as cp_event / cp_pevent)
    always @* begin : ph_ev_proc
        integer lane_idx; // training lane
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_ph_ev[lane_idx] = (lane_ph[lane_idx*PH_W +: PH_W] == e1_ph);
            lane_ph_p[lane_idx]  = (lane_ph[lane_idx*PH_W +: PH_W] == e2_ph);
        end // for lane_idx
    end // always ph_ev_proc

    // S1: FOLD_COPIES copies of every lane's PC tag fold for the cards'
    // key compares (each copy feeds the SPF sets x NW cards of its group,
    // instead of one fold driving all 64 cards, T-6)
    genvar fold_g;  // tag fold copy
    genvar flane_g; // training lane of that copy
    generate
        for (fold_g = 0; fold_g < FOLD_COPIES; fold_g = fold_g + 1) begin : g_fold
            for (flane_g = 0; flane_g < NB; flane_g = flane_g + 1) begin : g_lane
                l1d_pf3_fold #(.IN_W(FOLD_W), .OUT_W(TAG_W)) u_tag_fold (
                    .din  (train_pc[flane_g*PC_W + FOLD_LSB +: FOLD_W]),
                    .dout (cp_s1_tag[(fold_g*NB + flane_g)*TAG_W +: TAG_W])
                );
                l1d_pf3_fold #(.IN_W(PPN_W), .OUT_W(PH_W)) u_ph_fold (
                    .din  (train_addr[flane_g*ADDR_W + PAGE_W +: PPN_W]),
                    .dout (cp_s1_ph[(fold_g*NB + flane_g)*PH_W +: PH_W])
                );
            end // for flane_g
        end // for fold_g
    endgenerate
    // PICK_COPIES copies of the picked event word (fanout split, T-6): each
    // copy is the OR of the four masked lane words and drives CPP event
    // registers, instead of one OR driving all E1_COPIES of them
    genvar pick_g; // lane pick copy
    generate
        for (pick_g = 0; pick_g < PICK_COPIES; pick_g = pick_g + 1) begin : g_pick
            l1d_pf3_or_rows #(.W (EV_W), .N (NB)) u_pick (
                .rows   (lane_event),
                .or_out (pk_event[pick_g*EV_W +: EV_W])
            );
        end // for pick_g
    endgenerate

    // ---- S1 -> S2: central register (filter key, page, valid) ----
    always @(posedge clk) begin : s1_reg_proc
        if (rst) begin
            e1_vld <= 1'b0;
        end
        else begin
            e1_vld <= pick_take;
        end // rst
        e1_ph  <= pick_ph;
        e1_st  <= pick_st;
        e1_ppn <= pick_ppn;
    end // always s1_reg_proc

    // ---- S1 -> S2: E1_COPIES copies of the event, each for ENTRIES / E1_COPIES = 4 entries ----
    genvar copy_g; // event register copy
    generate
        for (copy_g = 0; copy_g < E1_COPIES; copy_g = copy_g + 1) begin : g_copy
            // the set-on bits of this copy's SPC sets (reset: no event)
            l1d_pf3_rep #(.W(SPC), .HAS_RST(1), .HAS_EN(0)) u_set_on (
                .clk (clk),
                .rst (rst),
                .en  (1'b1),
                .d   (pick_set_on[copy_g*SPC +: SPC]),
                .q   (cp_set_on[copy_g*SPC +: SPC])
            );
            // the event word
            l1d_pf3_rep #(.W(EV_W), .HAS_RST(0), .HAS_EN(0)) u_event (
                .clk (clk),
                .rst (1'b0),
                .en  (1'b1),
                .d   (pk_event[(copy_g/CPP)*EV_W +: EV_W]),
                .q   (cp_event[copy_g*EV_W +: EV_W])
            );
            // the event's lane (one-hot) and "it has E-2's key": the
            // entries' S2 hit uses them with their S1 lane compares
            l1d_pf3_rep #(.W(4*NB), .HAS_RST(0), .HAS_EN(0)) u_lane (
                .clk (clk),
                .rst (1'b0),
                .en  (1'b1),
                .d   ({lane_pick, lane_p2_key, lane_ph_ev, lane_ph_p}),
                .q   ({cp_lane[copy_g*NB +: NB], cp_p2_key[copy_g*NB +: NB],
                       cp_ph_ev[copy_g*NB +: NB], cp_ph_p[copy_g*NB +: NB]})
            );
            // the event before: written into its allocated entry now
            l1d_pf3_rep #(.W(EV_W), .HAS_RST(0), .HAS_EN(0)) u_pevent (
                .clk (clk),
                .rst (1'b0),
                .en  (1'b1),
                .d   (cp_event[copy_g*EV_W +: EV_W]),
                .q   (cp_pevent[copy_g*EV_W +: EV_W])
            );
            // what a card E-1 allocated becomes if E hits it (shared by the
            // copy's 4 cards; the card picks it with a late 2:1)
            l1d_pf3_stride_anew #(
                .TAG_W  (TAG_W),
                .PAGE_W (PAGE_W),
                .PH_W   (PH_W)
            ) u_anew (
                .ev_tag    (cp_event[copy_g*EV_W + EV_W-1 -: TAG_W]),
                .ev_st     (cp_event[copy_g*EV_W + EV_W-1-TAG_W]),
                .ev_lo     (cp_event[copy_g*EV_W + LO3_W+PH_W +: PAGE_W]),
                .ev_lo3    (cp_event[copy_g*EV_W + PH_W +: LO3_W]),
                .ev_ph     (cp_event[copy_g*EV_W +: PH_W]),
                .p_tag     (cp_pevent[copy_g*EV_W + EV_W-1 -: TAG_W]),
                .p_st      (cp_pevent[copy_g*EV_W + EV_W-1-TAG_W]),
                .p_lo      (cp_pevent[copy_g*EV_W + LO3_W+PH_W +: PAGE_W]),
                .p_lo3     (cp_pevent[copy_g*EV_W + PH_W +: LO3_W]),
                .p_ph      (cp_pevent[copy_g*EV_W +: PH_W]),
                .a_key     (cp_a_key[copy_g]),
                .a_correct (cp_a_correct[copy_g]),
                .a_stride  (cp_a_stride[copy_g*(PAGE_W+1) +: PAGE_W+1]),
                .a_exp     (cp_a_exp[copy_g*(PAGE_W+1) +: PAGE_W+1]),
                .a_s3      (cp_a_s3[copy_g*(PAGE_W+3) +: PAGE_W+3]),
                .a_tgt     (cp_a_tgt[copy_g*(PAGE_W+3) +: PAGE_W+3]),
                .a_nz      (cp_a_nz[copy_g])
            );
        end // for copy_g
    endgenerate

    // ---- the event before's target and the filter keys: LC_N copies
    //      (fewer than the event copies: their sources are spread over the
    //      sets, so each extra copy costs wire, rule T-6) ----
    genvar lcopy_g; // target / filter copy
    generate
        for (lcopy_g = 0; lcopy_g < LC_N; lcopy_g = lcopy_g + 1) begin : g_lcopy
            // the candidate target of the event before (dup check)
            // (OR of the per-set target flops, LC_N copies, each for SPL sets)
            l1d_pf3_or_rows #(.W(LIP_W), .N(NG)) u_prev_lip (
                .rows   (grp_lip_q),
                .or_out (cp_prev_lip[lcopy_g*LIP_W +: LIP_W])
            );
            // the filter's line-in-page keys (shift on cand_valid)
            l1d_pf3_rep #(.W(FL_W), .HAS_RST(0), .HAS_EN(1)) u_filt_lip (
                .clk (clk),
                .rst (1'b0),
                .en  (cand_cp[NB + 3 + lcopy_g]),
                .d   (filt_lip_next),
                .q   (cp_filt_lip[lcopy_g*FL_W +: FL_W])
            );
        end // for lcopy_g
    endgenerate

    // ======================================================================
    // S2: the 64 entries, the victim of each set, the hit's target
    // ======================================================================
    genvar set_g; // table set
    generate
        for (set_g = 0; set_g < NS; set_g = set_g + 1) begin : g_set
            // which copy of the event this set reads
            // copy index = (set_g / SPC), written inline below (no
            // localparam in a generate loop: plain Verilog-2005 for Genus)
            wire                 set_on;                  // event in this set
            wire [TAG_W-1:0]     ev_tag;                  // 12 bits
            wire                 ev_st;                   // store event
            wire [PAGE_W-1:0]    ev_lo;                   // 12 bits
            wire [LO3_W-1:0]     ev_lo3;                  // 14 bits
            wire [PH_W-1:0]      ev_ph;                   // 10 bits
            wire [TAG_W-1:0]     p_tag;                   // 12: event before
            wire                 p_st;                    // its store bit
            wire [PAGE_W-1:0]    p_lo;                    // 12 bits
            wire [LO3_W-1:0]     p_lo3;                   // 14 bits
            wire [PH_W-1:0]      p_ph;                    // 10 bits
            wire [FILT_DEPTH-1:0] filt_pm;                // 4: slot, same page
            wire                 prev_pm;                 // E-1 same page
            wire                 set_miss;                // no way hit
            reg  [NW*3-1:0]      way_class;               // 4 x 3
            reg  [NW-1:0]        victim_way;              // one-hot
            reg                  way_beaten;              // a better victim
            genvar way_g;                                 // table way
            genvar slot_g;                                // filter slot

            assign set_on = cp_set_on[(set_g / SPC)*SPC + (set_g % SPC)];
            assign {ev_tag, ev_st, ev_lo, ev_lo3, ev_ph} =
                   cp_event[(set_g / SPC)*EV_W +: EV_W];
            assign {p_tag, p_st, p_lo, p_lo3, p_ph} =
                   cp_pevent[(set_g / SPC)*EV_W +: EV_W];
            // filter slot k: valid and same page hash as the event
            for (slot_g = 0; slot_g < FILT_DEPTH; slot_g = slot_g + 1) begin : g_pm
                assign filt_pm[slot_g] = filt_vld[slot_g]
                                         & (filt_ph[slot_g*PH_W +: PH_W] == ev_ph);
            end // for slot_g
            // the event before has the same page hash (its key is the one
            // sent in S3 now)
            assign prev_pm = e2_vld & (e2_ph == ev_ph);

            for (way_g = 0; way_g < NW; way_g = way_g + 1) begin : g_way
                // entry index = (set_g * NW + way_g), written inline below
                // (no localparam in a generate loop: plain Verilog-2005)
                assign ent_alloc[(set_g * NW + way_g)] = set_on & set_miss & victim_way[way_g];
                l1d_pf3_stride_entry #(
                    .TAG_W      (TAG_W),
                    .PAGE_W     (PAGE_W),
                    .OFFSET_W   (OFFSET_W),
                    .PH_W       (PH_W),
                    .FILT_DEPTH (FILT_DEPTH),
                    .NL         (NB)
                ) u_entry (
                    .clk        (clk),
                    .rst        (rst),
                    .clear      (clear),
                    .ev_set_on  (set_on),
                    .s1_tag     (cp_s1_tag[(set_g/SPF)*NB*TAG_W +: NB*TAG_W]),
                    .s1_st      (train_is_store),
                    .ev_lane    (cp_lane[(set_g / SPC)*NB +: NB]),
                    .ev_p2_key  (cp_p2_key[(set_g / SPC)*NB +: NB]),
                    .s1_ph      (cp_s1_ph[(set_g/SPF)*NB*PH_W +: NB*PH_W]),
                    .ev_ph_ev   (cp_ph_ev[(set_g / SPC)*NB +: NB]),
                    .ev_ph_p    (cp_ph_p[(set_g / SPC)*NB +: NB]),
                    .ev_tag     (ev_tag),
                    .ev_st      (ev_st),
                    .ev_lo      (ev_lo),
                    .ev_lo3     (ev_lo3),
                    .ev_ph      (ev_ph),
                    .filt_pm    (filt_pm),
                    .filt_lip   (cp_filt_lip[(set_g/SPL)*FL_W +: FL_W]),
                    .prev_pm    (prev_pm),
                    .prev_lip   (cp_prev_lip[(set_g/SPL)*LIP_W +: LIP_W]),
                    .p_tag      (p_tag),
                    .p_st       (p_st),
                    .p_lo       (p_lo),
                    .p_lo3      (p_lo3),
                    .p_ph       (p_ph),
                    .a_key      (cp_a_key[(set_g / SPC)]),
                    .a_correct  (cp_a_correct[(set_g / SPC)]),
                    .a_stride   (cp_a_stride[(set_g / SPC)*(PAGE_W+1) +: PAGE_W+1]),
                    .a_exp      (cp_a_exp[(set_g / SPC)*(PAGE_W+1) +: PAGE_W+1]),
                    .a_s3       (cp_a_s3[(set_g / SPC)*(PAGE_W+3) +: PAGE_W+3]),
                    .a_tgt      (cp_a_tgt[(set_g / SPC)*(PAGE_W+3) +: PAGE_W+3]),
                    .a_nz       (cp_a_nz[(set_g / SPC)]),
                    .alloc_en   (ent_alloc[(set_g * NW + way_g)]),
                    .hit        (ent_hit[(set_g * NW + way_g)]),
                    .class_rank (ent_class[(set_g * NW + way_g)*3 +: 3]),
                    .fire_new   (ent_fire_new[(set_g * NW + way_g)]),
                    .fire_dup   (ent_fire_dup[(set_g * NW + way_g)]),
                    .tgt_lip    (ent_tgt_lip[(set_g * NW + way_g)*LIP_W +: LIP_W])
                );
            end // for way_g

            assign set_miss = ~(|ent_hit[set_g*NW +: NW]);

            // victim: the way with the lowest class, ties -> lowest way
            // (classes are the entries' flops: up to date, one at a time)
            always @* begin : victim_proc
                integer way_idx;   // candidate way
                integer other_idx; // way compared with
                way_class = ent_class[set_g*NW*3 +: NW*3];
                for (way_idx = 0; way_idx < NW; way_idx = way_idx + 1) begin
                    way_beaten = 1'b0;
                    for (other_idx = 0; other_idx < NW; other_idx = other_idx + 1) begin
                        if (other_idx != way_idx) begin
                            if ((way_class[other_idx*3 +: 3] < way_class[way_idx*3 +: 3])
                                || ((way_class[other_idx*3 +: 3] == way_class[way_idx*3 +: 3])
                                    && (other_idx < way_idx))) begin
                                way_beaten = 1'b1;
                            end
                        end
                    end // for other_idx
                    victim_way[way_idx] = ~way_beaten;
                end // for way_idx
            end // always victim_proc

            // per set fire flags (only the event's set can fire)
            always @(posedge clk) begin : set_fire_proc
                if (rst || clear) begin
                    fire_new_q[set_g] <= 1'b0;
                    fire_dup_q[set_g] <= 1'b0;
                end
                else begin
                    fire_new_q[set_g] <= |ent_fire_new[set_g*NW +: NW];
                    fire_dup_q[set_g] <= |ent_fire_dup[set_g*NW +: NW];
                end // rst
            end // always set_fire_proc

            // per set target of its hit entry (0 when no hit): a flat
            // one-hot AND-OR (at most one way hits, checked under
            // L1D_DEBUG), not a priority chain of 2:1 selects (rule T-9)
            reg [LIP_W-1:0] set_lip_next; // 6 bits: the hit way's target
            always @* begin : set_lip_next_proc
                integer way_idx; // walks the ways
                set_lip_next = {LIP_W{1'b0}};
                for (way_idx = 0; way_idx < NW; way_idx = way_idx + 1) begin
                    set_lip_next = set_lip_next
                        | ({LIP_W{ent_hit[set_g*NW + way_idx]}}
                           & ent_tgt_lip[(set_g*NW + way_idx)*LIP_W +: LIP_W]);
                end // for way_idx
            end // always set_lip_next_proc
            // the same pick for the bank, decoded before the flop: each
            // way's stored target bank (flops) is decoded early, the hit
            // picks it, so S3 needs no decode after its 16-input OR
            reg [NB-1:0] set_bank_next; // 4: the hit way's bank, one-hot
            always @* begin : set_bank_next_proc
                integer way_idx;  // walks the ways
                integer bank_idx; // walks the banks
                set_bank_next = {NB{1'b0}};
                for (way_idx = 0; way_idx < NW; way_idx = way_idx + 1) begin
                    for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
                        set_bank_next[bank_idx] = set_bank_next[bank_idx]
                            | (ent_hit[set_g*NW + way_idx]
                               & (ent_tgt_lip[(set_g*NW + way_idx)*LIP_W +: BANK_W]
                                  == bank_idx[BANK_W-1:0]));
                    end // for bank_idx
                end // for way_idx
            end // always set_bank_next_proc
            assign set_lip_nx[set_g*LIP_W +: LIP_W] = set_lip_next;
            assign set_bank_nx[set_g*NB +: NB]     = set_bank_next;
        end // for set_g
    endgenerate

    // per group of SPG sets: the OR of its sets' target and bank (only
    // the event's set can hit, so at most one set of a group is non-zero)
    reg  [NG*LIP_W-1:0] grp_lip_next;  // 4 x 6: OR of each group's sets
    reg  [NG*NB-1:0]    grp_bank_next; // 4 x 4: same for the bank
    always @* begin : grp_next_proc
        integer grp_idx; // walks the groups
        integer sub_idx; // walks the sets of a group
        grp_lip_next  = {(NG*LIP_W){1'b0}};
        grp_bank_next = {(NG*NB){1'b0}};
        for (grp_idx = 0; grp_idx < NG; grp_idx = grp_idx + 1) begin
            for (sub_idx = 0; sub_idx < SPG; sub_idx = sub_idx + 1) begin
                grp_lip_next[grp_idx*LIP_W +: LIP_W] =
                    grp_lip_next[grp_idx*LIP_W +: LIP_W]
                    | set_lip_nx[(grp_idx*SPG + sub_idx)*LIP_W +: LIP_W];
                grp_bank_next[grp_idx*NB +: NB] =
                    grp_bank_next[grp_idx*NB +: NB]
                    | set_bank_nx[(grp_idx*SPG + sub_idx)*NB +: NB];
            end // for sub_idx
        end // for grp_idx
    end // always grp_next_proc
    always @(posedge clk) begin : grp_lip_proc
        grp_lip_q     <= grp_lip_next;
        grp_bank_oh_q <= grp_bank_next;
    end // always grp_lip_proc

    // the hit entry's stored target: per set, picked by the set's hit
    // vector (one-hot), ORed per group and registered; only the event's
    // set can hit, so in S3 the OR of the NG group flops is the target
    // (used only when a set flag fires)
    l1d_pf3_or_rows #(.W(LIP_W), .N(NG)) u_cand_lip (
        .rows   (grp_lip_q),
        .or_out (cand_lip_q)
    );
    l1d_pf3_or_rows #(.W(LIP_W), .N(NG)) u_out_lip (
        .rows   (grp_lip_q),
        .or_out (out_lip)
    );
    // the candidate bank: OR of the per-set one-hot banks (kept copy,
    // T-6); 0000 when no set hit, only read with cand_valid (the rank's
    // no-new-stride cases use its held bank)
    l1d_pf3_or_rows #(.W(NB), .N(NG)) u_bank_oh (
        .rows   (grp_bank_oh_q),
        .or_out (bank_oh)
    );

    // ---- S2 -> S3 registers ----
    always @(posedge clk) begin : s2_reg_proc
        if (rst) begin
            e2_vld <= 1'b0;
        end
        else begin
            e2_vld <= e1_vld & ~clear;
        end // rst
        e2_st      <= e1_st;
        e2_ph      <= e1_ph;
        e2_ppn     <= e1_ppn;
    end // always s2_reg_proc

    // ======================================================================
    // S3: candidate out, filter update
    // ======================================================================
    // the event before sent a key -> filter slot 3 is out of the window and
    // the event before's key is in it (the entry flagged both cases)
    // the candidate valid is late (an OR of 16 set flags) and drives many
    // loads, so it is built CV_N times, each from its own OR of the flags
    // (kept hierarchy, rule T-6): 0 = rank control, 1 .. NB+1 = rank
    // data (one per bank line select + one for the hold loads), NB+2 =
    // the filter registers here, NB+3 .. = one per filter-key copy
    genvar cvcopy_g; // candidate valid copy
    generate
        for (cvcopy_g = 0; cvcopy_g < CV_N; cvcopy_g = cvcopy_g + 1) begin : g_cv
            wire any_dup; // a set fired, E-1's key not hit
            wire any_new; // a set fired, filter slot 3 not hit
            l1d_pf3_or_rows #(.W (1), .N (NS)) u_dup_or (
                .rows (fire_dup_q), .or_out (any_dup)
            );
            l1d_pf3_or_rows #(.W (1), .N (NS)) u_new_or (
                .rows (fire_new_q), .or_out (any_new)
            );
            assign cand_cp[cvcopy_g] = (prev_ins_q ? any_dup : any_new)
                                       & pf_enable & ~clear;
        end // for cvcopy_g
    endgenerate
    assign cand_valid    = cand_cp[0];
    assign cand_valid_dp = cand_cp[NB+1:1];
    assign cand_line   = {{PAD_W{1'b0}}, e2_ppn, out_lip};
    assign cand_unique = e2_st;

    always @* begin : bank_proc
        integer bank_idx; // cache bank
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
            cand_bank[bank_idx] = bank_oh[bank_idx];
        end // for bank_idx
    end // always bank_proc

    // filter: newest key into slot 0 when the candidate is sent
    assign filt_lip_next = {cp_filt_lip[FL_W-LIP_W-1:0], cand_lip_q};

    always @(posedge clk) begin : filter_proc
        if (rst || clear) begin
            filt_vld   <= {FILT_DEPTH{1'b0}};
            prev_ins_q <= 1'b0;
        end
        else begin
            prev_ins_q <= cand_cp[NB+2];
            if (cand_cp[NB+2]) begin
                filt_vld <= {filt_vld[FILT_DEPTH-2:0], 1'b1};
            end
        end // rst
        if (cand_cp[NB+2]) begin
            filt_ph <= {filt_ph[(FILT_DEPTH-1)*PH_W-1:0], e2_ph};
        end
    end // always filter_proc

`ifdef L1D_DEBUG
    // ---- checks (simulation only) ----
    initial begin : dist_check
        if (DIST != 2) begin
            $display("L1D PF3_STRIDE ERROR DIST must be 2 (A4), is %0d", DIST);
        end
    end // initial dist_check
    always @(posedge clk) begin : debug_proc
        if (!rst) begin
            if ((ent_hit & (ent_hit - 1'b1)) != {ENTRIES{1'b0}}) begin
                $display("[%0t] L1D PF3_STRIDE ERROR two entries hit", $time);
            end
            if ((fire_new_q & (fire_new_q - 1'b1)) != {NS{1'b0}}) begin
                $display("[%0t] L1D PF3_STRIDE ERROR two sets fired", $time);
            end
            if ((ent_alloc & ent_hit) != {ENTRIES{1'b0}}) begin
                $display("[%0t] L1D PF3_STRIDE ERROR allocation on a hit", $time);
            end
        end // !rst
    end // always debug_proc
`endif

endmodule

`default_nettype wire
`endif // L1D_PF3_STRIDE_V
