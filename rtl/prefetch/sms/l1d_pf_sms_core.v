// ==========================================================================
// l1d_pf_sms_core.v - SMS learning core: FT, AT and PHT tables
// ==========================================================================
// IN SIMPLE WORDS : SMS learns which lines of a 2 KB region a piece of
//   code touches. The first access to a region (the "trigger", by PC X
//   at line L) is written on a sticky note in the FILTER table (FT).
//   A second access to a different line moves the note to the
//   ACCUMULATION table (AT), where every further access ticks a box in
//   a 32-box footprint. When a used line leaves the cache (eviction
//   event) the footprint is final: it is filed in the PATTERN HISTORY
//   table (PHT) under (X, L). The next time X triggers a new region at
//   line L, the filed footprint is looked up and handed to the
//   expander, which fetches those lines.
//   One event (training or eviction) is processed per cycle.
// STAGES (each output is a flop, T-1, T-5):
//   S0  pick event (train/evict), hash, line decode      -> e_* regs
//   S1  tag compare FT/AT, victim pick, PHT set read     -> f_* regs
//   S2  decide, write FT/AT, PHT lookup, hand to expander
//   W   PHT write (one write per event, one cycle)
//   Trigger to slot: 3 stages (S0, S1, S2) after the FIFO; the README
//   target is about 4 stages from FIFO to the first request.
// INSTANTIATED BY : l1d_pf_sms, instance u_core, 1 copy.
//
// PARAMETERS:
//   name           default unit    meaning
//   LINE_ADDR_W    50      bits    physical line address
//   LIR_W          5       bits    line number inside a region (2 KB)
//   PC_HASH_W      12      bits    PC hash
//   REGION_TAG_W   16      bits    region tag (XOR fold)
//   FT_ENTRIES     16      entries filter table (power of two)
//   AT_ENTRIES     32      entries accumulation table (power of two)
//   PHT_SETS       32      sets    pattern history sets (power of two)
//   PHT_WAYS       4       ways    pattern history ways (power of two)
//   PHT_TAG_W      8       bits    pattern history tag
//
// PORTS (default widths):
//   clk, rst                  in  1    clock, synchronous reset (high)
//   train_head_valid          in  1    training record waiting
//   train_head_pc_hash        in  12   its PC hash
//   train_head_line_addr      in  50   its line address
//   evt_head_valid            in  1    eviction event waiting
//   evt_head_line_addr        in  50   its line address
//   train_pop, evt_pop        out 1    take the FIFO head (comb.)
//   load_valid                out 1    pattern for the expander (comb.)
//   load_region               out 45   region number
//   load_mask                 out 32   lines to fetch (trigger cleared)
//   load_line                 out 5    trigger line
//
// WORKED EXAMPLE (README section 10, PC hash X, lines {0,3,6}):
//   1 X, line 0, region R1 -> trigger: FT[R1] = (X,0), PHT miss
//   2 line 3 of R1         -> FT hit, new line: AT[R1] pattern {0,3}
//   3 line 6 of R1         -> AT hit: pattern {0,3,6}
//   4 evict line 3 of R1   -> AT hit, bit 3 set: PHT[(X,0)] = {0,3,6}
//   5 X, line 0, region R2 -> trigger: PHT hit -> load_mask {3,6}
//
// ASSUMPTIONS (V-9.2), checked under L1D_DEBUG:
//   A1 table and set sizes are powers of two; FT_ENTRIES, AT_ENTRIES >= 2.
//   A2 one event per cycle; the PHT is a hint: a read that races with a
//      PHT write may see the old pattern (costs one prefetch, no harm).
//   A3 a promote needs a second DIFFERENT line (the same line again
//      leaves the note in the FT). README says "second access";
//      original SMS promotes on a second distinct block (open SM-14).
//   A4 AT / FT replacement without a free slot: the slot a ring
//      pointer shows. The pointer moves once per training event while
//      the table is full, so the victim is a rotating pick, NOT the
//      oldest and not by last use (README: 5-bit age, replace the
//      oldest; no age field here, open SM-15). A hint table only.
// COMBINATIONAL OUTPUTS (V-3.11): train_pop, evt_pop, load_* (S0 / S2
//   decisions that feed flops in the next module).
//
// TIMING (T-9, ESTIMATES, nothing synthesised): S0 about 8 levels,
//   S1 about 10 (16-bit compare 5, 32:1 PHT set mux 5 in parallel),
//   S2 about 14 (one-hot read 6, decision 4, write enable 2, PHT key
//   fold 3 for the W register), W about 12 (set read, way pick, write).
//   GR-1 STATUS: NOT PROVEN. S2 and W are expected to need rework
//   (split into two registered steps) once synthesis names the gap.
// ==========================================================================
`ifndef L1D_PF_SMS_CORE_V
`define L1D_PF_SMS_CORE_V
`default_nettype none

module l1d_pf_sms_core #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter LINE_ADDR_W  = 50, // default 50 bits: line address
    parameter LIR_W        = 5,  // default 5 bits: line in region
    parameter PC_HASH_W    = 12, // default 12 bits: PC hash
    parameter REGION_TAG_W = 16, // default 16 bits: region tag
    parameter FT_ENTRIES   = 16, // default 16: filter table
    parameter AT_ENTRIES   = 32, // default 32: accumulation table
    parameter PHT_SETS     = 32, // default 32: pattern history sets
    parameter PHT_WAYS     = 4,  // default 4: pattern history ways
    parameter PHT_TAG_W    = 8   // default 8 bits: pattern history tag
) (
    // ---- clock and reset ----
    input  wire                         clk,                  // clock
    input  wire                         rst,                  // sync reset
    // ---- records from the front end ----
    input  wire                         train_head_valid,     // waiting
    input  wire [PC_HASH_W-1:0]         train_head_pc_hash,   // 12 bits
    input  wire [LINE_ADDR_W-1:0]       train_head_line_addr, // 50 bits
    input  wire                         evt_head_valid,       // waiting
    input  wire [LINE_ADDR_W-1:0]       evt_head_line_addr,   // 50 bits
    output wire                         train_pop,            // take head
    output wire                         evt_pop,              // take head
    // ---- pattern to the expander (combinational) ----
    output wire                         load_valid,           // pattern
    output wire [LINE_ADDR_W-LIR_W-1:0] load_region,          // 45 bits
    output wire [(1<<LIR_W)-1:0]        load_mask,            // 32 bits
    output wire [LIR_W-1:0]             load_line             // 5 bits
);

    // ---- local constants (value at the default) ----
    localparam REGION_NUM_W = LINE_ADDR_W - LIR_W;     // 45 bits
    localparam PATTERN_W    = (1 << LIR_W);            // 32 bits
    localparam KEY_W        = PC_HASH_W + LIR_W;       // 17 bits PHT key
    localparam SET_W        = $clog2(PHT_SETS);        // 5 bits
    localparam WAY_W        = $clog2(PHT_WAYS);        // 2 bits
    localparam FT_IDX_W     = $clog2(FT_ENTRIES);      // 4 bits
    localparam AT_IDX_W     = $clog2(AT_ENTRIES);      // 5 bits
    localparam PHT_TOTAL    = PHT_SETS * PHT_WAYS;     // 128 entries

    // ---- filter table (FT), entry i at [i*W +: W] ----
    reg  [FT_ENTRIES-1:0]                ft_vld_q;   // 16: in use
    reg  [FT_ENTRIES*REGION_TAG_W-1:0]   ft_tag_q;   // 16 x 16
    reg  [FT_ENTRIES*PC_HASH_W-1:0]      ft_pc_q;    // 16 x 12: trigger PC
    reg  [FT_ENTRIES*LIR_W-1:0]          ft_line_q;  // 16 x 5: trigger line
    reg  [FT_IDX_W-1:0]                  ft_ring_q;  // 4: oldest slot
    // ---- accumulation table (AT) ----
    reg  [AT_ENTRIES-1:0]                at_vld_q;   // 32: in use
    reg  [AT_ENTRIES*REGION_TAG_W-1:0]   at_tag_q;   // 32 x 16
    reg  [AT_ENTRIES*PC_HASH_W-1:0]      at_pc_q;    // 32 x 12
    reg  [AT_ENTRIES*LIR_W-1:0]          at_line_q;  // 32 x 5
    reg  [AT_ENTRIES*PATTERN_W-1:0]      at_pat_q;   // 32 x 32: footprint
    reg  [AT_IDX_W-1:0]                  at_ring_q;  // 5: oldest slot
    // ---- pattern history table (PHT), entry (set*WAYS + way) ----
    reg  [PHT_TOTAL-1:0]                 pht_vld_q;  // 128
    reg  [PHT_TOTAL*PHT_TAG_W-1:0]       pht_tag_q;  // 128 x 8
    reg  [PHT_TOTAL*PATTERN_W-1:0]       pht_pat_q;  // 128 x 32
    reg  [PHT_SETS*WAY_W-1:0]            pht_ptr_q;  // 32 x 2: replace way

    // ---- S0: pick the event ----
    reg                                  prefer_evt_q;  // 1: evict first
    wire                                 take_evt;      // 1: take evict
    wire                                 take_train;    // 1: take train
    wire [LINE_ADDR_W-1:0]               s0_line_addr;  // 50
    wire [REGION_NUM_W-1:0]              s0_region;     // 45
    wire [LIR_W-1:0]                     s0_line;       // 5
    wire [REGION_TAG_W-1:0]              s0_tag;        // 16
    wire [KEY_W-1:0]                     s0_key;        // 17
    wire [PATTERN_W-1:0]                 s0_line_oh;    // 32
    // ---- S0 -> S1 registers ----
    reg                                  e_vld_q;       // 1: event
    reg                                  e_evt_q;       // 1: is eviction
    reg  [REGION_NUM_W-1:0]              e_region_q;    // 45
    reg  [REGION_TAG_W-1:0]              e_tag_q;       // 16
    reg  [LIR_W-1:0]                     e_line_q;      // 5
    reg  [PATTERN_W-1:0]                 e_line_oh_q;   // 32
    reg  [PC_HASH_W-1:0]                 e_pc_q;        // 12
    reg  [SET_W-1:0]                     e_set_q;       // 5: PHT set
    reg  [PHT_TAG_W-1:0]                 e_ptag_q;      // 8: PHT tag

    // ---- S1: compare, victims, PHT read ----
    reg  [FT_ENTRIES-1:0]                s1_ft_hit;     // 16
    reg  [AT_ENTRIES-1:0]                s1_at_hit;     // 32
    wire                                 s1_same_prev;  // 1
    reg  [FT_ENTRIES-1:0]                ft_free;       // 16: free, usable
    reg  [FT_ENTRIES-1:0]                s1_ft_vic;     // 16: FT victim
    reg  [AT_ENTRIES-1:0]                at_free;       // 32: free, usable
    reg  [AT_ENTRIES-1:0]                s1_at_vic;     // 32: AT victim
    reg                                  ft_seen;       // 1: loop flag
    reg                                  at_seen;       // 1: loop flag
    reg  [PHT_WAYS-1:0]                  s1_pht_vld;    // 4
    reg  [PHT_WAYS*PHT_TAG_W-1:0]        s1_pht_tag;    // 4 x 8
    reg  [PHT_WAYS*PATTERN_W-1:0]        s1_pht_pat;    // 4 x 32
    integer                              pht_base;      // set * ways
    // ---- S1 -> S2 registers ----
    reg                                  f_vld_q;       // 1: event
    reg                                  f_evt_q;       // 1: is eviction
    reg  [REGION_NUM_W-1:0]              f_region_q;    // 45
    reg  [REGION_TAG_W-1:0]              f_tag_q;       // 16
    reg  [LIR_W-1:0]                     f_line_q;      // 5
    reg  [PATTERN_W-1:0]                 f_line_oh_q;   // 32
    reg  [PC_HASH_W-1:0]                 f_pc_q;        // 12
    reg  [PHT_TAG_W-1:0]                 f_ptag_q;      // 8
    reg  [FT_ENTRIES-1:0]                f_ft_hit_q;    // 16
    reg  [AT_ENTRIES-1:0]                f_at_hit_q;    // 32
    reg                                  f_same_prev_q; // 1
    reg  [FT_ENTRIES-1:0]                f_ft_vic_q;    // 16
    reg  [AT_ENTRIES-1:0]                f_at_vic_q;    // 32
    reg  [PHT_WAYS-1:0]                  f_pht_vld_q;   // 4
    reg  [PHT_WAYS*PHT_TAG_W-1:0]        f_pht_tag_q;   // 4 x 8
    reg  [PHT_WAYS*PATTERN_W-1:0]        f_pht_pat_q;   // 4 x 32
    reg  [FT_ENTRIES-1:0]                p_ft_alloc_q;  // 16: FT slot the
                                                        //   event before made
    reg  [AT_ENTRIES-1:0]                p_at_alloc_q;  // 32: AT slot made

    // ---- S2: decision ----
    wire                                 fwd_ft;        // 1
    wire                                 fwd_at;        // 1
    reg  [FT_ENTRIES-1:0]                ft_eff;        // 16: FT entry used
    reg  [AT_ENTRIES-1:0]                at_eff;        // 32: AT entry used
    wire                                 any_ft;        // 1
    wire                                 any_at;        // 1
    reg  [PC_HASH_W-1:0]                 ft_rd_pc;      // 12
    reg  [LIR_W-1:0]                     ft_rd_line;    // 5
    reg  [PC_HASH_W-1:0]                 at_rd_pc;      // 12
    reg  [LIR_W-1:0]                     at_rd_line;    // 5
    reg  [PATTERN_W-1:0]                 at_rd_pat;     // 32
    reg                                  vic_rd_vld;    // 1: AT victim used
    reg  [PC_HASH_W-1:0]                 vic_rd_pc;     // 12
    reg  [LIR_W-1:0]                     vic_rd_line;   // 5
    reg  [PATTERN_W-1:0]                 vic_rd_pat;    // 32
    wire                                 do_at_set;     // 1: tick a box
    wire                                 do_promote;    // 1: FT -> AT
    wire                                 do_trigger;    // 1: new region
    wire                                 do_end_gen;    // 1: file pattern
    wire                                 do_ft_discard; // 1: drop FT note
    wire                                 pw_req;        // 1: PHT write
    reg  [PC_HASH_W-1:0]                 pw_pc;         // 12
    reg  [LIR_W-1:0]                     pw_line;       // 5
    reg  [PATTERN_W-1:0]                 pw_pat;        // 32
    wire [KEY_W-1:0]                     pw_key;        // 17
    wire [PATTERN_W-1:0]                 ft_line_oh;    // 32: FT trigger
    reg  [PHT_WAYS-1:0]                  pht_hit_way;   // 4
    reg  [PATTERN_W-1:0]                 pht_hit_pat;   // 32
    wire [PATTERN_W-1:0]                 pht_pat_rest;  // 32: no trigger bit
    // ---- W stage (PHT write) ----
    reg                                  w_vld_q;       // 1
    reg  [SET_W-1:0]                     w_set_q;       // 5
    reg  [PHT_TAG_W-1:0]                 w_tag_q;       // 8
    reg  [PATTERN_W-1:0]                 w_pat_q;       // 32
    reg  [PHT_WAYS-1:0]                  w_match;       // 4: same tag
    reg  [PHT_WAYS-1:0]                  w_free;        // 4: unused way
    reg  [PHT_WAYS-1:0]                  w_way_one;     // 4: way written
    reg  [WAY_W-1:0]                     w_ptr;         // 2: ring way
    reg                                  w_seen;        // 1: loop flag
    reg                                  w_adv;         // 1: ring moves

    integer entry_idx;   // walks table entries
    integer way_idx;     // walks PHT ways
    integer w_base;      // PHT entry base of the write set

    // ---- helper functions (XOR folds and decode) ----
    function [REGION_TAG_W-1:0] fold_region;
        input [REGION_NUM_W-1:0] region_number;
        integer bit_idx;
        reg [REGION_TAG_W-1:0] folded;
        begin
            folded = {REGION_TAG_W{1'b0}};
            for (bit_idx = 0; bit_idx < REGION_NUM_W; bit_idx = bit_idx + 1) begin
                folded[bit_idx % REGION_TAG_W] =
                    folded[bit_idx % REGION_TAG_W] ^ region_number[bit_idx];
            end // for bit_idx
            fold_region = folded;
        end
    endfunction

    function [SET_W-1:0] fold_set;
        input [KEY_W-1:0] key_value;
        integer bit_idx;
        reg [SET_W-1:0] folded;
        begin
            folded = {SET_W{1'b0}};
            for (bit_idx = 0; bit_idx < KEY_W; bit_idx = bit_idx + 1) begin
                folded[bit_idx % SET_W] = folded[bit_idx % SET_W]
                                          ^ key_value[bit_idx];
            end // for bit_idx
            fold_set = folded;
        end
    endfunction

    function [PHT_TAG_W-1:0] fold_ptag;
        input [KEY_W-1:0] key_value;
        integer bit_idx;
        reg [PHT_TAG_W-1:0] folded;
        begin
            folded = {PHT_TAG_W{1'b0}};
            // shifted by 3 so the tag is not a copy of the set bits
            for (bit_idx = 0; bit_idx < KEY_W; bit_idx = bit_idx + 1) begin
                folded[(bit_idx + 3) % PHT_TAG_W] =
                    folded[(bit_idx + 3) % PHT_TAG_W] ^ key_value[bit_idx];
            end // for bit_idx
            fold_ptag = folded;
        end
    endfunction

    function [PATTERN_W-1:0] one_hot_line;
        input [LIR_W-1:0] line_number;
        reg [PATTERN_W-1:0] decoded;
        begin
            decoded              = {PATTERN_W{1'b0}};
            decoded[line_number] = 1'b1;
            one_hot_line         = decoded;
        end
    endfunction

    // ======================================================================
    // S0: choose training or eviction (alternate when both wait)
    // ======================================================================
    assign take_evt     = evt_head_valid & (~train_head_valid | prefer_evt_q);
    assign take_train   = train_head_valid & ~take_evt;
    assign train_pop    = take_train;
    assign evt_pop      = take_evt;
    assign s0_line_addr = take_evt ? evt_head_line_addr : train_head_line_addr;
    assign s0_region    = s0_line_addr[LINE_ADDR_W-1:LIR_W];
    assign s0_line      = s0_line_addr[LIR_W-1:0];
    assign s0_tag       = fold_region(s0_region);
    assign s0_key       = {train_head_pc_hash, s0_line};
    assign s0_line_oh   = one_hot_line(s0_line);

    always @(posedge clk) begin : s0_register_proc
        if (rst) begin
            prefer_evt_q <= 1'b0;
            e_vld_q      <= 1'b0;
        end
        else begin
            if (evt_head_valid && train_head_valid) begin
                prefer_evt_q <= ~prefer_evt_q;
            end
            e_vld_q <= take_evt | take_train;
        end // else (rst)
        // data flops need no reset: e_vld_q guards them (S-6)
        e_evt_q     <= take_evt;
        e_region_q  <= s0_region;
        e_tag_q     <= s0_tag;
        e_line_q    <= s0_line;
        e_line_oh_q <= s0_line_oh;
        e_pc_q      <= train_head_pc_hash;
        e_set_q     <= fold_set(s0_key);
        e_ptag_q    <= fold_ptag(s0_key);
    end // always s0_register_proc

    // ======================================================================
    // S1: compare with FT and AT, pick victims, read the PHT set
    // ======================================================================
    always @* begin : s1_compare_proc
        integer entry_idx; // walks table entries
        for (entry_idx = 0; entry_idx < FT_ENTRIES; entry_idx = entry_idx + 1) begin
            s1_ft_hit[entry_idx] = ft_vld_q[entry_idx]
                & (ft_tag_q[entry_idx*REGION_TAG_W +: REGION_TAG_W] == e_tag_q);
        end // for entry_idx
        for (entry_idx = 0; entry_idx < AT_ENTRIES; entry_idx = entry_idx + 1) begin
            s1_at_hit[entry_idx] = at_vld_q[entry_idx]
                & (at_tag_q[entry_idx*REGION_TAG_W +: REGION_TAG_W] == e_tag_q);
        end // for entry_idx
    end // always s1_compare_proc

    // same region as the event now in S2 (its table change is not seen)
    assign s1_same_prev = f_vld_q & (f_tag_q == e_tag_q);

    // victims: a free slot first (never the one the S2 event takes),
    // else the slot the ring pointer shows (oldest allocation)
    always @* begin : s1_victim_proc
        integer entry_idx; // walks table entries
        ft_seen = 1'b0;
        at_seen = 1'b0;
        s1_ft_vic = {FT_ENTRIES{1'b0}};
        s1_at_vic = {AT_ENTRIES{1'b0}};
        for (entry_idx = 0; entry_idx < FT_ENTRIES; entry_idx = entry_idx + 1) begin
            ft_free[entry_idx] = ~ft_vld_q[entry_idx]
                                 & ~(f_vld_q & f_ft_vic_q[entry_idx]);
        end // for entry_idx
        for (entry_idx = 0; entry_idx < AT_ENTRIES; entry_idx = entry_idx + 1) begin
            at_free[entry_idx] = ~at_vld_q[entry_idx]
                                 & ~(f_vld_q & f_at_vic_q[entry_idx]);
        end // for entry_idx
        for (entry_idx = 0; entry_idx < FT_ENTRIES; entry_idx = entry_idx + 1) begin
            if (ft_free[entry_idx] && !ft_seen) begin
                s1_ft_vic[entry_idx] = 1'b1;
                ft_seen              = 1'b1;
            end
        end // for entry_idx
        if (!ft_seen) begin
            s1_ft_vic[ft_ring_q] = 1'b1;
        end
        for (entry_idx = 0; entry_idx < AT_ENTRIES; entry_idx = entry_idx + 1) begin
            if (at_free[entry_idx] && !at_seen) begin
                s1_at_vic[entry_idx] = 1'b1;
                at_seen              = 1'b1;
            end
        end // for entry_idx
        if (!at_seen) begin
            s1_at_vic[at_ring_q] = 1'b1;
        end
    end // always s1_victim_proc

    // read the PHT set of the event (4 ways)
    always @* begin : s1_pht_read_proc
        integer way_idx;   // walks PHT ways
        pht_base   = e_set_q * PHT_WAYS;
        s1_pht_vld = {PHT_WAYS{1'b0}};
        s1_pht_tag = {(PHT_WAYS*PHT_TAG_W){1'b0}};
        s1_pht_pat = {(PHT_WAYS*PATTERN_W){1'b0}};
        for (way_idx = 0; way_idx < PHT_WAYS; way_idx = way_idx + 1) begin
            s1_pht_vld[way_idx] = pht_vld_q[pht_base + way_idx];
            s1_pht_tag[way_idx*PHT_TAG_W +: PHT_TAG_W] =
                pht_tag_q[(pht_base + way_idx)*PHT_TAG_W +: PHT_TAG_W];
            s1_pht_pat[way_idx*PATTERN_W +: PATTERN_W] =
                pht_pat_q[(pht_base + way_idx)*PATTERN_W +: PATTERN_W];
        end // for way_idx
    end // always s1_pht_read_proc

    always @(posedge clk) begin : s1_register_proc
        if (rst) begin
            f_vld_q       <= 1'b0;
            f_same_prev_q <= 1'b0;
            f_tag_q       <= {REGION_TAG_W{1'b0}};
            f_ft_hit_q    <= {FT_ENTRIES{1'b0}};
            f_at_hit_q    <= {AT_ENTRIES{1'b0}};
            f_ft_vic_q    <= {FT_ENTRIES{1'b0}};
            f_at_vic_q    <= {AT_ENTRIES{1'b0}};
        end
        else begin
            f_vld_q       <= e_vld_q;
            f_same_prev_q <= s1_same_prev;
            f_tag_q       <= e_tag_q;
            f_ft_hit_q    <= s1_ft_hit;
            f_at_hit_q    <= s1_at_hit;
            f_ft_vic_q    <= s1_ft_vic;
            f_at_vic_q    <= s1_at_vic;
        end // else (rst)
        f_evt_q     <= e_evt_q;
        f_region_q  <= e_region_q;
        f_line_q    <= e_line_q;
        f_line_oh_q <= e_line_oh_q;
        f_pc_q      <= e_pc_q;
        f_ptag_q    <= e_ptag_q;
        f_pht_vld_q <= s1_pht_vld;
        f_pht_tag_q <= s1_pht_tag;
        f_pht_pat_q <= s1_pht_pat;
    end // always s1_register_proc

    // ======================================================================
    // S2: decide what the event does
    // ======================================================================
    // Hazard with the event before (it wrote the tables at the end of
    // the last cycle, so the tables are up to date, but S1 compared
    // before that):
    //  - it made an FT / AT slot for this region: use that slot;
    //  - it freed or re-used a slot S1 matched: drop that stale match
    //    (freed: the valid bit is 0 now; re-used: p_*_alloc_q).
    assign fwd_ft = f_same_prev_q & (|p_ft_alloc_q);
    assign fwd_at = f_same_prev_q & (|p_at_alloc_q);
    always @* begin : s2_effective_proc
        if (fwd_at) begin
            ft_eff = {FT_ENTRIES{1'b0}};
            at_eff = p_at_alloc_q;
        end
        else if (fwd_ft) begin
            ft_eff = p_ft_alloc_q;
            at_eff = {AT_ENTRIES{1'b0}};
        end
        else begin
            ft_eff = f_ft_hit_q & ft_vld_q & ~p_ft_alloc_q;
            at_eff = f_at_hit_q & at_vld_q & ~p_at_alloc_q;
        end // else (no forward)
    end // always s2_effective_proc
    assign any_ft = |ft_eff;
    assign any_at = |at_eff;

    // one-hot AND-OR reads of the matched entries and the AT victim
    always @* begin : s2_read_proc
        integer entry_idx; // walks table entries
        ft_rd_pc    = {PC_HASH_W{1'b0}};
        ft_rd_line  = {LIR_W{1'b0}};
        at_rd_pc    = {PC_HASH_W{1'b0}};
        at_rd_line  = {LIR_W{1'b0}};
        at_rd_pat   = {PATTERN_W{1'b0}};
        vic_rd_vld  = 1'b0;
        vic_rd_pc   = {PC_HASH_W{1'b0}};
        vic_rd_line = {LIR_W{1'b0}};
        vic_rd_pat  = {PATTERN_W{1'b0}};
        for (entry_idx = 0; entry_idx < FT_ENTRIES; entry_idx = entry_idx + 1) begin
            ft_rd_pc   = ft_rd_pc | (ft_pc_q[entry_idx*PC_HASH_W +: PC_HASH_W]
                                     & {PC_HASH_W{ft_eff[entry_idx]}});
            ft_rd_line = ft_rd_line | (ft_line_q[entry_idx*LIR_W +: LIR_W]
                                       & {LIR_W{ft_eff[entry_idx]}});
        end // for entry_idx
        for (entry_idx = 0; entry_idx < AT_ENTRIES; entry_idx = entry_idx + 1) begin
            at_rd_pc    = at_rd_pc | (at_pc_q[entry_idx*PC_HASH_W +: PC_HASH_W]
                                      & {PC_HASH_W{at_eff[entry_idx]}});
            at_rd_line  = at_rd_line | (at_line_q[entry_idx*LIR_W +: LIR_W]
                                        & {LIR_W{at_eff[entry_idx]}});
            at_rd_pat   = at_rd_pat | (at_pat_q[entry_idx*PATTERN_W +: PATTERN_W]
                                       & {PATTERN_W{at_eff[entry_idx]}});
            vic_rd_vld  = vic_rd_vld | (at_vld_q[entry_idx] & f_at_vic_q[entry_idx]);
            vic_rd_pc   = vic_rd_pc | (at_pc_q[entry_idx*PC_HASH_W +: PC_HASH_W]
                                       & {PC_HASH_W{f_at_vic_q[entry_idx]}});
            vic_rd_line = vic_rd_line | (at_line_q[entry_idx*LIR_W +: LIR_W]
                                         & {LIR_W{f_at_vic_q[entry_idx]}});
            vic_rd_pat  = vic_rd_pat | (at_pat_q[entry_idx*PATTERN_W +: PATTERN_W]
                                        & {PATTERN_W{f_at_vic_q[entry_idx]}});
        end // for entry_idx
    end // always s2_read_proc

    assign ft_line_oh    = one_hot_line(ft_rd_line);
    assign do_at_set     = f_vld_q & ~f_evt_q & any_at;
    assign do_promote    = f_vld_q & ~f_evt_q & ~any_at & any_ft
                           & (f_line_q != ft_rd_line);
    assign do_trigger    = f_vld_q & ~f_evt_q & ~any_at & ~any_ft;
    assign do_end_gen    = f_vld_q & f_evt_q & any_at
                           & (|(at_rd_pat & f_line_oh_q));
    assign do_ft_discard = f_vld_q & f_evt_q & ~any_at & any_ft;

    // PHT write: an ended generation, or the AT slot a promote replaces
    assign pw_req = do_end_gen | (do_promote & vic_rd_vld);
    always @* begin : s2_pht_write_proc
        if (do_end_gen) begin
            pw_pc   = at_rd_pc;
            pw_line = at_rd_line;
            pw_pat  = at_rd_pat;
        end
        else begin
            pw_pc   = vic_rd_pc;
            pw_line = vic_rd_line;
            pw_pat  = vic_rd_pat;
        end // else (replacement)
    end // always s2_pht_write_proc
    assign pw_key = {pw_pc, pw_line};

    // PHT lookup for a trigger: tag check over the 4 ways read in S1
    always @* begin : s2_pht_lookup_proc
        integer way_idx;   // walks PHT ways
        pht_hit_way = {PHT_WAYS{1'b0}};
        pht_hit_pat = {PATTERN_W{1'b0}};
        for (way_idx = 0; way_idx < PHT_WAYS; way_idx = way_idx + 1) begin
            pht_hit_way[way_idx] = f_pht_vld_q[way_idx]
                & (f_pht_tag_q[way_idx*PHT_TAG_W +: PHT_TAG_W] == f_ptag_q);
            pht_hit_pat = pht_hit_pat
                | (f_pht_pat_q[way_idx*PATTERN_W +: PATTERN_W]
                   & {PATTERN_W{pht_hit_way[way_idx]}});
        end // for way_idx
    end // always s2_pht_lookup_proc
    assign pht_pat_rest = pht_hit_pat & ~f_line_oh_q;  // trigger line out
    assign load_valid   = do_trigger & (|pht_hit_way) & (|pht_pat_rest);
    assign load_region  = f_region_q;
    assign load_mask    = pht_pat_rest;
    assign load_line    = f_line_q;

    // ======================================================================
    // clocked table updates
    // ======================================================================
    always @(posedge clk) begin : table_proc
        if (rst) begin
            ft_vld_q     <= {FT_ENTRIES{1'b0}};
            at_vld_q     <= {AT_ENTRIES{1'b0}};
            ft_ring_q    <= {FT_IDX_W{1'b0}};
            at_ring_q    <= {AT_IDX_W{1'b0}};
            p_ft_alloc_q <= {FT_ENTRIES{1'b0}};
            p_at_alloc_q <= {AT_ENTRIES{1'b0}};
            w_vld_q      <= 1'b0;
        end
        else begin
            // ---- ring pointers move in S1 (so S1 of the next event
            //      already sees the new value): no free slot used ----
            if (e_vld_q && !e_evt_q && !ft_seen) begin
                ft_ring_q <= ft_ring_q + 1'b1;
            end
            if (e_vld_q && !e_evt_q && !at_seen) begin
                at_ring_q <= at_ring_q + 1'b1;
            end

            // ---- what this event allocated (for the next event) ----
            p_ft_alloc_q <= f_ft_vic_q & {FT_ENTRIES{do_trigger}};
            p_at_alloc_q <= f_at_vic_q & {AT_ENTRIES{do_promote}};

            // ---- FT writes ----
            for (entry_idx = 0; entry_idx < FT_ENTRIES; entry_idx = entry_idx + 1) begin
                if ((do_promote || do_ft_discard) && ft_eff[entry_idx]) begin
                    ft_vld_q[entry_idx] <= 1'b0;
                end
                if (do_trigger && f_ft_vic_q[entry_idx]) begin
                    ft_vld_q[entry_idx] <= 1'b1;
                    ft_tag_q[entry_idx*REGION_TAG_W +: REGION_TAG_W] <= f_tag_q;
                    ft_pc_q[entry_idx*PC_HASH_W +: PC_HASH_W]        <= f_pc_q;
                    ft_line_q[entry_idx*LIR_W +: LIR_W]              <= f_line_q;
                end
            end // for entry_idx

            // ---- AT writes ----
            for (entry_idx = 0; entry_idx < AT_ENTRIES; entry_idx = entry_idx + 1) begin
                if (do_at_set && at_eff[entry_idx]) begin
                    at_pat_q[entry_idx*PATTERN_W +: PATTERN_W] <=
                        at_pat_q[entry_idx*PATTERN_W +: PATTERN_W] | f_line_oh_q;
                end
                if (do_end_gen && at_eff[entry_idx]) begin
                    at_vld_q[entry_idx] <= 1'b0;
                end
                if (do_promote && f_at_vic_q[entry_idx]) begin
                    at_vld_q[entry_idx] <= 1'b1;
                    at_tag_q[entry_idx*REGION_TAG_W +: REGION_TAG_W] <= f_tag_q;
                    at_pc_q[entry_idx*PC_HASH_W +: PC_HASH_W]        <= ft_rd_pc;
                    at_line_q[entry_idx*LIR_W +: LIR_W]              <= ft_rd_line;
                    at_pat_q[entry_idx*PATTERN_W +: PATTERN_W]       <=
                        ft_line_oh | f_line_oh_q;
                end
            end // for entry_idx

            // ---- PHT write request register (W stage input) ----
            w_vld_q <= pw_req;
        end // else (run)
        w_set_q <= fold_set(pw_key);
        w_tag_q <= fold_ptag(pw_key);
        w_pat_q <= pw_pat;
    end // always table_proc

    // ======================================================================
    // W: write one pattern into the PHT (same tag, else free way, else
    // the ring way of the set)
    // ======================================================================
    always @* begin : w_way_proc
        integer way_idx;   // walks PHT ways
        w_base    = w_set_q * PHT_WAYS;
        w_ptr     = pht_ptr_q[w_set_q*WAY_W +: WAY_W];
        w_match   = {PHT_WAYS{1'b0}};
        w_free    = {PHT_WAYS{1'b0}};
        w_way_one = {PHT_WAYS{1'b0}};
        w_seen    = 1'b0;
        for (way_idx = 0; way_idx < PHT_WAYS; way_idx = way_idx + 1) begin
            w_match[way_idx] = pht_vld_q[w_base + way_idx]
                & (pht_tag_q[(w_base + way_idx)*PHT_TAG_W +: PHT_TAG_W] == w_tag_q);
            w_free[way_idx]  = ~pht_vld_q[w_base + way_idx];
        end // for way_idx
        // priority: same tag, then a free way, then the ring way
        for (way_idx = 0; way_idx < PHT_WAYS; way_idx = way_idx + 1) begin
            if (w_match[way_idx] && !w_seen) begin
                w_way_one[way_idx] = 1'b1;
                w_seen             = 1'b1;
            end
        end // for way_idx
        for (way_idx = 0; way_idx < PHT_WAYS; way_idx = way_idx + 1) begin
            if (w_free[way_idx] && !w_seen) begin
                w_way_one[way_idx] = 1'b1;
                w_seen             = 1'b1;
            end
        end // for way_idx
        w_adv = !w_seen;                 // ring way used: move the ring
        if (!w_seen) begin
            w_way_one[w_ptr] = 1'b1;
        end
    end // always w_way_proc

    always @(posedge clk) begin : pht_proc
        if (rst) begin
            pht_vld_q <= {PHT_TOTAL{1'b0}};
            pht_ptr_q <= {(PHT_SETS*WAY_W){1'b0}};
            pht_tag_q <= {(PHT_TOTAL*PHT_TAG_W){1'b0}};
            pht_pat_q <= {(PHT_TOTAL*PATTERN_W){1'b0}};
        end
        else if (w_vld_q) begin
            for (way_idx = 0; way_idx < PHT_WAYS; way_idx = way_idx + 1) begin
                if (w_way_one[way_idx]) begin
                    pht_vld_q[w_base + way_idx] <= 1'b1;
                    pht_tag_q[(w_base + way_idx)*PHT_TAG_W +: PHT_TAG_W] <= w_tag_q;
                    pht_pat_q[(w_base + way_idx)*PATTERN_W +: PATTERN_W] <= w_pat_q;
                end
            end // for way_idx
            if (w_adv) begin
                pht_ptr_q[w_set_q*WAY_W +: WAY_W] <= w_ptr + 1'b1;
            end
        end // else if (w_vld_q)
    end // always pht_proc

`ifdef L1D_DEBUG
    // ---- watch-dogs (V-9.3), simulation only ----
    integer check_a; // first entry of a pair
    integer check_b; // second entry of a pair
    always @(posedge clk) begin : debug_proc
        if (!rst) begin
            if (f_vld_q && ((ft_eff & (ft_eff - 1'b1)) != {FT_ENTRIES{1'b0}})) begin
                $display("L1D_PF_SMS ERROR: two FT entries used at %0t", $time);
            end
            if (f_vld_q && ((at_eff & (at_eff - 1'b1)) != {AT_ENTRIES{1'b0}})) begin
                $display("L1D_PF_SMS ERROR: two AT entries used at %0t", $time);
            end
            if (f_vld_q && any_ft && any_at) begin
                $display("L1D_PF_SMS ERROR: region in FT and AT at %0t", $time);
            end
            if (do_promote && ((f_at_vic_q & (f_at_vic_q - 1'b1)) != {AT_ENTRIES{1'b0}}
                               || f_at_vic_q == {AT_ENTRIES{1'b0}})) begin
                $display("L1D_PF_SMS ERROR: AT victim not one-hot at %0t", $time);
            end
            for (check_a = 0; check_a < AT_ENTRIES; check_a = check_a + 1) begin
                for (check_b = check_a + 1; check_b < AT_ENTRIES; check_b = check_b + 1) begin
                    if (at_vld_q[check_a] && at_vld_q[check_b]
                        && at_tag_q[check_a*REGION_TAG_W +: REGION_TAG_W]
                           == at_tag_q[check_b*REGION_TAG_W +: REGION_TAG_W]) begin
                        $display("L1D_PF_SMS ERROR: duplicate AT tag at %0t", $time);
                    end
                end // for check_b
            end // for check_a
        end // if (!rst)
    end // always debug_proc
`endif

endmodule

`default_nettype wire
`endif
