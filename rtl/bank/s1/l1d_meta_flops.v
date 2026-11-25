// ============================================================================
// l1d_meta_flops.v  -  line states, PLRU bits and the "prefetched, not yet used" bit of one bank, kept in flip-flops
// ============================================================================
// IN SIMPLE WORDS
//   Besides its 512 data bits and its tag, every cache line has a few bits of "metadata":
//     state per {set, way}  2 bits: I invalid, S shared, E exclusive (clean), M modified (dirty)
//     PLRU per set          WAYS-1 = 3 bits (see l1d_plru.v)
//     pf_bit per {set, way} 1 bit (cache_v6): the line was brought in by a PREFETCH and no demand access has used it yet.
//                           The first demand hit clears it and tells the prefetchers (train_pf_hit: "your prefetch was useful").
//   They are kept in FLIP-FLOPS, not in a RAM, because:
//     - they must be READ combinationally in Stage 1 (no extra read cycle),
//     - reset has to make EVERY line invalid at once (a RAM cannot be cleared in one cycle).
//   That is 64 sets x (4 ways x 2 bits + 3 bits + 4 bits) = 960 flip-flops per bank.
//
// HOW IT IS USED
//   Read (Stage 1): give read_set; read_way_states_flat (all 4 states of the set), read_way_valid, read_plru_bits and
//   read_pf_bits come out in the same cycle (asynchronous read).
//   Write (Stage 1): one state write, one PLRU write and one pf_bit write per cycle. They take effect at the NEXT clock edge,
//   so the next operation in Stage 1 already sees the new values. (A read of the same set in the same cycle still sees the
//   OLD values.) The new state comes from l1d_mesi_next.
//
// TWO WAYS TO READ AND TO WRITE (tm_opt s1 iter 3/4, 2026-11-19; the defaults are the v6 behaviour)
//   READ_ONEHOT = 0  read by the set number: a 64:1 multiplexer per bit, every select bit drives ~1000 multiplexer inputs.
//   READ_ONEHOT = 1  read_set is decoded ONCE to a 64-bit one-hot vector (read_hot); every bit read is one AND-OR of the
//                    64 sets (AND with read_hot, then an OR tree), the 64 sets in parallel. read_way_valid is the OR of the
//                    two state bits of every way of EVERY set (before the read), so "valid" is ready with the states.
//   WRITE_ONEHOT = 0 one state write by {set, way number, value}, one pf write by {set, way number}: the way number is
//                    decoded here again.
//   WRITE_ONEHOT = 1 the writer gives one enable and one value PER WAY (state_write_way_enable / _value,
//                    pf_write_way_enable), so a way's flops wait only for that way's own enable; the set is still
//                    state_write_set / pf_write_set. The v6 write ports (state_write_enable / _way / _value,
//                    pf_write_enable / pf_write_way) are not used for the write then; the simulation trace still reads them,
//                    so the writer drives them with the same write.
//   PLRU_ONEHOT = 1  (tm_opt s1 iter 14) the PLRU write also comes per way: plru_write_way_enable[w] writes
//                    plru_write_way_value[w] (the PLRU bits after an access of way w) to the set plru_write_set. The
//                    writer opens at most one way; plru_write_enable / plru_write_value are then not used for the write.
//   (tm_opt s1 iter 16) read_inv_first / read_any_invalid: the lowest invalid way and "some way invalid" of the read set; with
//                    READ_ONEHOT = 1 they are made for every set before the read and read out like the states.
//   WRITE_BEHIND = 1/2 (tm_opt s1 iter 23/24; needs WRITE_ONEHOT = 1 and PLRU_ONEHOT = 1) the writes of this cycle go into a
//                    small PENDING register only (set, per-way enables and values: ~50 flops), not into the 960 storage
//                    flops. In the next cycle the pending write is copied into the storage flops (states_held, plru_held,
//                    pf_held), from flops, so early. states / plru / pf_bit are then the held values WITH the pending write
//                    applied (combinational): the same values, cycle by cycle, as the storage of WRITE_BEHIND = 0, and the
//                    names the testbenches and the proof look at. 1: the read takes the held flops (one-hot AND-OR) and puts
//                    the pending write over the result (forwarding after the read); 2: the read takes states / plru / pf_bit.
//                    The late write enables then load ~50 flops instead of fanning out to 64 sets.
//   All modes store and read the same values for the same writes.
//
// WHAT KIND OF STATE MACHINE IS IT?
//   It is the state register of 256 independent per-line MESI machines and 64 per-set PLRU machines (Moore style):
//   this file only stores and reads the state; the next-state logic is l1d_mesi_next and l1d_plru.
//
// STAGE / PARENT
//   S1, instance u_meta of l1d_bank_s1.
//
// PARAMETERS
//   SETS 64, WAYS 4, INDEX_W 6, WAY_W 2, NB 3 (PLRU bits), READ_ONEHOT 0, WRITE_ONEHOT 0, PLRU_ONEHOT 0, WRITE_BEHIND 0
//
// PORTS
//   read_set                     in   INDEX_W   set whose state, PLRU and pf bits are read
//   read_way_states_flat         out  2*WAYS    state (2 bits) of every way of that set
//   read_way_valid               out  WAYS      every way of that set holds a line (state != ST_I)
//   read_plru_bits               out  NB        PLRU bits of that set
//   read_pf_bits                 out  WAYS      pf_bit of every way of that set (cache_v6)
//   read_inv_first               out  WAYS      one-hot: the lowest invalid way of that set (all 0 when every way is valid)
//   read_any_invalid             out  1         some way of that set is invalid
//   state_write_enable/_set/_way/_value   in    write one way's state (ST_I / ST_S / ST_E / ST_M) (WRITE_ONEHOT = 0)
//   state_write_way_enable       in   WAYS      WRITE_ONEHOT = 1: write the state of way w of state_write_set
//   state_write_way_value        in   2*WAYS    WRITE_ONEHOT = 1: new state of way w at [2*w +: 2]
//   plru_write_enable/_set/_value         in    write the PLRU bits of one set (value: PLRU_ONEHOT = 0)
//   plru_write_way_enable        in   WAYS      PLRU_ONEHOT = 1: write plru_write_way_value of way w to plru_write_set
//   plru_write_way_value         in   WAYS*NB   PLRU_ONEHOT = 1: PLRU bits after an access of way w at [w*NB +: NB]
//   pf_write_enable/_set/_way/_value      in    write one way's pf_bit (cache_v6): 1 at a prefetch install, 0 at the first
//                                               demand hit, a demand install, an invalidation (way number: WRITE_ONEHOT = 0)
//   pf_write_way_enable          in   WAYS      WRITE_ONEHOT = 1: write pf_write_value to the pf bit of way w of pf_write_set
//
// WORKED EXAMPLE (WRITE_ONEHOT = 1)
//   A load hits way 2 of set 5 and the line goes E -> M: state_write_set = 5, state_write_way_enable = 4'b0100,
//   state_write_way_value[5:4] = ST_M. At the clock edge only states[5][5:4] changes; the other 3 ways keep their value.
// ============================================================================
`ifndef L1D_META_FLOPS_V
`define L1D_META_FLOPS_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_meta_flops #(
    parameter SETS    = 64,                               // default 64: sets per bank
    parameter WAYS    = 4,                                // default 4: ways per set
    parameter INDEX_W = $clog2(SETS),                     // INDEX_W = 6 bits
    parameter WAY_W   = $clog2(WAYS),                     // WAY_W = 2 bits
    parameter NB      = WAYS - 1,                         // NB = 3 PLRU bits per set
    parameter READ_ONEHOT  = 0,                           // default 0: read by set number (v6); 1: one-hot set, AND-OR read
    parameter WRITE_ONEHOT = 0,                           // default 0: write by way number (v6); 1: one enable and value per way
    parameter PLRU_ONEHOT  = 0,                           // default 0: one PLRU value (v6); 1: PLRU enable and value per way
    parameter WRITE_BEHIND = 0                            // default 0: write the storage (v6); 1/2: pending register, see header
) (
    input  wire               clk,                        // clock: all registers change on its rising edge
    input  wire               rst,                        // synchronous reset, active high (every line invalid, no pf bit)
    // read (Stage 1)
    input  wire [INDEX_W-1:0] read_set,                   // set whose metadata is read, 6 bits
    output wire [2*WAYS-1:0]  read_way_states_flat,       // state (2 bits) of every way of that set, 8 bits
    output wire [WAYS-1:0]    read_way_valid,             // every way of that set holds a line (state != ST_I), 4 bits
    output wire [NB-1:0]      read_plru_bits,             // PLRU bits of that set, 3 bits
    output wire [WAYS-1:0]    read_pf_bits,               // pf_bit of every way of that set, 4 bits
    output wire [WAYS-1:0]    read_inv_first,             // one-hot: the lowest invalid way of that set (0: none), 4 bits
    output wire               read_any_invalid,           // some way of that set is invalid
    // state write
    input  wire               state_write_enable,         // write one way's state (WRITE_ONEHOT = 0)
    input  wire [INDEX_W-1:0] state_write_set,            // set of the state write, 6 bits
    input  wire [WAY_W-1:0]   state_write_way,            // way of the state write, 2 bits (WRITE_ONEHOT = 0)
    input  wire [1:0]         state_write_value,          // new state (ST_I / ST_S / ST_E / ST_M) (WRITE_ONEHOT = 0)
    input  wire [WAYS-1:0]    state_write_way_enable,     // WRITE_ONEHOT = 1: write the state of way w, 4 bits
    input  wire [2*WAYS-1:0]  state_write_way_value,      // WRITE_ONEHOT = 1: new state of way w at [2*w +: 2], 8 bits
    // PLRU write
    input  wire               plru_write_enable,          // write the PLRU bits of one set
    input  wire [INDEX_W-1:0] plru_write_set,             // set of the PLRU write, 6 bits
    input  wire [NB-1:0]      plru_write_value,           // new PLRU bits, 3 bits (PLRU_ONEHOT = 0)
    input  wire [WAYS-1:0]    plru_write_way_enable,      // PLRU_ONEHOT = 1: write the PLRU bits of an access of way w, 4 bits
    input  wire [WAYS*NB-1:0] plru_write_way_value,       // PLRU_ONEHOT = 1: PLRU bits after an access of way w, WAYS x NB = 12 bits
    // pf_bit write (cache_v6)
    input  wire               pf_write_enable,            // write one way's pf_bit (WRITE_ONEHOT = 0)
    input  wire [INDEX_W-1:0] pf_write_set,               // set of the pf_bit write, 6 bits
    input  wire [WAY_W-1:0]   pf_write_way,               // way of the pf_bit write, 2 bits (WRITE_ONEHOT = 0)
    input  wire               pf_write_value,             // new pf_bit
    input  wire [WAYS-1:0]    pf_write_way_enable         // WRITE_ONEHOT = 1: write pf_write_value to the pf bit of way w, 4 bits
);

    // the storage: one word of WAYS 2-bit states per set, WAYS-1 PLRU bits per set, WAYS pf bits per set
    // (WRITE_BEHIND = 0: these are the flops; WRITE_BEHIND != 0: the held flops with the pending write applied, combinational)
    reg [2*WAYS-1:0] states [0:SETS-1];                   // 64 x 8 bits
    reg [NB-1:0]     plru   [0:SETS-1];                   // 64 x 3 bits
    reg [WAYS-1:0]   pf_bit [0:SETS-1];                   // 64 x 4 bits
    // WRITE_BEHIND != 0: the storage flops (one write behind) and the pending write
    reg [2*WAYS-1:0]  states_held [0:SETS-1];             // held line states, 64 x 8 bits
    reg [NB-1:0]      plru_held   [0:SETS-1];             // held PLRU bits, 64 x 3 bits
    reg [WAYS-1:0]    pf_held     [0:SETS-1];             // held pf bits, 64 x 4 bits
    reg [INDEX_W-1:0] pend_state_set;                     // set of the pending state write, 6 bits
    reg [WAYS-1:0]    pend_state_en;                      // ways of the pending state write, 4 bits
    reg [2*WAYS-1:0]  pend_state_val;                     // new state of way w at [2*w +: 2], 8 bits
    reg [INDEX_W-1:0] pend_plru_set;                      // set of the pending PLRU write, 6 bits
    reg [WAYS-1:0]    pend_plru_en;                       // accessed way of the pending PLRU write (one-hot or 0), 4 bits
    reg [WAYS*NB-1:0] pend_plru_cand;                     // PLRU bits after an access of way w, WAYS x NB = 12 bits
    reg [INDEX_W-1:0] pend_pf_set;                        // set of the pending pf write, 6 bits
    reg [WAYS-1:0]    pend_pf_en;                         // ways of the pending pf write, 4 bits
    reg               pend_pf_val;                        // new pf bit
    reg [NB-1:0]      pend_plru_pick;                     // PLRU bits of the pending PLRU write (last enabled way, as PLRU_ONEHOT = 1)
    integer           es;                                 // loop variable: set (effective values)
    integer           ew;                                 // loop variable: way (effective values, PLRU pick)
    integer s;                                            // loop variable: set (reset, one-hot write)
    integer ww;                                           // loop variable: way (one-hot write)

    // ---- the set numbers decoded to one-hot vectors (used by READ_ONEHOT = 1 / WRITE_ONEHOT = 1; removed otherwise) ----
    wire [SETS-1:0] read_hot;                             // 1 at read_set, SETS = 64 bits
    wire [SETS-1:0] state_write_hot;                      // 1 at state_write_set, SETS = 64 bits
    wire [SETS-1:0] plru_write_hot;                       // 1 at plru_write_set, SETS = 64 bits
    wire [SETS-1:0] pf_write_hot;                         // 1 at pf_write_set, SETS = 64 bits
    wire [SETS-1:0] pend_state_hot;                       // 1 at pend_state_set (WRITE_BEHIND), SETS = 64 bits
    wire [SETS-1:0] pend_plru_hot;                        // 1 at pend_plru_set (WRITE_BEHIND), SETS = 64 bits
    wire [SETS-1:0] pend_pf_hot;                          // 1 at pend_pf_set (WRITE_BEHIND), SETS = 64 bits
    genvar hs;                                            // loop variable: set
    generate
        for (hs = 0; hs < SETS; hs = hs + 1) begin : g_set_hot
            assign read_hot[hs]        = (read_set == hs);
            assign state_write_hot[hs] = (state_write_set == hs);
            assign plru_write_hot[hs]  = (plru_write_set == hs);
            assign pf_write_hot[hs]    = (pf_write_set == hs);
            assign pend_state_hot[hs]  = (pend_state_set == hs);
            assign pend_plru_hot[hs]   = (pend_plru_set == hs);
            assign pend_pf_hot[hs]     = (pend_pf_set == hs);
        end
    endgenerate

    // ---- the lowest invalid way of one set, one-hot (tm_opt s1 iter 16); all 0 when every way is valid ----
    function [WAYS-1:0] inv_first_of;
        input [WAYS-1:0] valid_bits;                      // valid bit of every way of the set, 4 bits
        integer fw;                                       // loop variable: way
        reg     found;                                    // an invalid way was found below fw
        begin
            inv_first_of = {WAYS{1'b0}};
            found        = 1'b0;
            for (fw = 0; fw < WAYS; fw = fw + 1) begin
                inv_first_of[fw] = !valid_bits[fw] && !found;
                found            = found || !valid_bits[fw];
            end
        end
    endfunction

    generate
    if (WRITE_BEHIND == 0) begin : g_write_direct
    // ---- clocked part: reset makes every line invalid (ST_I = 0), clears the PLRU and pf bits; otherwise apply the writes ----
    always @(posedge clk) begin : meta_proc
        if (rst) begin
            for (s = 0; s < SETS; s = s + 1) begin
                states[s] <= {2*WAYS{1'b0}};     // all ways invalid (ST_I = 0)
                plru[s]   <= {NB{1'b0}};
                pf_bit[s] <= {WAYS{1'b0}};
            end
        end else if (WRITE_ONEHOT != 0) begin
            // every {set, way} has its own write condition: the set's one-hot bit AND the way's own enable
            for (s = 0; s < SETS; s = s + 1) begin
                for (ww = 0; ww < WAYS; ww = ww + 1) begin
                    if (state_write_hot[s] && state_write_way_enable[ww])
                        states[s][2*ww +: 2] <= state_write_way_value[2*ww +: 2];
                    if (pf_write_hot[s] && pf_write_way_enable[ww])
                        pf_bit[s][ww] <= pf_write_value;
                end
                if (PLRU_ONEHOT != 0) begin
                    for (ww = 0; ww < WAYS; ww = ww + 1)
                        if (plru_write_hot[s] && plru_write_way_enable[ww])
                            plru[s] <= plru_write_way_value[ww*NB +: NB];
                end else if (plru_write_enable && plru_write_hot[s])
                    plru[s] <= plru_write_value;
            end
        end else begin
            if (state_write_enable)
                states[state_write_set][2*state_write_way +: 2] <= state_write_value;
            if (plru_write_enable)
                plru[plru_write_set] <= plru_write_value;
            if (pf_write_enable)
                pf_bit[pf_write_set][pf_write_way] <= pf_write_value;
        end
    end // always meta_proc
    end else begin : g_write_behind
    // ---- write-behind: the effective values = held flops with the pending write applied (combinational) ----
    always @* begin : pend_plru_proc
        pend_plru_pick = {NB{1'b0}};
        for (ew = 0; ew < WAYS; ew = ew + 1)
            if (pend_plru_en[ew])
                pend_plru_pick = pend_plru_cand[ew*NB +: NB];
    end
    always @* begin : effective_proc
        for (es = 0; es < SETS; es = es + 1) begin
            states[es] = states_held[es];
            plru[es]   = plru_held[es];
            pf_bit[es] = pf_held[es];
            for (ew = 0; ew < WAYS; ew = ew + 1) begin
                if (pend_state_hot[es] && pend_state_en[ew])
                    states[es][2*ew +: 2] = pend_state_val[2*ew +: 2];
                if (pend_pf_hot[es] && pend_pf_en[ew])
                    pf_bit[es][ew] = pend_pf_val;
            end
            if (pend_plru_hot[es] && (|pend_plru_en))
                plru[es] = pend_plru_pick;
        end
    end
    // ---- write-behind, clocked: the held flops take the effective values (the pending write goes in), the pending register
    //      takes this cycle's write; reset clears the held flops and the pending enables ----
    always @(posedge clk) begin : meta_behind_proc
        if (rst) begin
            for (s = 0; s < SETS; s = s + 1) begin
                states_held[s] <= {2*WAYS{1'b0}};
                plru_held[s]   <= {NB{1'b0}};
                pf_held[s]     <= {WAYS{1'b0}};
            end
            pend_state_en <= {WAYS{1'b0}};
            pend_plru_en  <= {WAYS{1'b0}};
            pend_pf_en    <= {WAYS{1'b0}};
        end else begin
            for (s = 0; s < SETS; s = s + 1) begin
                states_held[s] <= states[s];
                plru_held[s]   <= plru[s];
                pf_held[s]     <= pf_bit[s];
            end
            pend_state_set <= state_write_set;
            pend_state_en  <= state_write_way_enable;
            pend_state_val <= state_write_way_value;
            pend_plru_set  <= plru_write_set;
            pend_plru_en   <= plru_write_way_enable;
            pend_plru_cand <= plru_write_way_value;
            pend_pf_set    <= pf_write_set;
            pend_pf_en     <= pf_write_way_enable;
            pend_pf_val    <= pf_write_value;
        end
    end // always meta_behind_proc
    end
    endgenerate

    // ---- asynchronous read of the chosen set ----
    generate
        if (WRITE_BEHIND == 1) begin : g_read_behind
            // the held flops read by the one-hot AND-OR, then the pending write put over the result (forwarding after the read)
            reg [2*WAYS-1:0] held_states_or;              // held states of the read set, 8 bits
            reg [WAYS-1:0]   held_valid_or;               // held valid bits of the read set, 4 bits
            reg [NB-1:0]     held_plru_or;                // held PLRU bits of the read set, 3 bits
            reg [WAYS-1:0]   held_pf_or;                  // held pf bits of the read set, 4 bits
            integer hr;                                   // loop variable: set
            integer hv;                                   // loop variable: way
            always @* begin
                held_states_or = {2*WAYS{1'b0}};
                held_valid_or  = {WAYS{1'b0}};
                held_plru_or   = {NB{1'b0}};
                held_pf_or     = {WAYS{1'b0}};
                for (hr = 0; hr < SETS; hr = hr + 1) begin
                    held_states_or = held_states_or | ({2*WAYS{read_hot[hr]}} & states_held[hr]);
                    held_plru_or   = held_plru_or   | ({NB{read_hot[hr]}}     & plru_held[hr]);
                    held_pf_or     = held_pf_or     | ({WAYS{read_hot[hr]}}   & pf_held[hr]);
                    for (hv = 0; hv < WAYS; hv = hv + 1)
                        held_valid_or[hv] = held_valid_or[hv] | (read_hot[hr] & (|states_held[hr][2*hv +: 2]));   // != ST_I
                end
            end
            wire pend_state_here;                         // the pending state write is for the read set
            wire pend_plru_here;                          // a pending PLRU write is for the read set
            wire pend_pf_here;                            // the pending pf write is for the read set
            assign pend_state_here = (pend_state_set == read_set);
            assign pend_plru_here  = (pend_plru_set == read_set) && (|pend_plru_en);
            assign pend_pf_here    = (pend_pf_set == read_set);
            genvar fv;                                    // loop variable: way
            for (fv = 0; fv < WAYS; fv = fv + 1) begin : g_forward
                assign read_way_states_flat[2*fv +: 2] = (pend_state_here && pend_state_en[fv]) ? pend_state_val[2*fv +: 2] :
                                                                                                  held_states_or[2*fv +: 2];
                assign read_way_valid[fv]  = (pend_state_here && pend_state_en[fv]) ? (|pend_state_val[2*fv +: 2]) :
                                                                                      held_valid_or[fv];
                assign read_pf_bits[fv]    = (pend_pf_here && pend_pf_en[fv]) ? pend_pf_val : held_pf_or[fv];
            end
            assign read_plru_bits   = pend_plru_here ? pend_plru_pick : held_plru_or;
            assign read_inv_first   = inv_first_of(read_way_valid);
            assign read_any_invalid = !(&read_way_valid);
        end else if (READ_ONEHOT != 0) begin : g_read_onehot
            // one AND-OR per bit: (read_hot[s] AND bit of set s), ORed over the 64 sets; the 64 sets work in parallel
            reg [2*WAYS-1:0] read_states_or;              // states of the read set, 8 bits
            reg [WAYS-1:0]   read_valid_or;               // valid bits of the read set, 4 bits
            reg [NB-1:0]     read_plru_or;                // PLRU bits of the read set, 3 bits
            reg [WAYS-1:0]   read_pf_or;                  // pf bits of the read set, 4 bits
            reg [WAYS-1:0]   read_inv_first_or;           // lowest invalid way of the read set, 4 bits
            reg              read_any_invalid_or;         // some way of the read set is invalid
            reg [WAYS-1:0]   set_valid_bits;              // valid bits of set rs (inside the loop), 4 bits
            integer rs;                                   // loop variable: set
            integer rw;                                   // loop variable: way
            always @* begin
                read_states_or = {2*WAYS{1'b0}};
                read_valid_or  = {WAYS{1'b0}};
                read_plru_or   = {NB{1'b0}};
                read_pf_or     = {WAYS{1'b0}};
                read_inv_first_or   = {WAYS{1'b0}};
                read_any_invalid_or = 1'b0;
                set_valid_bits      = {WAYS{1'b0}};
                for (rs = 0; rs < SETS; rs = rs + 1) begin
                    for (rw = 0; rw < WAYS; rw = rw + 1)
                        set_valid_bits[rw] = |states[rs][2*rw +: 2];                                  // != ST_I
                    read_inv_first_or   = read_inv_first_or   | ({WAYS{read_hot[rs]}} & inv_first_of(set_valid_bits));
                    read_any_invalid_or = read_any_invalid_or | (read_hot[rs] & !(&set_valid_bits));
                    read_states_or = read_states_or | ({2*WAYS{read_hot[rs]}} & states[rs]);
                    read_plru_or   = read_plru_or   | ({NB{read_hot[rs]}}     & plru[rs]);
                    read_pf_or     = read_pf_or     | ({WAYS{read_hot[rs]}}   & pf_bit[rs]);
                    for (rw = 0; rw < WAYS; rw = rw + 1)
                        read_valid_or[rw] = read_valid_or[rw] | (read_hot[rs] & (|states[rs][2*rw +: 2]));   // != ST_I
                end
            end
            assign read_way_states_flat = read_states_or;
            assign read_way_valid       = read_valid_or;
            assign read_plru_bits       = read_plru_or;
            assign read_pf_bits         = read_pf_or;
            assign read_inv_first       = read_inv_first_or;
            assign read_any_invalid     = read_any_invalid_or;
        end else begin : g_read_binary
            genvar vw;                                    // loop variable: way
            assign read_way_states_flat = states[read_set];
            assign read_plru_bits       = plru[read_set];
            assign read_pf_bits         = pf_bit[read_set];
            for (vw = 0; vw < WAYS; vw = vw + 1) begin : g_valid
                assign read_way_valid[vw] = |read_way_states_flat[2*vw +: 2];                        // != ST_I
            end
            assign read_inv_first   = inv_first_of(read_way_valid);
            assign read_any_invalid = !(&read_way_valid);
        end
    endgenerate

// ---- simulation-only trace: every line-state change (0 = I, 1 = S, 2 = E, 3 = M) and pf_bit change (v6 write ports) ----
`ifdef L1D_DEBUG
    always @(posedge clk) begin
        if (!rst && state_write_enable && states[state_write_set][2*state_write_way +: 2] != state_write_value)
            $display("[%0t] L1D META %m set=%0d way=%0d state %0d -> %0d",
                     $time, state_write_set, state_write_way, states[state_write_set][2*state_write_way +: 2], state_write_value);
        if (!rst && pf_write_enable && pf_bit[pf_write_set][pf_write_way] != pf_write_value)
            $display("[%0t] L1D META %m set=%0d way=%0d pf_bit -> %0d", $time, pf_write_set, pf_write_way, pf_write_value);
    end
`endif

endmodule

`default_nettype wire
`endif // L1D_META_FLOPS_V
