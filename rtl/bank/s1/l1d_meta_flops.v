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
//   Read (Stage 1): give read_set; read_way_states_flat (all 4 states of the set), read_plru_bits and read_pf_bits come out
//   in the same cycle (asynchronous read).
//   Write (Stage 1): one state write, one PLRU write and one pf_bit write per cycle. They take effect at the NEXT clock edge,
//   so the next operation in Stage 1 already sees the new values. (A read of the same set in the same cycle still sees the
//   OLD values.) The new state comes from l1d_mesi_next.
//
// WHAT KIND OF STATE MACHINE IS IT?
//   It is the state register of 256 independent per-line MESI machines and 64 per-set PLRU machines (Moore style):
//   this file only stores and reads the state; the next-state logic is l1d_mesi_next and l1d_plru.
//
// PORTS
//   read_set                     set whose state, PLRU and pf bits are read
//   read_way_states_flat         state (2 bits) of every way of that set
//   read_plru_bits               PLRU bits of that set
//   read_pf_bits                 pf_bit of every way of that set (cache_v6)
//   state_write_enable/_set/_way/_value   write one way's state (ST_I / ST_S / ST_E / ST_M)
//   plru_write_enable/_set/_value         write the PLRU bits of one set
//   pf_write_enable/_set/_way/_value      write one way's pf_bit (cache_v6): 1 at a prefetch install, 0 at the first demand hit,
//                                         a demand install, an invalidation
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

module l1d_meta_flops #(
    parameter SETS    = 64,                               // default 64: sets per bank
    parameter WAYS    = 4,                                // default 4: ways per set
    parameter INDEX_W = $clog2(SETS),                     // INDEX_W = 6 bits
    parameter WAY_W   = $clog2(WAYS),                     // WAY_W = 2 bits
    parameter NB      = WAYS - 1                          // NB = 3 PLRU bits per set
) (
    input  wire               clk,                        // clock: all registers change on its rising edge
    input  wire               rst,                        // synchronous reset, active high (every line invalid, no pf bit)
    // read (Stage 1)
    input  wire [INDEX_W-1:0] read_set,                   // set whose metadata is read, 6 bits
    output wire [2*WAYS-1:0]  read_way_states_flat,       // state (2 bits) of every way of that set, 8 bits
    output wire [NB-1:0]      read_plru_bits,             // PLRU bits of that set, 3 bits
    output wire [WAYS-1:0]    read_pf_bits,               // pf_bit of every way of that set, 4 bits
    // state write
    input  wire               state_write_enable,         // write one way's state
    input  wire [INDEX_W-1:0] state_write_set,            // set of the state write, 6 bits
    input  wire [WAY_W-1:0]   state_write_way,            // way of the state write, 2 bits
    input  wire [1:0]         state_write_value,          // new state (ST_I / ST_S / ST_E / ST_M)
    // PLRU write
    input  wire               plru_write_enable,          // write the PLRU bits of one set
    input  wire [INDEX_W-1:0] plru_write_set,             // set of the PLRU write, 6 bits
    input  wire [NB-1:0]      plru_write_value,           // new PLRU bits, 3 bits
    // pf_bit write (cache_v6)
    input  wire               pf_write_enable,            // write one way's pf_bit
    input  wire [INDEX_W-1:0] pf_write_set,               // set of the pf_bit write, 6 bits
    input  wire [WAY_W-1:0]   pf_write_way,               // way of the pf_bit write, 2 bits
    input  wire               pf_write_value              // new pf_bit
);

    // the storage: one word of WAYS 2-bit states per set, WAYS-1 PLRU bits per set, WAYS pf bits per set
    reg [2*WAYS-1:0] states [0:SETS-1];                   // 64 x 8 bits
    reg [NB-1:0]     plru   [0:SETS-1];                   // 64 x 3 bits
    reg [WAYS-1:0]   pf_bit [0:SETS-1];                   // 64 x 4 bits
    integer s;                                            // loop variable: set (reset)

    // ---- clocked part: reset makes every line invalid (ST_I = 0), clears the PLRU and pf bits; otherwise apply the writes ----
    always @(posedge clk) begin : meta_proc
        if (rst) begin
            for (s = 0; s < SETS; s = s + 1) begin
                states[s] <= {2*WAYS{1'b0}};     // all ways invalid (ST_I = 0)
                plru[s]   <= {NB{1'b0}};
                pf_bit[s] <= {WAYS{1'b0}};
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

    // ---- asynchronous read of the chosen set ----
    assign read_way_states_flat = states[read_set];
    assign read_plru_bits       = plru[read_set];
    assign read_pf_bits         = pf_bit[read_set];

// ---- simulation-only trace: every line-state change (0 = I, 1 = S, 2 = E, 3 = M) and pf_bit change ----
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
