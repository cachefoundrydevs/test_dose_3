// ============================================================================
// l1d_vec_store_buf.v  -  the line data of the vector stores waiting in ONE bank (cache_v5 C-20, VPU port)
// ============================================================================
// IN SIMPLE WORDS
//   A vector store brings a whole line (512 bits) of new data. Making every request packet 512 bits wider would make
//   every seat, queue place and pipeline register of every port 512 bits wider, so the packet carries only a small place
//   number and the 512 bits wait here, in one of VEC_SLOTS (2) places of the bank. Think of a cloakroom: the request
//   keeps the ticket (the place number in its data field), the coat (the line) hangs here.
//     - at the bank door (FIFO stage) a vector store takes the lowest free place (write_slot) and its line is written
//       there; if no place is free, the vector store waits at the door (slot_available = 0, l1d_req_tracker and l1d_port_front)
//     - each time the store passes S1 its line is read from its place (read_slot -> read_line_data) and registered as
//       the store data of S2 (all 64 byte enables on)
//     - when the store has written its line in S2 (answered), its place is freed (release_*)
//   A store that must wait (miss, upgrade, retry) keeps its place, so its data is there for the next pass.
//
//      write_valid / write_line_data --> slot_line[write_slot] --> read_line_data (place read_slot, S1)
//      slot_used (flops): set at the write, cleared at release_valid
//
// STAGE / CYCLE
//   Written at the bank door (stage G cycle of the request), read in S1, freed in S2. The free place (write_slot,
//   slot_available) comes from flops only; read_line_data is an AND-OR read of the places (flops) with read_slot.
//
// INSTANTIATED BY
//   l1d_bank, instance u_vec_buf (one per bank).
//
// PARAMETERS
//   `L1D_PARAMS   all cache parameters (l1d_defines.vh); used here: LINE_BYTES (64) and, from l1d_fields.vh,
//                 LINE_W (512), VEC_SLOTS (2, `L1D_VEC_SLOTS), VEC_SLOT_W (1 bit)
//   BANK_ID       default 0: number of this bank (messages only)
//
// PORTS (name, direction, width at the defaults, meaning)
//   clk, rst          in   1     clock; synchronous reset, active high (all places free)
//   write_valid       in   1     a vector store is taken by the bank this cycle: keep its line in place write_slot
//   write_line_data   in   512   its line
//   slot_available    out  1     a place is free (a vector store may enter the bank)
//   write_slot        out  1     the lowest free place (the number the request carries)
//   read_slot         in   1     the place of the vector store in S1
//   read_line_data    out  512   its line
//   release_valid     in   1     the vector store in S2 has written its line: free its place
//   release_slot      in   1     that place
//
// WORKED EXAMPLE (VEC_SLOTS = 2)
//   cycle 1: store A enters -> slot_used = 01, A keeps place 0.   cycle 2: store B enters -> slot_used = 11, B: place 1.
//   cycle 3: store C arrives -> slot_available = 0: C waits at the bank door.
//   cycle 5: A writes its line in S2 -> release place 0 -> slot_used = 10 at the edge; cycle 6: C enters, takes place 0.
//
// VERILOG WORDS USED IN THIS FILE
//   reg [W-1:0] m [0:N-1]   N registers of W bits (a small memory of flops)
//   ~&x                     NOT of the AND of all bits of x: 1 when at least one bit is 0
// ============================================================================
`ifndef L1D_VEC_STORE_BUF_V
`define L1D_VEC_STORE_BUF_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_vec_store_buf #(
    `L1D_PARAMS,                                       // all cache parameters (l1d_defines.vh)
    parameter BANK_ID = 0                              // default 0: number of this bank (messages only)
) (
    clk, rst,
    write_valid, write_line_data, slot_available, write_slot,
    read_slot, read_line_data,
    release_valid, release_slot
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    input  wire                   clk;                   // clock: all registers change on its rising edge
    input  wire                   rst;                   // synchronous reset, active high: all places free
    input  wire                   write_valid;           // a vector store is taken by the bank: keep its line
    input  wire [LINE_W-1:0]      write_line_data;       // its line, LINE_W = 512 bits
    output wire                   slot_available;        // a place is free
    output reg  [VEC_SLOT_W-1:0]  write_slot;            // the lowest free place, VEC_SLOT_W = 1 bit
    input  wire [VEC_SLOT_W-1:0]  read_slot;             // the place of the vector store in S1, 1 bit
    output reg  [LINE_W-1:0]      read_line_data;        // its line, 512 bits
    input  wire                   release_valid;         // the vector store in S2 wrote its line: free its place
    input  wire [VEC_SLOT_W-1:0]  release_slot;          // that place, 1 bit

    reg [VEC_SLOTS-1:0] slot_used;                       // one bit per place: it holds a line, VEC_SLOTS = 2 bits
    reg [LINE_W-1:0]    slot_line [0:VEC_SLOTS-1];       // the lines, VEC_SLOTS places of 512 bits

    // ---- lowest free place (from flops only) ----
    integer free_scan;                                   // loop variable: place, scanned from the top
    always @* begin
        write_slot = {VEC_SLOT_W{1'b0}};
        for (free_scan = VEC_SLOTS - 1; free_scan >= 0; free_scan = free_scan - 1)
            if (!slot_used[free_scan]) write_slot = free_scan[VEC_SLOT_W-1:0];
    end
    assign slot_available = ~&slot_used;

    // ---- place bookkeeping: set at the write, cleared at the release (never the same place in one cycle) ----
    always @(posedge clk) begin
        if (rst) begin
            slot_used <= {VEC_SLOTS{1'b0}};
        end else begin
            if (release_valid) slot_used[release_slot] <= 1'b0;
            if (write_valid)   slot_used[write_slot]   <= 1'b1;
        end
    end

    // ---- the line data (no reset: a place is read only after its write) ----
    always @(posedge clk) begin
        if (write_valid) slot_line[write_slot] <= write_line_data;
    end

    // ---- read of the place of the store in S1: AND-OR over the places ----
    integer read_scan;                                   // loop variable: place
    always @* begin
        read_line_data = {LINE_W{1'b0}};
        for (read_scan = 0; read_scan < VEC_SLOTS; read_scan = read_scan + 1)
            read_line_data = read_line_data | ({LINE_W{read_slot == read_scan[VEC_SLOT_W-1:0]}} & slot_line[read_scan]);
    end

    // ---- simulation checks: the bank door rule and the release of a place that is in use ----
    always @(posedge clk) begin
        if (!rst) begin
            if (write_valid && !slot_available)
                $display("[%0t] L1D VECBUF ERROR bank %0d: a vector store entered while all %0d places were used", $time, BANK_ID, VEC_SLOTS);
            if (release_valid && !slot_used[release_slot])
                $display("[%0t] L1D VECBUF ERROR bank %0d: release of free place %0d", $time, BANK_ID, release_slot);
        end
    end

endmodule

`default_nettype wire
`endif // L1D_VEC_STORE_BUF_V
