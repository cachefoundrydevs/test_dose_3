// ============================================================================
// l1d_sram_tdp.v  -  TRUE DUAL-PORT SRAM model (two independent ports, A and B)
// ============================================================================
// cache_v2_bank_pipe (2026-11-03, user decision: "keep the memory as true dual port so we can solve simultaneous read
// and write from two stages"). Both ports can read or write in every cycle. The bank uses port A for the reads
// launched at the grant (Stage G) and port B for the writes of Stage 2 (store bytes, refill line, refill tag), so a
// read and a write happen in the same cycle.
//
// Timing of each port (as l1d_sram_sp): address / enable in cycle X, read data from the port's own output register in
// X+1. A port that writes also updates its own read register with the OLD contents (read-first).
//
// SAME ADDRESS ON BOTH PORTS IN ONE CYCLE (A reads, B writes): A returns the OLD data (read-first; a real macro may
// return undefined data). The bank never uses that data: a CPU request issued in that cycle to the line being written
// is cancelled in Stage R and retried (l1d_bank.v, wl_hit), and line readers (snoop, refill, flush/clean) are not
// granted while a store write is pending. Two writes to one address in one cycle never happen (only port B writes).
//
// PORTS
//   a_en, a_we[NB_COL], a_addr, a_wdata, a_rdata    port A (the bank: reads only)
//   b_en, b_we[NB_COL], b_addr, b_wdata, b_rdata    port B (the bank: writes only)
// ============================================================================

`default_nettype none

module l1d_sram_tdp #(
    parameter DEPTH  = 64,
    parameter COL_W  = 8,
    parameter NB_COL = 64,
    parameter DATA_W = COL_W * NB_COL,
    parameter ADDR_W = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
    input  wire              clk,
    input  wire              a_en,
    input  wire [NB_COL-1:0] a_we,
    input  wire [ADDR_W-1:0] a_addr,
    input  wire [DATA_W-1:0] a_wdata,
    output reg  [DATA_W-1:0] a_rdata,
    input  wire              b_en,
    input  wire [NB_COL-1:0] b_we,
    input  wire [ADDR_W-1:0] b_addr,
    input  wire [DATA_W-1:0] b_wdata,
    output reg  [DATA_W-1:0] b_rdata
);
    reg [DATA_W-1:0] mem [0:DEPTH-1];
    integer i, c;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1)
            mem[i] = {DATA_W{1'b0}};
        a_rdata = {DATA_W{1'b0}};
        b_rdata = {DATA_W{1'b0}};
    end

    always @(posedge clk) begin
        if (a_en) begin
            for (c = 0; c < NB_COL; c = c + 1)
                if (a_we[c]) mem[a_addr][c*COL_W +: COL_W] <= a_wdata[c*COL_W +: COL_W];
            a_rdata <= mem[a_addr];
        end
        if (b_en) begin
            for (c = 0; c < NB_COL; c = c + 1)
                if (b_we[c]) mem[b_addr][c*COL_W +: COL_W] <= b_wdata[c*COL_W +: COL_W];
            b_rdata <= mem[b_addr];
        end
    end

`ifdef L1D_DEBUG
    always @(posedge clk)
        if (a_en && b_en && (a_addr == b_addr) && ((|a_we) || (|b_we)))
            $display("[%0t] L1D TDP %m same address on both ports with a write (port A reads old data)", $time);
`endif
endmodule

`default_nettype wire
