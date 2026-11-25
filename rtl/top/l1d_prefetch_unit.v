// ============================================================================
// l1d_prefetch_unit.v - the prefetch system of the L1D (3-cycle version):
//                       stride, stream and SMS beside the banks; training
//                       in, at most one hint per bank per cycle out
// ============================================================================
// IN SIMPLE WORDS
//   Three "guessers" watch what the program loads and stores and guess
//   which cache lines it will want next:
//     stride  needs the instruction PC: "this load walks the array 128
//             bytes at a time"                        (l1d_pf3_stride)
//     stream  needs no PC: "the program walks this page line after line"
//                                                     (l1d_pf3_stream)
//     SMS     remembers which lines of a 2 KB region a piece of code
//             touches and replays it                  (l1d_pf3_sms_core,
//                                                      l1d_pf3_sms_expand)
//   The rank stage (l1d_pf3_rank) takes their guesses, keeps the best one
//   per bank (stride > stream > SMS, rule SH-1), drops duplicates and
//   registers the hints for the banks.
//
// THE 3-CYCLE PROMISE (user decision, 2026-11-11; GOD RULE GR-1)
//   A training record on the pins in cycle t (from the banks' S2 flops)
//   gives its first hint in pf_req_* in cycle t + 3, when the prefetcher
//   and the bank are idle:
//     S1 (t)    pick, hashes, compares       -> registers of S1
//     S2 (t+1)  table check and update,      -> candidate registers
//               candidate
//     S3 (t+2)  rank per bank                -> pf_req_* (output register)
//   Every stage must complete in under 240 ps after place and route.
//
// INSTANTIATED BY
//   l1d_top, instance u_prefetch (one copy, generate on HAS_PF).
//   This module only connects the blocks (V-3.7).
//
// PARAMETERS (defaults from l1d_defines.vh)
//   `L1D_PARAMS   all cache parameters; used here: NUM_BANKS (4), PC_W (64),
//                 ADDR_W (64), LINE_BYTES (64)
//   PF_HOLD_MAX   default 2: decision cycles of a request in the rank stage
//   PF_PA_W       default 56: physical address bits the prefetchers use
//   PF_REGION_BYTES default 2048: SMS region (32 lines of 64 bytes)
//
// PORTS (same as the 5-stage l1d_prefetch_unit; b = bank, [b*W +: W])
//   clk, rst                     in   1          clock; synchronous reset
//   pf_enable                    in   1          1 = prefetching on
//   train_valid/_pc/_addr/_is_store/_miss/_pf_hit/_no_train
//                                in   4 / 4x64 / 4x64 / 4 / 4 / 4 / 4
//   evict_valid, evict_line_addr in   4 / 4x58   the banks' eviction events
//   pf_fb_valid/_line_addr/_result in 4 / 4x58 / 4x2  feedback (credits)
//   pf_req_ready                 in   4          bank b takes a hint (flop)
//   pf_req_valid/_line_addr/_unique out 4 / 4x58 / 4  hints (registered)
//
// WORKED EXAMPLE: see l1d_pf3_stride (stride 0x80 at PC X: the third
//   access in cycle t gives the hint for line 0x1200 in cycle t + 3).
//
// LOGIC LEVELS (T-9): none of its own (wiring of the blocks; the stages
//   and their levels are in each block header: S1 / S2 in the prefetchers,
//   S3 in l1d_pf3_rank).
// ASSUMPTIONS AND LIMITS (V-9.2; all are "hints are lossy")
//   A1 stride and stream take one record per cycle (lowest bank lane);
//      SMS takes one (rotating lane) and queues the rest in its FIFO.
//   A2 the feedback of bank b returns one SMS credit of bank b; its line
//      address and result are not needed (the bank is the lane).
//   A3 a hint registered here reaches the bank one cycle after
//      pf_req_ready was read: the bank drops it if its register was
//      booked meanwhile (as the 5-stage unit).
// ============================================================================
`ifndef L1D_PREFETCH_UNIT_V
`define L1D_PREFETCH_UNIT_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_prefetch_unit #(
    `L1D_PARAMS,                                    // all cache parameters
    parameter PF_HOLD_MAX = `L1D_PF_HOLD_MAX,       // default 2: decision cycles
    parameter PF_PA_W     = 56,                     // default 56 bits: PA width
    parameter PF_REGION_BYTES = 2048                // default 2048 bytes: SMS region
) (
    clk, rst, pf_enable,
    train_valid, train_pc, train_addr, train_is_store, train_miss,
    train_pf_hit, train_no_train,
    evict_valid, evict_line_addr,
    pf_fb_valid, pf_fb_line_addr, pf_fb_result,
    pf_req_ready, pf_req_valid, pf_req_line_addr, pf_req_unique
);
    `include "l1d_fields.vh"

    // ---- local constants (value at the defaults) ----
    localparam NB = NUM_BANKS;                     // banks, 4
    localparam SMS_LIR_W = $clog2(PF_REGION_BYTES / LINE_BYTES); // 5 bits:
                                                   //   line in region
    localparam SMS_PAT_W = (1 << SMS_LIR_W);       // 32 bits: footprint

    // ---- ports ----
    input  wire                      clk;               // clock
    input  wire                      rst;               // sync reset, high
    input  wire                      pf_enable;         // 1 = on
    input  wire [NB-1:0]             train_valid;       // 4
    input  wire [NB*PC_W-1:0]        train_pc;          // 4 x 64
    input  wire [NB*ADDR_W-1:0]      train_addr;        // 4 x 64
    input  wire [NB-1:0]             train_is_store;    // 4
    input  wire [NB-1:0]             train_miss;        // 4
    input  wire [NB-1:0]             train_pf_hit;      // 4
    input  wire [NB-1:0]             train_no_train;    // 4
    input  wire [NB-1:0]             evict_valid;       // 4
    input  wire [NB*LINE_ADDR_W-1:0] evict_line_addr;   // 4 x 58
    input  wire [NB-1:0]             pf_fb_valid;       // 4
    input  wire [NB*LINE_ADDR_W-1:0] pf_fb_line_addr;   // 4 x 58 (A2: unused)
    input  wire [NB*2-1:0]           pf_fb_result;      // 4 x 2 (A2: unused)
    input  wire [NB-1:0]             pf_req_ready;      // 4
    output wire [NB-1:0]             pf_req_valid;      // 4, registered
    output wire [NB*LINE_ADDR_W-1:0] pf_req_line_addr;  // 4 x 58, registered
    output wire [NB-1:0]             pf_req_unique;     // 4, registered

    // ---- wires between the blocks ----
    wire                   stride_clear;   // satp write (not wired: 0)
    wire                   s_valid;        // stride candidate (S3, comb.)
    wire [NB:0]            s_valid_dp;     // NB+1 = 5: same, copies for
                                           //   the rank data
    wire [LINE_ADDR_W-1:0] s_line;         // 58 bits
    wire [NB-1:0]          s_bank;         // 4
    wire                   s_unique;       // ask unique
    wire                   t_valid;        // stream candidate (flops)
    wire [NB:0]            t_valid_dp;     // NB+1 = 5: same, copies for
                                           //   the rank data
    wire [LINE_ADDR_W-1:0] t_line;         // 58 bits
    wire [LINE_ADDR_W-1:0] t_line_scan;    // 58 bits: stream scan line
    wire [LINE_ADDR_W-1:0] t_line_direct;  // 58 bits: stream direct line
    wire                   t_pick_scan;    // t_line is the scan line
    wire [NB-1:0]          t_bank;         // 4
    wire                   t_unique;       // ask unique
    wire                   t_held;         // stream offer still held
    wire                   m_valid;        // SMS candidate (flops)
    wire                   m_valid_dp;     // the same, data copy
    wire [LINE_ADDR_W-1:0] m_line;         // 58 bits
    wire [LINE_ADDR_W-1:0] m_line_scan;    // 58 bits: SMS scan line
    wire [LINE_ADDR_W-1:0] m_line_direct;  // 58 bits: SMS direct line
    wire                   m_pick_scan;    // m_line is the scan line
    wire [NB-1:0]          m_bank;         // 4
    wire                   m_held;         // SMS offer still held
    wire                   load_valid;     // SMS footprint (S3, from flops)
    wire [PF_PA_W-OFFSET_W-SMS_LIR_W-1:0] load_region; // 45 bits
    wire [SMS_LIR_W-1:0]   load_line;      // SMS_LIR_W = 5 bits, line in region
    wire [SMS_LIR_W-1:0]   load_first;     // 5 bits
    wire [NB-1:0]          load_first_bank;// 4
    wire [SMS_LIR_W-1:0]   load_second;    // 5 bits, line after the first
    wire [SMS_PAT_W-1:0]   load_rest;      // SMS_PAT_W = 32 bits
    wire                   fb_unused;      // keeps lint quiet (A2)

    assign stride_clear = 1'b0;
    assign fb_unused    = &{1'b0, pf_fb_line_addr, pf_fb_result};

    l1d_pf3_stride #(
        .NB         (NB),
        .PC_W       (PC_W),
        .ADDR_W     (ADDR_W),
        .PA_W       (PF_PA_W),
        .LINE_BYTES (LINE_BYTES)
    ) u_stride (
        .clk            (clk),
        .rst            (rst),
        .pf_enable      (pf_enable),
        .clear          (stride_clear),
        .train_valid    (train_valid),
        .train_no_train (train_no_train),
        .train_is_store (train_is_store),
        .train_pc       (train_pc),
        .train_addr     (train_addr),
        .cand_valid     (s_valid),
        .cand_valid_dp  (s_valid_dp),
        .cand_line      (s_line),
        .cand_bank      (s_bank),
        .cand_unique    (s_unique)
    );

    l1d_pf3_stream #(
        .NB         (NB),
        .ADDR_W     (ADDR_W),
        .PA_W       (PF_PA_W),
        .LINE_BYTES (LINE_BYTES)
    ) u_stream (
        .clk            (clk),
        .rst            (rst),
        .pf_enable      (pf_enable),
        .train_valid    (train_valid),
        .train_no_train (train_no_train),
        .train_addr     (train_addr),
        .train_is_store (train_is_store),
        .train_miss     (train_miss),
        .train_pf_hit   (train_pf_hit),
        .held           (t_held),
        .cand_valid     (t_valid),
        .cand_valid_dp  (t_valid_dp),
        .cand_line      (t_line),
        .cand_line_scan   (t_line_scan),
        .cand_line_direct (t_line_direct),
        .cand_pick_scan   (t_pick_scan),
        .cand_bank      (t_bank),
        .cand_unique    (t_unique)
    );

    l1d_pf3_sms_core #(
        .NB         (NB),
        .PC_W       (PC_W),
        .ADDR_W     (ADDR_W),
        .PA_W       (PF_PA_W),
        .LINE_BYTES (LINE_BYTES),
        .REGION_BYTES (PF_REGION_BYTES),
        .LINE_W     (LINE_ADDR_W)
    ) u_sms_core (
        .clk             (clk),
        .rst             (rst),
        .pf_enable       (pf_enable),
        .train_valid     (train_valid),
        .train_no_train  (train_no_train),
        .train_pc        (train_pc),
        .train_addr      (train_addr),
        .evict_valid     (evict_valid),
        .evict_line_addr (evict_line_addr),
        .load_valid      (load_valid),
        .load_region     (load_region),
        .load_line       (load_line),
        .load_first      (load_first),
        .load_first_bank (load_first_bank),
        .load_second     (load_second),
        .load_rest       (load_rest)
    );

    l1d_pf3_sms_expand #(
        .NB           (NB),
        .LINE_W       (LINE_ADDR_W),
        .REGION_NUM_W (PF_PA_W - OFFSET_W - SMS_LIR_W),
        .LIR_W        (SMS_LIR_W)
    ) u_sms_expand (
        .clk             (clk),
        .rst             (rst),
        .pf_enable       (pf_enable),
        .load_valid      (load_valid),
        .load_region     (load_region),
        .load_line       (load_line),
        .load_first      (load_first),
        .load_first_bank (load_first_bank),
        .load_second     (load_second),
        .load_rest       (load_rest),
        .held            (m_held),
        .cand_valid      (m_valid),
        .cand_line       (m_line),
        .cand_bank       (m_bank),
        .cand_valid_dp   (m_valid_dp),
        .cand_line_scan  (m_line_scan),
        .cand_line_direct(m_line_direct),
        .cand_pick_scan  (m_pick_scan)
    );

    l1d_pf3_rank #(
        .NB       (NB),
        .LINE_W   (LINE_ADDR_W),
        .HOLD_MAX (PF_HOLD_MAX)
    ) u_rank (
        .clk           (clk),
        .rst           (rst),
        .pf_enable     (pf_enable),
        .pf_req_ready  (pf_req_ready),
        .s_valid       (s_valid),
        .s_valid_dp    (s_valid_dp),
        .s_line        (s_line),
        .s_bank        (s_bank),
        .s_unique      (s_unique),
        .t_valid       (t_valid),
        .t_valid_dp    (t_valid_dp),
        .t_line        (t_line),
        .t_line_scan   (t_line_scan),
        .t_line_direct (t_line_direct),
        .t_pick_scan   (t_pick_scan),
        .t_bank        (t_bank),
        .t_unique      (t_unique),
        .m_valid       (m_valid),
        .m_valid_dp    (m_valid_dp),
        .m_line        (m_line),
        .m_line_scan   (m_line_scan),
        .m_line_direct (m_line_direct),
        .m_pick_scan   (m_pick_scan),
        .m_bank        (m_bank),
        .fb_valid      (pf_fb_valid),
        .t_held        (t_held),
        .m_held        (m_held),
        .pf_req_valid  (pf_req_valid),
        .pf_req_line   (pf_req_line_addr),
        .pf_req_unique (pf_req_unique)
    );

endmodule

`default_nettype wire
`endif // L1D_PREFETCH_UNIT_V
