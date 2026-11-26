// ==========================================================================
// l1d_pf_sms.v - SMS (Spatial Memory Streaming) prefetcher of the L1D
// ==========================================================================
// IN SIMPLE WORDS : SMS learns which lines of a 2 KB region a piece of
//   code touches (its "footprint"), files the footprint under the
//   instruction that first touched the region (PC hash and line), and
//   replays it when the same instruction starts on a new region.
//   Example: a loop walks an array of structs and needs lines 0, 3 and
//   6 of each region. After one region is learned, the next region
//   gets lines 3 and 6 fetched at the moment line 0 is touched.
//   Spec: ../README.md. Shared rules: ../../README.md (sections 1 to 6).
//   This top module only connects three blocks (V-3.7):
//     u_front   pick one record per cycle, queue it   (l1d_pf_sms_front)
//     u_core    FT / AT / PHT learning and lookup     (l1d_pf_sms_core)
//     u_expand  pattern -> requests with bank credits (l1d_pf_sms_expand)
//
// STAGES FROM A TRIGGER TO THE FIRST REQUEST (registered stages):
//   FIFO (front) -> S0 -> S1 -> S2 (slot load) -> X0 -> X1 -> X2 (out)
//   = 7 registers after the FIFO input. README target: about 4 stages
//   from the training FIFO to the first request. Gap: 3 stages. NOT
//   PROVEN, nothing synthesised (GR-1 status in DESIGN_RULES.md).
//
// PARAMETERS (V-7.10), forwarded to the blocks:
//   name              default unit     meaning
//   PA_W              56      bits     physical address width
//   LINE_BYTES        64      bytes    cache line
//   REGION_BYTES      2048    bytes    region (max 4096: inside a page)
//   PC_W              32      bits     PC bits from the cache (assumed)
//   N_PORTS           4       ports    request ports (training records)
//   N_BANKS           4       banks    cache banks
//   FT_ENTRIES        16      entries  filter table
//   AT_ENTRIES        32      entries  accumulation table
//   PHT_SETS          32      sets     pattern history sets
//   PHT_WAYS          4       ways     pattern history ways
//   REGION_TAG_W      16      bits     region tag in FT / AT
//   PC_HASH_W         12      bits     PC hash
//   PHT_TAG_W         8       bits     PHT tag
//   EXPAND_SLOTS      4       slots    patterns issued at the same time
//   CREDIT_PER_BANK   2       count    requests without feedback per bank
//   CREDIT_WAIT_MAX   64      cycles   wait for a credit before giving up
//   TRAIN_FIFO_DEPTH  4       records  training FIFO
//   EVT_FIFO_DEPTH    4       records  eviction FIFO
//
// PORTS (default widths; first draft of the README, section 8, plus
//   tr_no_train and pf_ready as in the stream prefetcher):
//   clk, rst          in  1       clock, synchronous reset (high)
//   pf_enable         in  1       global on/off
//   tr_valid          in  4       training record per request port
//   tr_no_train       in  4       AMO / LR / SC / NC / wrong path: ignore
//   tr_pc             in  4x32    PC of the access
//   tr_line_addr      in  4x50    line address PA[55:6] of the access
//   tr_is_store       in  4       store (not used for patterns)
//   tr_miss           in  4       access missed (statistics only)
//   tr_pf_hit         in  4       hit on a prefetched line (statistics)
//   ev_valid          in  4       a line leaves the cache, per bank
//   ev_line_addr      in  4x50    its line address
//   pf_fb_valid       in  1       feedback for a prefetch request
//   pf_fb_addr        in  50      its line address
//   pf_fb_result      in  2       its result
//   pf_ready          in  1       the shared prefetch port accepts
//   pf_valid          out 1       prefetch request (registered)
//   pf_line_addr      out 50      physical line address; bank = [1:0]
//
// WORKED EXAMPLE: ../README.md section 10 (struct walk, lines {0,3,6}).
//
// ASSUMPTIONS (V-9.2): see the blocks. A1 region <= 4 KB and a power of
//   two. A2 tr_valid already excludes accesses that must not train, or
//   marks them with tr_no_train. A3 the cache gives an eviction event
//   (not in cache_v5 yet: plan item C-12, open SM-3).
// DEVIATIONS FROM THE README (all named in the blocks): one training
//   lane (README 2), no age field in the AT, an explicit time-out for
//   credits, tr_miss / tr_pf_hit not used (statistics only).
// ==========================================================================
`ifndef L1D_PF_SMS_V
`define L1D_PF_SMS_V
`default_nettype none

module l1d_pf_sms #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter PA_W             = 56,   // default 56 bits: physical addr
    parameter LINE_BYTES       = 64,   // default 64 bytes: cache line
    parameter REGION_BYTES     = 2048, // default 2048 bytes: region
    parameter PC_W             = 32,   // default 32 bits: PC from cache
    parameter N_PORTS          = 4,    // default 4: request ports
    parameter N_BANKS          = 4,    // default 4: cache banks
    parameter FT_ENTRIES       = 16,   // default 16: filter table
    parameter AT_ENTRIES       = 32,   // default 32: accumulation table
    parameter PHT_SETS         = 32,   // default 32: PHT sets
    parameter PHT_WAYS         = 4,    // default 4: PHT ways
    parameter REGION_TAG_W     = 16,   // default 16 bits: region tag
    parameter PC_HASH_W        = 12,   // default 12 bits: PC hash
    parameter PHT_TAG_W        = 8,    // default 8 bits: PHT tag
    parameter EXPAND_SLOTS     = 4,    // default 4: pattern slots
    parameter CREDIT_PER_BANK  = 2,    // default 2: credits per bank
    parameter CREDIT_WAIT_MAX  = 64,   // default 64 cycles: credit wait
    parameter TRAIN_FIFO_DEPTH = 4,    // default 4: training FIFO
    parameter EVT_FIFO_DEPTH   = 4     // default 4: eviction FIFO
) (
    // ---- clock and reset ----
    input  wire                                     clk,          // clock
    input  wire                                     rst,          // sync reset
    // ---- control ----
    input  wire                                     pf_enable,    // 1 = on
    // ---- training records (one per request port) ----
    input  wire [N_PORTS-1:0]                       tr_valid,     // record
    input  wire [N_PORTS-1:0]                       tr_no_train,  // ignore
    input  wire [N_PORTS*PC_W-1:0]                  tr_pc,        // 4 x 32
    input  wire [N_PORTS*(PA_W-$clog2(LINE_BYTES))-1:0] tr_line_addr, // 4 x 50
    input  wire [N_PORTS-1:0]                       tr_is_store,  // unused
    input  wire [N_PORTS-1:0]                       tr_miss,      // unused
    input  wire [N_PORTS-1:0]                       tr_pf_hit,    // unused
    // ---- eviction events (one per bank) ----
    input  wire [N_BANKS-1:0]                       ev_valid,     // event
    input  wire [N_BANKS*(PA_W-$clog2(LINE_BYTES))-1:0] ev_line_addr, // 4 x 50
    // ---- cache feedback for prefetch requests ----
    input  wire                                     pf_fb_valid,  // feedback
    input  wire [PA_W-$clog2(LINE_BYTES)-1:0]       pf_fb_addr,   // 50 bits
    input  wire [1:0]                               pf_fb_result, // 2 bits
    // ---- prefetch out (to the shared prefetch port) ----
    input  wire                                     pf_ready,     // accept
    output wire                                     pf_valid,     // request
    output wire [PA_W-$clog2(LINE_BYTES)-1:0]       pf_line_addr  // 50 bits
);

    // ---- local constants (value at the default) ----
    localparam LINE_ADDR_W = PA_W - $clog2(LINE_BYTES);          // 50 bits
    localparam LIR_W       = $clog2(REGION_BYTES / LINE_BYTES);  // 5 bits
    localparam REGION_NUM_W = LINE_ADDR_W - LIR_W;               // 45 bits
    localparam PATTERN_W   = (1 << LIR_W);                       // 32 bits

    // ---- wires between the blocks ----
    wire                     train_pop;            // 1: core takes record
    wire                     evt_pop;              // 1: core takes event
    wire                     train_head_valid;     // 1: record waiting
    wire [PC_HASH_W-1:0]     train_head_pc_hash;   // 12: its PC hash
    wire [LINE_ADDR_W-1:0]   train_head_line_addr; // 50: its line
    wire                     evt_head_valid;       // 1: event waiting
    wire [LINE_ADDR_W-1:0]   evt_head_line_addr;   // 50: its line
    wire                     load_valid;           // 1: pattern found
    wire [REGION_NUM_W-1:0]  load_region;          // 45: its region
    wire [PATTERN_W-1:0]     load_mask;            // 32: lines to fetch
    wire [LIR_W-1:0]         load_line;            // 5: trigger line

    // tr_is_store / tr_miss / tr_pf_hit are kept for the interface
    // (README section 8) and are not needed by the patterns
    wire unused_ok; // 1: keeps lint quiet about the unused inputs
    assign unused_ok = &{1'b0, tr_is_store, tr_miss, tr_pf_hit};

    l1d_pf_sms_front #(
        .N_PORTS          (N_PORTS),
        .N_BANKS          (N_BANKS),
        .PC_W             (PC_W),
        .LINE_ADDR_W      (LINE_ADDR_W),
        .PC_HASH_W        (PC_HASH_W),
        .TRAIN_FIFO_DEPTH (TRAIN_FIFO_DEPTH),
        .EVT_FIFO_DEPTH   (EVT_FIFO_DEPTH)
    ) u_front (
        .clk                  (clk),
        .rst                  (rst),
        .pf_enable            (pf_enable),
        .tr_valid             (tr_valid),
        .tr_no_train          (tr_no_train),
        .tr_pc                (tr_pc),
        .tr_line_addr         (tr_line_addr),
        .ev_valid             (ev_valid),
        .ev_line_addr         (ev_line_addr),
        .train_pop            (train_pop),
        .evt_pop              (evt_pop),
        .train_head_valid     (train_head_valid),
        .train_head_pc_hash   (train_head_pc_hash),
        .train_head_line_addr (train_head_line_addr),
        .evt_head_valid       (evt_head_valid),
        .evt_head_line_addr   (evt_head_line_addr)
    );

    l1d_pf_sms_core #(
        .LINE_ADDR_W  (LINE_ADDR_W),
        .LIR_W        (LIR_W),
        .PC_HASH_W    (PC_HASH_W),
        .REGION_TAG_W (REGION_TAG_W),
        .FT_ENTRIES   (FT_ENTRIES),
        .AT_ENTRIES   (AT_ENTRIES),
        .PHT_SETS     (PHT_SETS),
        .PHT_WAYS     (PHT_WAYS),
        .PHT_TAG_W    (PHT_TAG_W)
    ) u_core (
        .clk                  (clk),
        .rst                  (rst),
        .train_head_valid     (train_head_valid),
        .train_head_pc_hash   (train_head_pc_hash),
        .train_head_line_addr (train_head_line_addr),
        .evt_head_valid       (evt_head_valid),
        .evt_head_line_addr   (evt_head_line_addr),
        .train_pop            (train_pop),
        .evt_pop              (evt_pop),
        .load_valid           (load_valid),
        .load_region          (load_region),
        .load_mask            (load_mask),
        .load_line            (load_line)
    );

    l1d_pf_sms_expand #(
        .LINE_ADDR_W     (LINE_ADDR_W),
        .LIR_W           (LIR_W),
        .N_BANKS         (N_BANKS),
        .EXPAND_SLOTS    (EXPAND_SLOTS),
        .CREDIT_PER_BANK (CREDIT_PER_BANK),
        .CREDIT_WAIT_MAX (CREDIT_WAIT_MAX)
    ) u_expand (
        .clk          (clk),
        .rst          (rst),
        .pf_enable    (pf_enable),
        .load_valid   (load_valid),
        .load_region  (load_region),
        .load_mask    (load_mask),
        .load_line    (load_line),
        .pf_ready     (pf_ready),
        .pf_valid     (pf_valid),
        .pf_line_addr (pf_line_addr),
        .fb_valid     (pf_fb_valid),
        .fb_addr      (pf_fb_addr),
        .fb_result    (pf_fb_result)
    );

endmodule

`default_nettype wire
`endif
