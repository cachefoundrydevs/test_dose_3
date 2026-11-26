// ============================================================================
// l1d_prefetch_unit.v  -  the three prefetchers (stride, stream, SMS) beside the banks: training in, one hint per bank out
// ============================================================================
// IN SIMPLE WORDS  (cache_v6, PLAN_v6.md section 6.2)
//   Three "guessers" watch what the program loads and stores and guess which cache lines it will want next:
//     stride  needs the instruction PC: "this load walks the array 128 bytes at a time"       (rtl/prefetch/stride)
//     stream  needs no PC: "the program walks this page line after line"                      (rtl/prefetch/stream)
//     SMS     remembers which lines of a 2 KB region a piece of code touches and replays it   (rtl/prefetch/sms)
//   This unit feeds them with the TRAINING RECORDS the banks make in S2 (one per bank per cycle), the EVICTION events and
//   the FEEDBACK of earlier hints, takes their guesses, drops duplicates, ranks them (stride > stream > SMS, rule SH-1)
//   and hands at most ONE hint per bank per cycle to the bank's prefetch register (l1d_bank_s0).
//
//      bank training records (4) --> pick the lowest bank --> stride, stream (one record per cycle each)
//                                --> all four lanes ------> SMS (N_PORTS = 4 lanes)
//      bank eviction events (4), bank feedback (4, lowest bank picked) --> SMS
//      stride / stream / SMS requests --> STAGE A (flops: line, unique, hold counter, 3 pairwise equalities)
//                                     --> STAGE B (flops: per bank the highest-rank request) --> pf_req_* to the banks
//
// STAGE / CYCLE
//   Off the hit path. Stage A registers a prefetcher's request once (a 2-cycle cadence per prefetcher: capture, decide);
//   stage B decides from the stage A flops and registers the per-bank hints. pf_ready back to a prefetcher = its request in
//   stage A was taken or dropped this cycle (from flops: about 6 levels to the prefetcher's pin).
//   ESTIMATED LOGIC LEVELS (T-9, not measured): stage A load: 58-bit equality from a 2:1 (1 + 7) = 8 into the equality
//   flops; stage B: bank compare (1) + duplicate / hold gate (2) + rank chain (2) + 3:1 AND-OR (2) = 7 into the hint flops;
//   the training pick: lowest bank (2) + 4:1 AND-OR (2) = 4 into the prefetchers' first stage (their own logic follows:
//   stride tap stage, stream S0 about 9 levels). The prefetchers themselves miss GR-1 (measured, PLAN_v6 O-7).
//
// INSTANTIATED BY
//   l1d_top, instance u_prefetch (one copy, generate on HAS_PF).
//
// PARAMETERS (defaults from l1d_defines.vh)
//   `L1D_PARAMS      all cache parameters; used here: NUM_BANKS (4), PC_W (64), ADDR_W (64), LINE_BYTES (64), LINE_ADDR_W (58)
//   PF_HOLD_MAX      default 2: cycles a request may wait in stage A for its bank before it is dropped (SH-1)
//   PF_PA_W          default 56: physical address bits the prefetchers work with (their PA_W); the cache address is 64 bits
//
// PORTS (name, direction, width at the defaults, meaning)                 b = bank 0..3; wide buses: bank b at [b*W +: W]
//   clk, rst                     in   1          clock; synchronous reset, active high
//   pf_enable                    in   1          1 = prefetching on; 0 = no training, every request dropped
//   train_valid/_pc/_addr/_is_store/_miss/_pf_hit/_no_train   in  4 / 4x64 / 4x64 / 4 / 4 / 4 / 4   the banks' training records
//   evict_valid, evict_line_addr in   4 / 4x58   the banks' eviction events
//   pf_fb_valid/_line_addr/_result  in 4 / 4x58 / 4x2   the banks' feedback for earlier hints (PF_FB_*)
//   pf_req_ready                 in   4          per bank: its prefetch register may take a new hint (flop of the bank)
//   pf_req_valid/_line_addr/_unique  out 4 / 4x58 / 4   per bank: the hint (registered, valid one cycle)
//
// WORKED EXAMPLE (a loop reads 0x1000, 0x1080, 0x1100 ... on port 0, bank of 0x1080 = 2)
//   The stride prefetcher sees PC X with stride 0x80 three times (records from the banks, one per cycle), and asks for
//   line 0x1200 (two strides ahead). Stage A captures it (a_busy[0] = 1, bank 0); next cycle stage B finds no other request
//   for bank 0, pf_req_ready[0] = 1 -> pf_req_valid[0] = 1 with the line, pf_ready to the stride prefetcher pulses. The bank
//   books the hint at the lowest priority, runs it as a tagged load and allocates a prefetch MSHR entry; its feedback
//   (ISSUED) comes back here three cycles later.
//
// WIDTH ADAPTATION (the prefetchers were written for PA_W = 56 physical bits; the cache address is ADDR_W = 64)
//   training address to a prefetcher: addr[PF_PA_W-1:0]; a line address to SMS: addr[PF_PA_W-1:OFFSET_W] (50 bits);
//   a prefetcher's 50-bit line address back to the cache: zero-extended to LINE_ADDR_W = 58; the stride prefetcher gives a
//   64-bit byte address: its [ADDR_W-1:OFFSET_W] is the line.
//
// ASSUMPTIONS AND LIMITS (V-9.2; all are "hints are lossy", no function of the cache depends on them)
//   A1 stride and stream take ONE record per cycle: when several banks train in one cycle the lowest bank wins, the rest are
//      dropped (PLAN_v6 O-9); SMS takes all four. A2 the SMS feedback port takes one event per cycle: the lowest bank's wins.
//   A3 a hint registered in stage B reaches the bank one cycle after pf_req_ready was read: the bank drops it if its register
//      was booked meanwhile. A4 the stride prefetcher may replace its shown request by a newer one while stage A holds the
//      older copy (its "newest first" rule): the pf_ready pulse then pops the newer one unseen.
// ============================================================================
`ifndef L1D_PREFETCH_UNIT_V
`define L1D_PREFETCH_UNIT_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_prefetch_unit #(
    `L1D_PARAMS,                                           // all cache parameters (l1d_defines.vh)
    parameter PF_HOLD_MAX = `L1D_PF_HOLD_MAX,              // default 2: cycles a request may wait in stage A (SH-1)
    parameter PF_PA_W     = 56                             // default 56 bits: physical address width of the prefetchers
) (
    clk, rst, pf_enable,
    train_valid, train_pc, train_addr, train_is_store, train_miss, train_pf_hit, train_no_train,
    evict_valid, evict_line_addr,
    pf_fb_valid, pf_fb_line_addr, pf_fb_result,
    pf_req_ready, pf_req_valid, pf_req_line_addr, pf_req_unique
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // ---- local constants (value at the defaults) ----
    localparam NB        = NUM_BANKS;                      // banks, 4
    localparam PF_LINE_W = PF_PA_W - OFFSET_W;             // line address width of the prefetchers, 56 - 6 = 50 bits
    localparam LINE_PAD  = LINE_ADDR_W - PF_LINE_W;        // zero bits added on top of a prefetcher's line, 8 bits
    localparam NPF       = 3;                              // prefetchers: 0 stride, 1 stream, 2 SMS (rank order)
    localparam PF_STRIDE = 0;                              // rank 0 (highest)
    localparam PF_STREAM = 1;                              // rank 1
    localparam PF_SMS    = 2;                              // rank 2 (lowest)
    localparam HOLD_W    = $clog2(PF_HOLD_MAX + 1);        // hold counter width, 2 bits
    localparam [HOLD_W-1:0] HOLD_LAST = PF_HOLD_MAX - 1;   // the counter value of the last chance, 1

    // ---- ports ----
    input  wire                      clk;                  // clock: all registers change on its rising edge
    input  wire                      rst;                  // synchronous reset, active high
    input  wire                      pf_enable;            // 1 = prefetching on
    input  wire [NB-1:0]             train_valid;          // per bank: a training record this cycle, 4 bits
    input  wire [NB*PC_W-1:0]        train_pc;             // per bank: its instruction PC, 4*64 bits
    input  wire [NB*ADDR_W-1:0]      train_addr;           // per bank: its byte address, 4*64 bits
    input  wire [NB-1:0]             train_is_store;       // per bank: store class, 4 bits
    input  wire [NB-1:0]             train_miss;           // per bank: the access missed, 4 bits
    input  wire [NB-1:0]             train_pf_hit;         // per bank: the access hit a prefetched line, 4 bits
    input  wire [NB-1:0]             train_no_train;       // per bank: atomic / non-cacheable / MMU: ignore, 4 bits
    input  wire [NB-1:0]             evict_valid;          // per bank: a line left the array, 4 bits
    input  wire [NB*LINE_ADDR_W-1:0] evict_line_addr;      // per bank: its line address, 4*58 bits
    input  wire [NB-1:0]             pf_fb_valid;          // per bank: feedback for a hint, 4 bits
    input  wire [NB*LINE_ADDR_W-1:0] pf_fb_line_addr;      // per bank: its line address, 4*58 bits
    input  wire [NB*2-1:0]           pf_fb_result;         // per bank: PF_FB_*, 4*2 bits
    input  wire [NB-1:0]             pf_req_ready;         // per bank: the prefetch register may take a hint (flop), 4 bits
    output reg  [NB-1:0]             pf_req_valid;         // per bank: a hint this cycle (registered), 4 bits
    output reg  [NB*LINE_ADDR_W-1:0] pf_req_line_addr;     // per bank: its line address (registered), 4*58 bits
    output reg  [NB-1:0]             pf_req_unique;        // per bank: it wants the line unique (registered), 4 bits

    // ======================================================================
    // Training record for stride and stream: the lowest bank with a record that may train (one per cycle, A1)
    // ======================================================================
    reg  [NB-1:0]      rec_pick_onehot;                    // the chosen bank, one-hot (or zero), 4 bits
    reg                rec_pick_seen;                      // a lower bank already has a record
    reg                rec_valid;                          // a record is chosen
    reg  [PC_W-1:0]    rec_pc;                             // its PC, 64 bits
    reg  [ADDR_W-1:0]  rec_addr;                           // its byte address, 64 bits
    reg                rec_is_store;                       // store class
    reg                rec_miss;                           // the access missed
    reg                rec_pf_hit;                         // the access hit a prefetched line
    integer rec_bank;                                      // loop variable: bank
    always @* begin
        rec_pick_seen = 1'b0;
        rec_valid     = 1'b0;
        rec_pc        = {PC_W{1'b0}};
        rec_addr      = {ADDR_W{1'b0}};
        rec_is_store  = 1'b0;
        rec_miss      = 1'b0;
        rec_pf_hit    = 1'b0;
        for (rec_bank = 0; rec_bank < NB; rec_bank = rec_bank + 1) begin
            rec_pick_onehot[rec_bank] = train_valid[rec_bank] & ~train_no_train[rec_bank] & ~rec_pick_seen;
            rec_pick_seen = rec_pick_seen | (train_valid[rec_bank] & ~train_no_train[rec_bank]);
            rec_valid     = rec_valid    | rec_pick_onehot[rec_bank];
            rec_pc        = rec_pc       | ({PC_W{rec_pick_onehot[rec_bank]}}   & train_pc[rec_bank*PC_W +: PC_W]);
            rec_addr      = rec_addr     | ({ADDR_W{rec_pick_onehot[rec_bank]}} & train_addr[rec_bank*ADDR_W +: ADDR_W]);
            rec_is_store  = rec_is_store | (rec_pick_onehot[rec_bank] & train_is_store[rec_bank]);
            rec_miss      = rec_miss     | (rec_pick_onehot[rec_bank] & train_miss[rec_bank]);
            rec_pf_hit    = rec_pf_hit   | (rec_pick_onehot[rec_bank] & train_pf_hit[rec_bank]);
        end
    end
    wire                rec_train_on;                      // the chosen record trains (prefetching enabled)
    wire [PF_PA_W-1:0]  rec_pa;                            // its physical address for the prefetchers, 56 bits
    wire                no_train_zero;                     // the stream prefetcher's "ignore" input: the pick already filtered
    assign rec_train_on  = rec_valid & pf_enable;
    assign rec_pa        = rec_addr[PF_PA_W-1:0];
    assign no_train_zero = 1'b0;

    // ======================================================================
    // Feedback for SMS: the lowest bank with a feedback event this cycle (A2)
    // ======================================================================
    reg  [NB-1:0]           fb_pick_onehot;                // the chosen bank, one-hot, 4 bits
    reg                     fb_pick_seen;                  // a lower bank already has feedback
    reg                     fb_valid;                      // a feedback event is chosen
    reg  [LINE_ADDR_W-1:0]  fb_line;                       // its line address, 58 bits
    reg  [1:0]              fb_result;                     // its result, 2 bits
    integer fb_bank;                                       // loop variable: bank
    always @* begin
        fb_pick_seen = 1'b0;
        fb_valid     = 1'b0;
        fb_line      = {LINE_ADDR_W{1'b0}};
        fb_result    = 2'b00;
        for (fb_bank = 0; fb_bank < NB; fb_bank = fb_bank + 1) begin
            fb_pick_onehot[fb_bank] = pf_fb_valid[fb_bank] & ~fb_pick_seen;
            fb_pick_seen = fb_pick_seen | pf_fb_valid[fb_bank];
            fb_valid     = fb_valid  | fb_pick_onehot[fb_bank];
            fb_line      = fb_line   | ({LINE_ADDR_W{fb_pick_onehot[fb_bank]}} & pf_fb_line_addr[fb_bank*LINE_ADDR_W +: LINE_ADDR_W]);
            fb_result    = fb_result | ({2{fb_pick_onehot[fb_bank]}} & pf_fb_result[fb_bank*2 +: 2]);
        end
    end
    wire [PF_LINE_W-1:0] fb_line_pf;                       // the feedback line at the prefetchers' width, 50 bits
    assign fb_line_pf = fb_line[PF_LINE_W-1:0];

    // ---- the SMS lanes: the banks' records and events at the prefetchers' widths ----
    wire [NB*PF_LINE_W-1:0] sms_train_line;                // per bank: the record's line address, 4*50 bits
    wire [NB*PF_LINE_W-1:0] sms_evict_line;                // per bank: the evicted line, 4*50 bits
    wire [NB-1:0]           sms_train_valid;               // per bank: record valid and prefetching on, 4 bits
    genvar lane;                                           // loop variable: bank lane
    generate
        for (lane = 0; lane < NB; lane = lane + 1) begin : g_sms_lane
            assign sms_train_line[lane*PF_LINE_W +: PF_LINE_W] = train_addr[lane*ADDR_W + OFFSET_W +: PF_LINE_W];
            assign sms_evict_line[lane*PF_LINE_W +: PF_LINE_W] = evict_line_addr[lane*LINE_ADDR_W +: PF_LINE_W];
            assign sms_train_valid[lane] = train_valid[lane] & pf_enable;
        end
    endgenerate

    // ======================================================================
    // The three prefetchers (taken as they are from rtl/prefetch/; PLAN_v6 O-7: they miss GR-1, measured)
    // ======================================================================
    wire [NPF-1:0]       src_valid;                        // per prefetcher: a request is shown, 3 bits
    wire [NPF-1:0]       src_unique;                       // per prefetcher: it wants the line unique, 3 bits
    wire [NPF-1:0]       src_ready;                        // per prefetcher: its request is taken or dropped now, 3 bits
    wire [ADDR_W-1:0]    stride_addr;                      // the stride prefetcher's byte address, 64 bits
    wire [PF_LINE_W-1:0] stream_line;                      // the stream prefetcher's line, 50 bits
    wire [PF_LINE_W-1:0] sms_line;                         // the SMS prefetcher's line, 50 bits
    wire                 stream_flag_unused;               // the stream prefetcher's constant "prefetch" flag
    wire                 stride_clear;                     // the stride table clear (a satp write: not wired in this version)
    assign stride_clear = 1'b0;

    l1d_pf_stride #(.PC_W(PC_W), .PA_W(PF_PA_W), .ADDR_W(ADDR_W), .LINE_BYTES(LINE_BYTES)) u_pf_stride (
        .rst         (rst),
        .clk         (clk),
        .clear       (stride_clear),
        .tr_valid    (rec_train_on),
        .tr_pc       (rec_pc),
        .tr_pa       (rec_pa),
        .tr_is_store (rec_is_store),
        .pf_ready    (src_ready[PF_STRIDE]),
        .pf_valid    (src_valid[PF_STRIDE]),
        .pf_addr     (stride_addr),
        .pf_unique   (src_unique[PF_STRIDE])
    );

    l1d_pf_stream #(.PA_W(PF_PA_W), .LINE_BYTES(LINE_BYTES)) u_pf_stream (
        .clk          (clk),
        .rst          (rst),
        .pf_enable    (pf_enable),
        .tr_valid     (rec_train_on),
        .tr_addr      (rec_pa),
        .tr_is_store  (rec_is_store),
        .tr_no_train  (no_train_zero),
        .tr_miss      (rec_miss),
        .tr_pf_hit    (rec_pf_hit),
        .pf_ready     (src_ready[PF_STREAM]),
        .pf_valid     (src_valid[PF_STREAM]),
        .pf_line_addr (stream_line),
        .pf_unique    (src_unique[PF_STREAM]),
        .pf_flag      (stream_flag_unused)
    );

    l1d_pf_sms #(.PA_W(PF_PA_W), .LINE_BYTES(LINE_BYTES), .PC_W(PC_W), .N_PORTS(NB), .N_BANKS(NB)) u_pf_sms (
        .clk          (clk),
        .rst          (rst),
        .pf_enable    (pf_enable),
        .tr_valid     (sms_train_valid),
        .tr_no_train  (train_no_train),
        .tr_pc        (train_pc),
        .tr_line_addr (sms_train_line),
        .tr_is_store  (train_is_store),
        .tr_miss      (train_miss),
        .tr_pf_hit    (train_pf_hit),
        .ev_valid     (evict_valid),
        .ev_line_addr (sms_evict_line),
        .pf_fb_valid  (fb_valid),
        .pf_fb_addr   (fb_line_pf),
        .pf_fb_result (fb_result),
        .pf_ready     (src_ready[PF_SMS]),
        .pf_valid     (src_valid[PF_SMS]),
        .pf_line_addr (sms_line)
    );
    assign src_unique[PF_SMS] = 1'b0;                      // SMS asks for shared lines only

    // the three requests as cache line addresses (58 bits)
    wire [LINE_ADDR_W-1:0] src_line [0:NPF-1];             // 3 x 58 bits
    assign src_line[PF_STRIDE] = stride_addr[ADDR_W-1:OFFSET_W];
    assign src_line[PF_STREAM] = {{LINE_PAD{1'b0}}, stream_line};
    assign src_line[PF_SMS]    = {{LINE_PAD{1'b0}}, sms_line};

    // ======================================================================
    // STAGE A: one captured request per prefetcher, its hold counter and the three pairwise line equalities (flops)
    // ======================================================================
    reg  [NPF-1:0]         a_busy_q;                       // stage A holds a request of prefetcher x, 3 bits
    reg  [LINE_ADDR_W-1:0] a_line_q [0:NPF-1];             // its line address, 3 x 58 bits
    reg  [NPF-1:0]         a_unique_q;                     // it wants the line unique, 3 bits
    reg  [HOLD_W-1:0]      a_hold_q [0:NPF-1];             // cycles it has waited in stage A, 3 x 2 bits
    reg  [NPF-1:0]         a_hold_last_q;                  // flop: this is its last chance (hold == PF_HOLD_MAX - 1), 3 bits
    reg                    eq_stride_stream_q;             // flop: stride's and stream's lines are equal
    reg                    eq_stride_sms_q;                // flop: stride's and SMS's lines are equal
    reg                    eq_stream_sms_q;                // flop: stream's and SMS's lines are equal
    wire [NPF-1:0]         a_load;                         // stage A captures the shown request now (only when empty), 3 bits
    wire [NPF-1:0]         a_done;                         // the held request is taken or dropped now, 3 bits
    wire [LINE_ADDR_W-1:0] a_line_next [0:NPF-1];          // the line stage A holds after this edge, 3 x 58 bits
    reg  [HOLD_W-1:0]      a_hold_next [0:NPF-1];          // the hold counter after this edge, 3 x 2 bits
    genvar ax;                                             // loop variable: prefetcher
    generate
        for (ax = 0; ax < NPF; ax = ax + 1) begin : g_stage_a
            assign a_load[ax]      = ~a_busy_q[ax] & src_valid[ax] & pf_enable;
            assign a_line_next[ax] = a_load[ax] ? src_line[ax] : a_line_q[ax];
        end
    endgenerate
    integer hx;                                            // loop variable: prefetcher (hold counter)
    always @* begin
        for (hx = 0; hx < NPF; hx = hx + 1) begin
            if (a_load[hx])          a_hold_next[hx] = {HOLD_W{1'b0}};
            else if (a_busy_q[hx])   a_hold_next[hx] = a_hold_q[hx] + {{(HOLD_W-1){1'b0}}, 1'b1};
            else                     a_hold_next[hx] = {HOLD_W{1'b0}};
        end
    end
    integer sx;                                            // loop variable: prefetcher (stage A flops)
    always @(posedge clk) begin : stage_a_proc
        if (rst) begin
            a_busy_q           <= {NPF{1'b0}};
            a_unique_q         <= {NPF{1'b0}};
            a_hold_last_q      <= {NPF{1'b0}};
            eq_stride_stream_q <= 1'b0;
            eq_stride_sms_q    <= 1'b0;
            eq_stream_sms_q    <= 1'b0;
            for (sx = 0; sx < NPF; sx = sx + 1) begin
                a_line_q[sx] <= {LINE_ADDR_W{1'b0}};
                a_hold_q[sx] <= {HOLD_W{1'b0}};
            end
        end else begin
            for (sx = 0; sx < NPF; sx = sx + 1) begin
                // a request stays until taken or dropped; a new one is captured only into an empty place (so the request
                // the prefetcher still shows in its pf_ready cycle is not captured twice)
                a_busy_q[sx]      <= (a_busy_q[sx] & ~a_done[sx]) | a_load[sx];
                a_line_q[sx]      <= a_line_next[sx];
                a_hold_q[sx]      <= a_hold_next[sx];
                a_hold_last_q[sx] <= (a_hold_next[sx] == HOLD_LAST);
                if (a_load[sx]) a_unique_q[sx] <= src_unique[sx];
            end
            // the equalities of the lines stage A holds next cycle (one 2:1 and a 58-bit compare into a flop)
            eq_stride_stream_q <= (a_line_next[PF_STRIDE] == a_line_next[PF_STREAM]);
            eq_stride_sms_q    <= (a_line_next[PF_STRIDE] == a_line_next[PF_SMS]);
            eq_stream_sms_q    <= (a_line_next[PF_STREAM] == a_line_next[PF_SMS]);
        end
    end // always stage_a_proc

    // ======================================================================
    // STAGE B: per bank the highest-rank request (stride > stream > SMS), duplicates dropped, hold limit, output flops
    // ======================================================================
    wire [NPF-1:0] dup;                                    // the request equals one of a higher rank: drop it, 3 bits
    wire [NPF-1:0] cand;                                   // the request may be sent, 3 bits
    assign dup[PF_STRIDE] = 1'b0;
    assign dup[PF_STREAM] = a_busy_q[PF_STRIDE] & eq_stride_stream_q;
    assign dup[PF_SMS]    = (a_busy_q[PF_STRIDE] & eq_stride_sms_q) | (a_busy_q[PF_STREAM] & eq_stream_sms_q);
    assign cand = a_busy_q & ~dup & {NPF{pf_enable}};
    reg  [NB*NPF-1:0]      win;                            // [b*NPF + x]: request x is sent to bank b now, 12 bits
    reg  [NPF-1:0]         taken;                          // request x is sent to its bank now, 3 bits
    reg  [NB-1:0]          req_valid_next;                 // per bank: a hint next cycle, 4 bits
    reg  [NB*LINE_ADDR_W-1:0] req_line_next;               // per bank: its line, 4*58 bits
    reg  [NB-1:0]          req_unique_next;                // per bank: it wants the line unique, 4 bits
    reg  [NPF-1:0]         for_bank;                       // scratch: request x is a candidate for the bank, 3 bits
    integer bb;                                            // loop variable: bank
    integer bx;                                            // loop variable: prefetcher
    always @* begin
        taken = {NPF{1'b0}};
        for (bb = 0; bb < NB; bb = bb + 1) begin
            for (bx = 0; bx < NPF; bx = bx + 1)
                for_bank[bx] = cand[bx] & (a_line_q[bx][LA_BANK_LSB +: BANK_W] == bb) & pf_req_ready[bb];
            // the rank chain: stride first, then stream, then SMS (a priority chain on purpose, rule SH-1)
            win[bb*NPF + PF_STRIDE] = for_bank[PF_STRIDE];
            win[bb*NPF + PF_STREAM] = for_bank[PF_STREAM] & ~for_bank[PF_STRIDE];
            win[bb*NPF + PF_SMS]    = for_bank[PF_SMS] & ~for_bank[PF_STREAM] & ~for_bank[PF_STRIDE];
            req_valid_next[bb]  = |win[bb*NPF +: NPF];
            req_unique_next[bb] = |(win[bb*NPF +: NPF] & a_unique_q);
            req_line_next[bb*LINE_ADDR_W +: LINE_ADDR_W] = {LINE_ADDR_W{1'b0}};
            for (bx = 0; bx < NPF; bx = bx + 1) begin
                req_line_next[bb*LINE_ADDR_W +: LINE_ADDR_W] = req_line_next[bb*LINE_ADDR_W +: LINE_ADDR_W] |
                                                               ({LINE_ADDR_W{win[bb*NPF + bx]}} & a_line_q[bx]);
                taken[bx] = taken[bx] | win[bb*NPF + bx];
            end
        end
    end
    wire [NPF-1:0] dropped;                                // the held request is dropped now: duplicate, hold limit or disabled, 3 bits
    assign dropped   = a_busy_q & (dup | a_hold_last_q | {NPF{~pf_enable}});
    assign a_done    = taken | dropped;
    assign src_ready = a_done | {NPF{~pf_enable}};         // disabled: the prefetchers' shown requests are drained too

    always @(posedge clk) begin : stage_b_proc
        if (rst) begin
            pf_req_valid  <= {NB{1'b0}};
            pf_req_unique <= {NB{1'b0}};
        end else begin
            pf_req_valid  <= req_valid_next;
            pf_req_unique <= req_unique_next;
        end
        pf_req_line_addr <= req_line_next;                 // wide data loads every cycle; pf_req_valid qualifies it
    end // always stage_b_proc

// ---- simulation-only trace (compiled only when L1D_DEBUG is defined) ----
`ifdef L1D_DEBUG
    integer dbg_x;                                         // loop variable: prefetcher
    integer dbg_b;                                         // loop variable: bank
    always @(posedge clk) begin
        if (!rst) begin
            for (dbg_x = 0; dbg_x < NPF; dbg_x = dbg_x + 1) begin
                if (a_load[dbg_x])
                    $display("[%0t] L1D PF   %s request line=%h unique=%b captured", $time,
                             (dbg_x == PF_STRIDE) ? "stride" : (dbg_x == PF_STREAM) ? "stream" : "SMS   ",
                             {src_line[dbg_x], {OFFSET_W{1'b0}}}, src_unique[dbg_x]);
                if (dropped[dbg_x])
                    $display("[%0t] L1D PF   %s request line=%h DROPPED (%s)", $time,
                             (dbg_x == PF_STRIDE) ? "stride" : (dbg_x == PF_STREAM) ? "stream" : "SMS   ",
                             {a_line_q[dbg_x], {OFFSET_W{1'b0}}},
                             dup[dbg_x] ? "duplicate" : (a_hold_last_q[dbg_x] ? "hold limit" : "disabled"));
            end
            for (dbg_b = 0; dbg_b < NB; dbg_b = dbg_b + 1)
                if (req_valid_next[dbg_b])
                    $display("[%0t] L1D PF   hint to bank %0d line=%h unique=%b", $time, dbg_b,
                             {req_line_next[dbg_b*LINE_ADDR_W +: LINE_ADDR_W], {OFFSET_W{1'b0}}}, req_unique_next[dbg_b]);
        end
    end
`endif

endmodule

`default_nettype wire
`endif // L1D_PREFETCH_UNIT_V
