// ==========================================================================
// l1d_pf3_sms_expand.v - SMS expander, 3-cycle version: footprint -> one
//                        line request per cycle
// ==========================================================================
// IN SIMPLE WORDS : when the core finds a learned footprint for a new
//   region it hands over the region, the first line to fetch (nearest
//   after the trigger) and the other lines. The first line becomes the
//   candidate at once (direct) when the expander is idle; the other lines
//   wait in a slot (up to EXPAND_SLOTS footprints) and come out one by
//   one, nearest after the trigger first, then wrapping round the region
//   (the same order as l1d_pf_sms_expand). If the expander is busy the
//   first line waits in the slot too ("first pending") and still comes
//   out first. Bank credits are checked by l1d_pf3_rank.
//
// STAGES (one clock each):
//   S2  scan: the slot picked one cycle before gives its next line (kept
//       ready in the slot as a one-hot, so this is a 4:1 select) -> scan
//       offer flops; direct allowed? -> flop
//   S3  offer (cand_*, from flops): the scan offer, else the direct first
//       line of the core's footprint (load_*, from the core's flops);
//       slot load (the rest of the footprint)
//   X0  scan pick (round robin over the slots), one cycle ahead, flops;
//       each slot finds its next line again in the background (from its
//       own flops) when it gives one.
//
// INSTANTIATED BY : l1d_prefetch_unit, instance u_sms_expand, 1 copy.
//
// PARAMETERS:
//   name          default unit   meaning
//   NB            4       banks  cache banks (bank = line bits [1:0])
//   LINE_W        58      bits   cache line address
//   REGION_NUM_W  45      bits   region number
//   LIR_W         5       bits   line in region
//   EXPAND_SLOTS  4       slots  footprints in progress (>= 2)
//
// PORTS (default widths):
//   clk, rst, pf_enable   in  1     clock, reset, on / off (off empties)
//   load_valid            in  1     S3: a footprint for a trigger
//   load_region           in  45    its region
//   load_line             in  5     trigger line
//   load_first            in  5     first line to fetch
//   load_first_bank       in  4     its bank, one-hot
//   load_second           in  5     the line after the first
//   load_rest             in  32    the other lines
//   held                  in  1     rank still holds the SMS offer
//   cand_valid/_line/_bank out 1/58/4  S3 offer (2:1 select of flops)
//   cand_valid_dp         out 1     the same valid from its own offer flop
//                                   copy, for the rank's wide data selects
//                                   (fanout split, T-6)
//   cand_line_scan        out 58    the scan offer's line (flops)
//   cand_line_direct      out 58    the direct offer's line (the load)
//   cand_pick_scan        out 1     1 = cand_line is cand_line_scan (own
//                                   offer flop copy): the rank compares
//                                   both lines and picks last
//
// WORKED EXAMPLE (region R, trigger 0, footprint {3, 6}, idle):
//   S3: load_first 3 -> offer R:3 (direct), the slot gets lines {6} with
//   next line 6 (load_second); the scan picks the slot and offers R:6
//   when the place is free; nothing is left, the slot is freed.
//
// LOGIC LEVELS (T-9, estimate): S3 offer: load_valid 2 (core) + AND 1 +
//   select 1; slot load: slot_vld set 4, data written by a free slot
//   every cycle (select = the slot_vld flop, 1); slot pending / next
//   line built for both direct_go values, direct_go picks last (1); scan: 4:1 select 2 +
//   encode 2 ->
//   flop; slot next line: second-in-order straight from the flops,
//   32-bit first / second set 6 + 3:1 select 2 = 8 (no mask level);
//   X0 pick 4.
//
// ASSUMPTIONS (V-9.2): A1 EXPAND_SLOTS >= 2. A3 a slot holds the lines
//   still to send; its next line is the first of them in the order
//   (nearest above the trigger, then from line 0, the same as
//   l1d_pf_sms_expand; checked under L1D_DEBUG). A2 a footprint that finds no
//   free slot keeps only its direct first line (the slot part is dropped:
//   a hint, as A4 of l1d_pf_sms_expand).
// ==========================================================================
`ifndef L1D_PF3_SMS_EXPAND_V
`define L1D_PF3_SMS_EXPAND_V
`default_nettype none

module l1d_pf3_sms_expand #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter NB           = 4,  // default 4 banks
    parameter LINE_W       = 58, // default 58 bits: cache line address
    parameter REGION_NUM_W = 45, // default 45 bits: region number
    parameter LIR_W        = 5,  // default 5 bits: line in region
    parameter EXPAND_SLOTS = 4   // default 4 slots
) (
    // ---- clock and reset ----
    input  wire                      clk,             // clock
    input  wire                      rst,             // sync reset, high
    input  wire                      pf_enable,       // 1 = on
    // ---- footprint from the core (S3, from the core's flops) ----
    input  wire                      load_valid,      // footprint
    input  wire [REGION_NUM_W-1:0]   load_region,     // 45 bits
    input  wire [LIR_W-1:0]          load_line,       // 5 bits
    input  wire [LIR_W-1:0]          load_first,      // 5 bits
    input  wire [NB-1:0]             load_first_bank, // 4: one-hot
    input  wire [LIR_W-1:0]          load_second,     // 5 bits
    input  wire [(1<<LIR_W)-1:0]     load_rest,       // 32 bits
    // ---- rank ----
    input  wire                      held,            // offer still held
    // ---- offer (S3) ----
    output wire                      cand_valid,      // offer
    output wire [LINE_W-1:0]         cand_line,       // 58 bits
    output reg  [NB-1:0]             cand_bank,       // 4: one-hot
    output wire                      cand_valid_dp,   // offer, data copy
    output wire [LINE_W-1:0]         cand_line_scan,  // 58 bits
    output wire [LINE_W-1:0]         cand_line_direct,// 58 bits
    output wire                      cand_pick_scan   // 1 = scan line
);

    // ---- local constants (value at the default) ----
    localparam PATTERN_W = (1 << LIR_W);              // 32 bits
    localparam NS        = EXPAND_SLOTS;              // 4 slots
    localparam PAD_W     = LINE_W - REGION_NUM_W - LIR_W; // 8 bits
    localparam BANK_W    = $clog2(NB);                // 2 bits
    // copies of the scan offer flop (fanout split, T-6): 0 = cand_valid
    // (rank control, slot_free), 1 = cand_line select, 2 = cand_bank
    // select, 3 = cand_valid_dp (rank line / hold selects), 4 =
    // cand_pick_scan (the rank's compare selects)
    localparam SO_N      = 5;                         // 5 copies

    // ---- slots ----
    reg  [NS-1:0]              slot_vld;     // 4: in use
    reg  [NS*REGION_NUM_W-1:0] slot_region;  // 4 x 45
    reg  [NS*PATTERN_W-1:0]    slot_pend;    // 4 x 32: lines still to send
    reg  [NS*PATTERN_W-1:0]    slot_high;    // 4 x 32: lines > trigger
    reg  [NS*PATTERN_W-1:0]    slot_next;    // 4 x 32: next line, one-hot

    // ---- slot next line, found again in the background ----
    wire [NS*PATTERN_W-1:0]    after_mask;   // 4 x 32: pend without next
    wire [NS*PATTERN_W-1:0]    after_next;   // 4 x 32: next after that
    wire [NS-1:0]              after_empty;  // 4: nothing left after next
`ifdef L1D_DEBUG
    wire [NS*PATTERN_W-1:0]    first_pend;   // 4 x 32: first pending line
                                             //   in the order (A3 check)
`endif

    // ---- load (S3) ----
    reg  [NS-1:0]              free_one;     // 4: slot to load
    reg                        free_seen;    // a free slot exists
    reg  [PATTERN_W-1:0]       load_high;    // 32: lines above the trigger
    reg  [LIR_W-1:0]           bit_number;   // scratch: load bit number
    reg  [LIR_W-1:0]           scan_bit_num; // scratch: scan bit number
    wire [PATTERN_W-1:0]       first_oh;     // 32: first line, one-hot
    wire [PATTERN_W-1:0]       second_oh;    // 32: second line, one-hot
    reg                        direct_ok_q;  // S2 -> S3: direct allowed
    wire                       direct_go;    // the first line goes direct
    wire                       load_slot;    // a slot is loaded

    // ---- scan ----
    reg  [NS-1:0]              rr_mask_q;    // 4: slots after the last
    reg  [NS-1:0]              sel_q;        // 4: picked slot
    reg                        sel_vld_q;    // a slot is picked
    reg  [NS-1:0]              pick_high;    // 4
    reg  [NS-1:0]              pick_base;    // 4
    reg  [NS-1:0]              pick_one;     // 4
    reg  [NS-1:0]              pick_above;   // 4
    reg                        pick_seen;    // scratch
    reg                        rd_vld;       // picked slot in use
    reg  [REGION_NUM_W-1:0]    rd_region;    // 45
    reg  [PATTERN_W-1:0]       rd_next;      // 32: its next line, one-hot
    reg  [LIR_W-1:0]           scan_line;    // 5: the line it asks
    wire                       slot_free;    // the offer place is free
    wire                       scan_take;    // the scan makes an offer
    wire                       scan_ok;      // the scan makes an offer if
                                             //   no direct offer is in S3
    // each slot's next pending / next line for both values of the late
    // direct_go (_dg: a direct offer in S3, _nd: none), picked last
    reg  [NS*PATTERN_W-1:0]    pend_dg;      // 4 x 32
    reg  [NS*PATTERN_W-1:0]    pend_nd;      // 4 x 32
    reg  [NS*PATTERN_W-1:0]    next_dg;      // 4 x 32
    reg  [NS*PATTERN_W-1:0]    next_nd;      // 4 x 32
    wire [NS*PATTERN_W-1:0]    pend_pick;    // 4 x 32: picked by direct_go
    wire [NS*PATTERN_W-1:0]    next_pick;    // 4 x 32: picked by direct_go
    reg                        soffer_q;     // S2 -> S3: scan offer
                                             //   (scan_ok, checks)
    wire [SO_N-1:0]            soffer_cp;    // 5: copies of soffer_q
    reg  [REGION_NUM_W-1:0]    s_region_q;   // 45
    reg  [LIR_W-1:0]           s_line_q;     // 5

    // ======================================================================
    // X0 (background): per slot, the next line after the current one
    // (nearest above the trigger, else the lowest), from the slot's flops
    // ======================================================================
    genvar slot_g; // generate index: slot
    generate
        for (slot_g = 0; slot_g < NS; slot_g = slot_g + 1) begin : g_next
            // order of the lines: above the trigger (ascending), then the
            // rest (ascending). slot_next is the first pending line in that
            // order (A3), so the line after it is the second one: found
            // straight from the pending / above flops (lowest and second
            // lowest of each half in parallel), not after removing the
            // next line first (rule T-9)
            wire [PATTERN_W-1:0] pend_above; // 32: pending, above trigger
            wire [PATTERN_W-1:0] pend_below; // 32: pending, the others
            // low_above is read only by the L1D_DEBUG check (A3)
            /* verilator lint_off UNUSEDSIGNAL */
            wire [PATTERN_W-1:0] low_above;  // 32: lowest above
            /* verilator lint_on UNUSEDSIGNAL */
            wire [PATTERN_W-1:0] low2_above; // 32: second lowest above
            wire [PATTERN_W-1:0] low_below;  // 32: lowest of the others
            wire [PATTERN_W-1:0] low2_below; // 32: second lowest of them
            wire                 any_above;  // a pending line above
            wire                 two_above;  // two or more above
            wire                 any_below;  // a pending line below
            wire                 two_below;  // two or more below
            assign pend_above = slot_pend[slot_g*PATTERN_W +: PATTERN_W]
                                & slot_high[slot_g*PATTERN_W +: PATTERN_W];
            assign pend_below = slot_pend[slot_g*PATTERN_W +: PATTERN_W]
                                & ~slot_high[slot_g*PATTERN_W +: PATTERN_W];
            l1d_pf3_ffs #(.W (PATTERN_W)) u_above (
                .in_vec (pend_above), .low_oh (low_above), .low2_oh (low2_above),
                .any_set (any_above), .two_set (two_above)
            );
            l1d_pf3_ffs #(.W (PATTERN_W)) u_below (
                .in_vec (pend_below), .low_oh (low_below), .low2_oh (low2_below),
                .any_set (any_below), .two_set (two_below)
            );
            assign after_mask[slot_g*PATTERN_W +: PATTERN_W] =
                slot_pend[slot_g*PATTERN_W +: PATTERN_W]
                & ~slot_next[slot_g*PATTERN_W +: PATTERN_W];
            assign after_next[slot_g*PATTERN_W +: PATTERN_W] =
                two_above ? low2_above : (any_above ? low_below : low2_below);
            // no line is left after this one: at most one pending
            assign after_empty[slot_g] = ~(two_above | two_below
                                           | (any_above & any_below));
`ifdef L1D_DEBUG
            // the first pending line in the order, for the A3 check
            assign first_pend[slot_g*PATTERN_W +: PATTERN_W] =
                any_above ? low_above : low_below;
`endif
        end // for slot_g
    endgenerate

    // ======================================================================
    // load (S3): first free slot; the first line goes direct when allowed
    // ======================================================================
    assign direct_go = load_valid & direct_ok_q;
    assign first_oh  = {{(PATTERN_W-1){1'b0}}, 1'b1} << load_first;
    assign second_oh = {{(PATTERN_W-1){1'b0}}, 1'b1} << load_second;

    always @* begin : load_proc
        integer slot_idx; // walks the slots
        integer bit_idx;  // walks the pattern bits
        free_one  = {NS{1'b0}};
        free_seen = 1'b0;
        for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
            if (!slot_vld[slot_idx] && !free_seen) begin
                free_one[slot_idx] = 1'b1;
                free_seen          = 1'b1;
            end
        end // for slot_idx
        for (bit_idx = 0; bit_idx < PATTERN_W; bit_idx = bit_idx + 1) begin
            bit_number         = bit_idx[LIR_W-1:0];
            load_high[bit_idx] = (bit_number > load_line);
        end // for bit_idx
    end // always load_proc
    // a slot is needed for the rest, or for everything when not direct
    // (per slot it is used with free_one, which already means a free slot
    // exists, so free_seen is not on the write-enable path)
    assign load_slot = load_valid & pf_enable
                       & ((|load_rest) | ~direct_ok_q);

    // ======================================================================
    // X0: round-robin pick of a slot that has lines left (one cycle ahead)
    // ======================================================================
    always @* begin : pick_proc
        integer slot_idx; // walks the slots
        pick_high = slot_vld & rr_mask_q;
        pick_base = (|pick_high) ? pick_high : slot_vld;
        pick_one  = {NS{1'b0}};
        pick_seen = 1'b0;
        for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
            if (pick_base[slot_idx] && !pick_seen) begin
                pick_one[slot_idx] = 1'b1;
                pick_seen          = 1'b1;
            end
        end // for slot_idx
        pick_above = {NS{1'b0}};
        pick_seen  = 1'b0;
        for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
            pick_above[slot_idx] = pick_seen;
            pick_seen            = pick_seen | pick_one[slot_idx];
        end // for slot_idx
    end // always pick_proc

    // ======================================================================
    // S2 scan: the picked slot's next line (4:1 select of flops)
    // ======================================================================
    always @* begin : scan_proc
        integer slot_idx; // walks the slots
        integer bit_idx;  // walks the pattern bits
        rd_vld    = 1'b0;
        rd_region = {REGION_NUM_W{1'b0}};
        rd_next   = {PATTERN_W{1'b0}};
        for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
            rd_vld    = rd_vld | (slot_vld[slot_idx] & sel_q[slot_idx]);
            rd_region = rd_region | ({REGION_NUM_W{sel_q[slot_idx]}}
                        & slot_region[slot_idx*REGION_NUM_W +: REGION_NUM_W]);
            rd_next   = rd_next | ({PATTERN_W{sel_q[slot_idx]}}
                        & slot_next[slot_idx*PATTERN_W +: PATTERN_W]);
        end // for slot_idx
        // one-hot -> number
        scan_line = {LIR_W{1'b0}};
        for (bit_idx = 0; bit_idx < PATTERN_W; bit_idx = bit_idx + 1) begin
            scan_bit_num = bit_idx[LIR_W-1:0];
            scan_line  = scan_line | ({LIR_W{rd_next[bit_idx]}} & scan_bit_num);
        end // for bit_idx
    end // always scan_proc

    // a new offer only when the place is free (no offer in S3, none held)
    assign slot_free = ~cand_valid & ~held;
    // the S3 direct offer (direct_go, late: core load_valid) is the last
    // input of scan_take; scan_ok is from flops
    assign scan_ok   = sel_vld_q & rd_vld & ~soffer_q & ~held & pf_enable;
    assign scan_take = scan_ok & ~direct_go;

    // ======================================================================
    // S3: the offer
    // ======================================================================
    assign cand_valid    = soffer_cp[0] | direct_go;
    assign cand_valid_dp = soffer_cp[3] | direct_go;
    assign cand_line  = soffer_cp[1] ? {{PAD_W{1'b0}}, s_region_q, s_line_q}
                                 : {{PAD_W{1'b0}}, load_region, load_first};
    // both possible lines and the select copy, for the rank's compares:
    // cand_line == (cand_pick_scan ? scan : direct) always
    assign cand_line_scan   = {{PAD_W{1'b0}}, s_region_q, s_line_q};
    assign cand_line_direct = {{PAD_W{1'b0}}, load_region, load_first};
    assign cand_pick_scan   = soffer_cp[4];

    always @* begin : bank_proc
        integer bank_idx; // walks the banks
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
            cand_bank[bank_idx] = soffer_cp[2]
                ? (s_line_q[BANK_W-1:0] == bank_idx[BANK_W-1:0])
                : load_first_bank[bank_idx];
        end // for bank_idx
    end // always bank_proc

    // the next pending / next line of each slot, for both direct_go cases:
    //   free slot: takes the load (direct: the rest, second next; else
    //   first line pending, first next); busy slot picked by the scan:
    //   the line goes (only when no direct offer); else unchanged
    always @* begin : slot_next_proc
        integer slot_idx; // walks the slots
        for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
            if (!slot_vld[slot_idx]) begin
                pend_dg[slot_idx*PATTERN_W +: PATTERN_W] = load_rest;
                next_dg[slot_idx*PATTERN_W +: PATTERN_W] = second_oh;
                pend_nd[slot_idx*PATTERN_W +: PATTERN_W] = load_rest | first_oh;
                next_nd[slot_idx*PATTERN_W +: PATTERN_W] = first_oh;
            end
            else begin
                pend_dg[slot_idx*PATTERN_W +: PATTERN_W] =
                    slot_pend[slot_idx*PATTERN_W +: PATTERN_W];
                next_dg[slot_idx*PATTERN_W +: PATTERN_W] =
                    slot_next[slot_idx*PATTERN_W +: PATTERN_W];
                if (scan_ok && sel_q[slot_idx]) begin
                    pend_nd[slot_idx*PATTERN_W +: PATTERN_W] =
                        after_mask[slot_idx*PATTERN_W +: PATTERN_W];
                    next_nd[slot_idx*PATTERN_W +: PATTERN_W] =
                        after_next[slot_idx*PATTERN_W +: PATTERN_W];
                end
                else begin
                    pend_nd[slot_idx*PATTERN_W +: PATTERN_W] =
                        slot_pend[slot_idx*PATTERN_W +: PATTERN_W];
                    next_nd[slot_idx*PATTERN_W +: PATTERN_W] =
                        slot_next[slot_idx*PATTERN_W +: PATTERN_W];
                end // else (not scanned)
            end // else (busy slot)
        end // for slot_idx
    end // always slot_next_proc

    // the direct_go select, one kept-hierarchy 2:1 per slot and field, so
    // synthesis keeps it the last gate (direct_go is the late core bit)
    genvar pick_g; // slot
    generate
        for (pick_g = 0; pick_g < NS; pick_g = pick_g + 1) begin : g_pick
            l1d_pf3_mux2 #(.W (PATTERN_W)) u_pend_sel (
                .sel (direct_go),
                .in1 (pend_dg[pick_g*PATTERN_W +: PATTERN_W]),
                .in0 (pend_nd[pick_g*PATTERN_W +: PATTERN_W]),
                .out (pend_pick[pick_g*PATTERN_W +: PATTERN_W])
            );
            l1d_pf3_mux2 #(.W (PATTERN_W)) u_next_sel (
                .sel (direct_go),
                .in1 (next_dg[pick_g*PATTERN_W +: PATTERN_W]),
                .in0 (next_nd[pick_g*PATTERN_W +: PATTERN_W]),
                .out (next_pick[pick_g*PATTERN_W +: PATTERN_W])
            );
        end // for pick_g
    endgenerate

    // ======================================================================
    // clocked state: slots, scan pick, offer flops
    // ======================================================================
    // the offer flop copies: same reset and data as soffer_q, kept
    // hierarchy so synthesis does not merge them
    l1d_pf3_rep #(.W(SO_N), .HAS_RST(1), .HAS_EN(0)) u_soffer (
        .clk (clk),
        .rst (rst | ~pf_enable),
        .en  (1'b1),
        .d   ({SO_N{scan_take}}),
        .q   (soffer_cp)
    );

    always @(posedge clk) begin : state_proc
        integer slot_idx; // walks the slots
        if (rst || !pf_enable) begin
            slot_vld    <= {NS{1'b0}};
            sel_vld_q   <= 1'b0;
            sel_q       <= {NS{1'b0}};
            rr_mask_q   <= {NS{1'b1}};
            soffer_q    <= 1'b0;
            direct_ok_q <= 1'b0;
        end
        else begin
            // S3: load a footprint into the free slot
            for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
                if (load_slot && free_one[slot_idx]) begin
                    slot_vld[slot_idx] <= 1'b1;
                end
            end // for slot_idx
            // S2: the scan takes one line of its slot; the slot is freed
            // when no line is left after it
            for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
                if (scan_take && sel_q[slot_idx] && after_empty[slot_idx]) begin
                    slot_vld[slot_idx] <= 1'b0;
                end
            end // for slot_idx
            // the pick holds while the offer place is busy
            if (!sel_vld_q || scan_take) begin
                sel_vld_q <= |slot_vld;
                sel_q     <= pick_one;
                if (|slot_vld) begin
                    rr_mask_q <= pick_above;
                end
            end // if (pick)
            soffer_q    <= scan_take;
            // a direct offer in S3 needs the place free now and no scan
            // offer (the scan only runs with a picked slot)
            direct_ok_q <= slot_free & ~sel_vld_q;
        end // else (run)
        s_region_q <= rd_region;
        s_line_q   <= scan_line;
        // slot data (slot_vld guards it). A free slot takes the load values
        // every cycle (early write, rule T-6): only slot_vld needs the late
        // "load this slot" decision; a free slot is never scanned (scan_take
        // needs the picked slot valid), so the select is the slot_vld flop
        for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
            if (!slot_vld[slot_idx]) begin
                slot_region[slot_idx*REGION_NUM_W +: REGION_NUM_W] <= load_region;
                slot_high[slot_idx*PATTERN_W +: PATTERN_W]         <= load_high;
            end
        end // for slot_idx
        // pending / next line: both direct_go cases are ready, direct_go
        // picks last (rule T-9, kept-hierarchy selects below)
        slot_pend <= pend_pick;
        slot_next <= next_pick;
    end // always state_proc

`ifdef L1D_DEBUG
    // ---- watch-dogs (V-9.3), simulation only ----
    always @(posedge clk) begin : debug_proc
        if (!rst && soffer_q && direct_go) begin
            $display("[%0t] L1D PF3_SMS_EXPAND ERROR scan and direct together",
                     $time);
        end
        if (!rst) begin : order_check
            integer slot_idx; // walks the slots
            for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
                if (slot_vld[slot_idx]
                    && (slot_next[slot_idx*PATTERN_W +: PATTERN_W]
                        != first_pend[slot_idx*PATTERN_W +: PATTERN_W])) begin
                    $display("[%0t] L1D PF3_SMS_EXPAND ERROR slot %0d next line is not the first pending (A3)",
                             $time, slot_idx);
                end
            end // for slot_idx
        end // order_check
    end // always debug_proc
`endif

endmodule

`default_nettype wire
`endif // L1D_PF3_SMS_EXPAND_V
