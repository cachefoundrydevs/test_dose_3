`ifndef L1D_PF3_SMS_EXPAND_V
`define L1D_PF3_SMS_EXPAND_V
`default_nettype none
module l1d_pf3_sms_expand #(
    parameter NB           = 4,
    parameter LINE_W       = 58,
    parameter REGION_NUM_W = 45,
    parameter LIR_W        = 5,
    parameter EXPAND_SLOTS = 4
) (
    input  wire                      clk,
    input  wire                      rst,
    input  wire                      pf_enable,
    input  wire                      load_valid,
    input  wire [REGION_NUM_W-1:0]   load_region,
    input  wire [LIR_W-1:0]          load_line,
    input  wire [LIR_W-1:0]          load_first,
    input  wire [NB-1:0]             load_first_bank,
    input  wire [LIR_W-1:0]          load_second,
    input  wire [(1<<LIR_W)-1:0]     load_rest,
    input  wire                      held,
    output wire                      cand_valid,
    output wire [LINE_W-1:0]         cand_line,
    output reg  [NB-1:0]             cand_bank,
    output wire                      cand_valid_dp,
    output wire [LINE_W-1:0]         cand_line_scan,
    output wire [LINE_W-1:0]         cand_line_direct,
    output wire                      cand_pick_scan
);
    localparam PATTERN_W = (1 << LIR_W);
    localparam NS        = EXPAND_SLOTS;
    localparam PAD_W     = LINE_W - REGION_NUM_W - LIR_W;
    localparam BANK_W    = $clog2(NB);
    localparam SO_N      = 5;
    reg  [NS-1:0]              slot_vld;
    reg  [NS*REGION_NUM_W-1:0] slot_region;
    reg  [NS*PATTERN_W-1:0]    slot_pend;
    reg  [NS*PATTERN_W-1:0]    slot_high;
    reg  [NS*PATTERN_W-1:0]    slot_next;
    wire [NS*PATTERN_W-1:0]    after_mask;
    wire [NS*PATTERN_W-1:0]    after_next;
    wire [NS-1:0]              after_empty;
`ifdef L1D_DEBUG
    wire [NS*PATTERN_W-1:0]    first_pend;
`endif
    reg  [NS-1:0]              free_one;
    reg                        free_seen;
    reg  [PATTERN_W-1:0]       load_high;
    reg  [LIR_W-1:0]           bit_number;
    reg  [LIR_W-1:0]           scan_bit_num;
    wire [PATTERN_W-1:0]       first_oh;
    wire [PATTERN_W-1:0]       second_oh;
    reg                        direct_ok_q;
    wire                       direct_go;
    wire                       load_slot;
    reg  [NS-1:0]              rr_mask_q;
    reg  [NS-1:0]              sel_q;
    reg                        sel_vld_q;
    reg  [NS-1:0]              pick_high;
    reg  [NS-1:0]              pick_base;
    reg  [NS-1:0]              pick_one;
    reg  [NS-1:0]              pick_above;
    reg                        pick_seen;
    reg                        rd_vld;
    reg  [REGION_NUM_W-1:0]    rd_region;
    reg  [PATTERN_W-1:0]       rd_next;
    reg  [LIR_W-1:0]           scan_line;
    wire                       slot_free;
    wire                       scan_take;
    wire                       scan_ok;
    reg  [NS*PATTERN_W-1:0]    pend_dg;
    reg  [NS*PATTERN_W-1:0]    pend_nd;
    reg  [NS*PATTERN_W-1:0]    next_dg;
    reg  [NS*PATTERN_W-1:0]    next_nd;
    wire [NS*PATTERN_W-1:0]    pend_pick;
    wire [NS*PATTERN_W-1:0]    next_pick;
    reg                        soffer_q;
    wire [SO_N-1:0]            soffer_cp;
    reg  [REGION_NUM_W-1:0]    s_region_q;
    reg  [LIR_W-1:0]           s_line_q;
    genvar slot_g;
    generate
        for (slot_g = 0; slot_g < NS; slot_g = slot_g + 1) begin : g_next
            wire [PATTERN_W-1:0] pend_above;
            wire [PATTERN_W-1:0] pend_below;
            wire [PATTERN_W-1:0] low_above;
            wire [PATTERN_W-1:0] low2_above;
            wire [PATTERN_W-1:0] low_below;
            wire [PATTERN_W-1:0] low2_below;
            wire                 any_above;
            wire                 two_above;
            wire                 any_below;
            wire                 two_below;
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
            assign after_empty[slot_g] = ~(two_above | two_below
                                           | (any_above & any_below));
`ifdef L1D_DEBUG
            assign first_pend[slot_g*PATTERN_W +: PATTERN_W] =
                any_above ? low_above : low_below;
`endif
        end
    endgenerate
    assign direct_go = load_valid & direct_ok_q;
    assign first_oh  = {{(PATTERN_W-1){1'b0}}, 1'b1} << load_first;
    assign second_oh = {{(PATTERN_W-1){1'b0}}, 1'b1} << load_second;
    always @* begin : load_proc
        integer slot_idx;
        integer bit_idx;
        free_one  = {NS{1'b0}};
        free_seen = 1'b0;
        for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
            if (!slot_vld[slot_idx] && !free_seen) begin
                free_one[slot_idx] = 1'b1;
                free_seen          = 1'b1;
            end
        end
        for (bit_idx = 0; bit_idx < PATTERN_W; bit_idx = bit_idx + 1) begin
            bit_number         = bit_idx[LIR_W-1:0];
            load_high[bit_idx] = (bit_number > load_line);
        end
    end
    assign load_slot = load_valid & pf_enable
                       & ((|load_rest) | ~direct_ok_q);
    always @* begin : pick_proc
        integer slot_idx;
        pick_high = slot_vld & rr_mask_q;
        pick_base = (|pick_high) ? pick_high : slot_vld;
        pick_one  = {NS{1'b0}};
        pick_seen = 1'b0;
        for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
            if (pick_base[slot_idx] && !pick_seen) begin
                pick_one[slot_idx] = 1'b1;
                pick_seen          = 1'b1;
            end
        end
        pick_above = {NS{1'b0}};
        pick_seen  = 1'b0;
        for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
            pick_above[slot_idx] = pick_seen;
            pick_seen            = pick_seen | pick_one[slot_idx];
        end
    end
    always @* begin : scan_proc
        integer slot_idx;
        integer bit_idx;
        rd_vld    = 1'b0;
        rd_region = {REGION_NUM_W{1'b0}};
        rd_next   = {PATTERN_W{1'b0}};
        for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
            rd_vld    = rd_vld | (slot_vld[slot_idx] & sel_q[slot_idx]);
            rd_region = rd_region | ({REGION_NUM_W{sel_q[slot_idx]}}
                        & slot_region[slot_idx*REGION_NUM_W +: REGION_NUM_W]);
            rd_next   = rd_next | ({PATTERN_W{sel_q[slot_idx]}}
                        & slot_next[slot_idx*PATTERN_W +: PATTERN_W]);
        end
        scan_line = {LIR_W{1'b0}};
        for (bit_idx = 0; bit_idx < PATTERN_W; bit_idx = bit_idx + 1) begin
            scan_bit_num = bit_idx[LIR_W-1:0];
            scan_line  = scan_line | ({LIR_W{rd_next[bit_idx]}} & scan_bit_num);
        end
    end
    assign slot_free = ~cand_valid & ~held;
    assign scan_ok   = sel_vld_q & rd_vld & ~soffer_q & ~held & pf_enable;
    assign scan_take = scan_ok & ~direct_go;
    assign cand_valid    = soffer_cp[0] | direct_go;
    assign cand_valid_dp = soffer_cp[3] | direct_go;
    assign cand_line  = soffer_cp[1] ? {{PAD_W{1'b0}}, s_region_q, s_line_q}
                                 : {{PAD_W{1'b0}}, load_region, load_first};
    assign cand_line_scan   = {{PAD_W{1'b0}}, s_region_q, s_line_q};
    assign cand_line_direct = {{PAD_W{1'b0}}, load_region, load_first};
    assign cand_pick_scan   = soffer_cp[4];
    always @* begin : bank_proc
        integer bank_idx;
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
            cand_bank[bank_idx] = soffer_cp[2]
                ? (s_line_q[BANK_W-1:0] == bank_idx[BANK_W-1:0])
                : load_first_bank[bank_idx];
        end
    end
    always @* begin : slot_next_proc
        integer slot_idx;
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
                end
            end
        end
    end
    genvar pick_g;
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
        end
    endgenerate
    l1d_pf3_rep #(.W(SO_N), .HAS_RST(1), .HAS_EN(0)) u_soffer (
        .clk (clk),
        .rst (rst | ~pf_enable),
        .en  (1'b1),
        .d   ({SO_N{scan_take}}),
        .q   (soffer_cp)
    );
    always @(posedge clk) begin : state_proc
        integer slot_idx;
        if (rst || !pf_enable) begin
            slot_vld    <= {NS{1'b0}};
            sel_vld_q   <= 1'b0;
            sel_q       <= {NS{1'b0}};
            rr_mask_q   <= {NS{1'b1}};
            soffer_q    <= 1'b0;
            direct_ok_q <= 1'b0;
        end
        else begin
            for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
                if (load_slot && free_one[slot_idx]) begin
                    slot_vld[slot_idx] <= 1'b1;
                end
            end
            for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
                if (scan_take && sel_q[slot_idx] && after_empty[slot_idx]) begin
                    slot_vld[slot_idx] <= 1'b0;
                end
            end
            if (!sel_vld_q || scan_take) begin
                sel_vld_q <= |slot_vld;
                sel_q     <= pick_one;
                if (|slot_vld) begin
                    rr_mask_q <= pick_above;
                end
            end
            soffer_q    <= scan_take;
            direct_ok_q <= slot_free & ~sel_vld_q;
        end
        s_region_q <= rd_region;
        s_line_q   <= scan_line;
        for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
            if (!slot_vld[slot_idx]) begin
                slot_region[slot_idx*REGION_NUM_W +: REGION_NUM_W] <= load_region;
                slot_high[slot_idx*PATTERN_W +: PATTERN_W]         <= load_high;
            end
        end
        slot_pend <= pend_pick;
        slot_next <= next_pick;
    end
`ifdef L1D_DEBUG
    always @(posedge clk) begin : debug_proc
        if (!rst && soffer_q && direct_go) begin
            $display("[%0t] L1D PF3_SMS_EXPAND ERROR scan and direct together",
                     $time);
        end
        if (!rst) begin : order_check
            integer slot_idx;
            for (slot_idx = 0; slot_idx < NS; slot_idx = slot_idx + 1) begin
                if (slot_vld[slot_idx]
                    && (slot_next[slot_idx*PATTERN_W +: PATTERN_W]
                        != first_pend[slot_idx*PATTERN_W +: PATTERN_W])) begin
                    $display("[%0t] L1D PF3_SMS_EXPAND ERROR slot %0d next line is not the first pending (A3)",
                             $time, slot_idx);
                end
            end
        end
    end
`endif
endmodule
`default_nettype wire
`endif
