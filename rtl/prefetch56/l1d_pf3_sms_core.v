`ifndef L1D_PF3_SMS_CORE_V
`define L1D_PF3_SMS_CORE_V
`default_nettype none
module l1d_pf3_sms_core #(
    parameter NB               = 4,
    parameter PC_W             = 64,
    parameter ADDR_W           = 64,
    parameter PA_W             = 56,
    parameter LINE_BYTES       = 64,
    parameter LINE_W           = 58,
    parameter REGION_BYTES     = 2048,
    parameter FT_ENTRIES       = 16,
    parameter AT_ENTRIES       = 32,
    parameter PHT_SETS         = 32,
    parameter PHT_WAYS         = 4,
    parameter REGION_TAG_W     = 16,
    parameter PC_HASH_W        = 12,
    parameter PHT_TAG_W        = 8,
    parameter TRAIN_FIFO_DEPTH = 4,
    parameter EVT_FIFO_DEPTH   = 4,
    parameter KEY_COPIES       = 8,
    parameter TAG_COPIES       = 4
) (
    input  wire                      clk,
    input  wire                      rst,
    input  wire                      pf_enable,
    input  wire [NB-1:0]             train_valid,
    input  wire [NB-1:0]             train_no_train,
    input  wire [NB*PC_W-1:0]        train_pc,
    input  wire [NB*ADDR_W-1:0]      train_addr,
    input  wire [NB-1:0]             evict_valid,
    input  wire [NB*LINE_W-1:0]      evict_line_addr,
    output wire                      load_valid,
    output wire [PA_W-$clog2(LINE_BYTES)-$clog2(REGION_BYTES/LINE_BYTES)-1:0]
                                     load_region,
    output wire [$clog2(REGION_BYTES/LINE_BYTES)-1:0] load_line,
    output wire [$clog2(REGION_BYTES/LINE_BYTES)-1:0] load_first,
    output wire [NB-1:0]             load_first_bank,
    output wire [$clog2(REGION_BYTES/LINE_BYTES)-1:0] load_second,
    output wire [(REGION_BYTES/LINE_BYTES)-1:0] load_rest
);
    localparam OFFSET_W     = $clog2(LINE_BYTES);
    localparam PLINE_W      = PA_W - OFFSET_W;
    localparam LIR_W        = $clog2(REGION_BYTES / LINE_BYTES);
    localparam REGION_NUM_W = PLINE_W - LIR_W;
    localparam PATTERN_W    = (1 << LIR_W);
    localparam KEY_W        = PC_HASH_W + LIR_W;
    localparam SET_W        = $clog2(PHT_SETS);
    localparam WAY_W        = $clog2(PHT_WAYS);
    localparam FT_IDX_W     = $clog2(FT_ENTRIES);
    localparam AT_IDX_W     = $clog2(AT_ENTRIES);
    localparam PHT_TOTAL    = PHT_SETS * PHT_WAYS;
    localparam PORT_W       = $clog2(NB);
    localparam BANK_W       = $clog2(NB);
    localparam TREC_W = REGION_NUM_W + LIR_W + REGION_TAG_W + SET_W
                        + PHT_TAG_W;
    localparam EREC_W = REGION_NUM_W + LIR_W + REGION_TAG_W;
    localparam KEY_PER      = PHT_TOTAL / KEY_COPIES;
    localparam KEY_CW       = PHT_TAG_W + LIR_W + SET_W;
    localparam TAG_CW       = REGION_TAG_W + LIR_W;
    localparam FT_PER       = FT_ENTRIES / TAG_COPIES;
    localparam AT_PER       = AT_ENTRIES / TAG_COPIES;
    localparam LOH_COPIES   = TAG_COPIES;
    localparam SEL_W        = REGION_NUM_W + LIR_W + REGION_TAG_W
                              + PATTERN_W;
    localparam WP_COPIES    = 3;
    localparam PAT_HALF     = PATTERN_W / 2;
    localparam TE_N         = 4;
    reg  [NB*REGION_TAG_W-1:0] lane_rtag;
    reg  [NB*SET_W-1:0]      lane_set;
    reg  [NB*PHT_TAG_W-1:0]  lane_ptag;
    reg  [NB*REGION_TAG_W-1:0] evl_rtag;
    reg  [KEY_W-1:0]         lane_key;
    reg  [PC_HASH_W-1:0]     pch_one;
    reg  [REGION_NUM_W-1:0]  region_one;
    wire [NB-1:0]            tr_ok;
    wire [NB-1:0]            ev_ok;
    reg  [NB-1:0]            tr_pick;
    reg  [NB-1:0]            ev_pick;
    reg  [NB*NB-1:0]         tr_before_q;
    reg  [NB*NB-1:0]         ev_before_q;
    reg  [NB*NB-1:0]         tr_before_nx;
    reg  [NB*NB-1:0]         ev_before_nx;
    reg  [NB*NB-1:0]         rst_before;
    reg  [NB-1:0]            train_rot_nx;
    reg  [NB-1:0]            evt_rot_nx;
    reg  [PORT_W-1:0]        lane_pos;
    reg  [PORT_W-1:0]        other_pos;
    reg  [TREC_W-1:0]        tr_word;
    reg  [EREC_W-1:0]        ev_word;
    wire                     tr_fresh;
    wire                     ev_fresh;
    reg  [NB-1:0]            train_rot_oh;
    reg  [NB-1:0]            evt_rot_oh;
    reg  [NB*PATTERN_W-1:0]  lane_loh;
    reg  [NB*PATTERN_W-1:0]  evl_loh;
    wire                     tq_vld;
    wire [TREC_W-1:0]        tq_head;
    wire                     eq_vld;
    wire [EREC_W-1:0]        eq_head;
    wire                     tr_avail;
    wire                     ev_avail;
    wire                     take_evt;
    wire                     take_tr;
    wire                     tq_push;
    wire                     eq_push;
    wire                     tq_pop;
    wire                     eq_pop;
    wire [PATTERN_W-1:0]     tq_loh;
    wire [PATTERN_W-1:0]     eq_loh;
    reg  [SEL_W-1:0]         tr_side;
    reg  [SEL_W-1:0]         ev_side;
    wire [SEL_W-1:0]         nx_side;
    wire [TE_N-1:0]          tr_fresh_cp;
    wire [TE_N-1:0]          ev_fresh_cp;
    wire [TE_N-1:0]          take_evt_cp;
    reg  [SET_W-1:0]         nx_set_r;
    reg  [PHT_TAG_W-1:0]     nx_ptag_r;
    reg                      prefer_evt_q;
    reg                      e1_vld;
    reg                      e1_evt;
    reg  [REGION_NUM_W-1:0]  e1_region;
    reg  [LIR_W-1:0]         e1_line;
    reg  [REGION_TAG_W-1:0]  e1_rtag;
    reg  [SET_W-1:0]         e1_set;
    reg  [PHT_TAG_W-1:0]     e1_ptag;
    wire [REGION_NUM_W-1:0]  nx_region;
    wire [LIR_W-1:0]         nx_line;
    wire [REGION_TAG_W-1:0]  nx_rtag;
    wire [SET_W-1:0]         nx_set;
    wire [PHT_TAG_W-1:0]     nx_ptag;
    wire [PATTERN_W-1:0]     nx_line_oh;
    wire [KEY_COPIES*KEY_CW-1:0]    kc_word;
    wire [TAG_COPIES*TAG_CW-1:0]    tc_word;
    wire [LOH_COPIES*PATTERN_W-1:0] lc_oh;
    reg  [FT_ENTRIES-1:0]               ft_vld;
    reg  [FT_ENTRIES*REGION_TAG_W-1:0]  ft_tag;
    reg  [FT_ENTRIES*LIR_W-1:0]         ft_line;
    reg  [FT_ENTRIES*SET_W-1:0]         ft_pset;
    reg  [FT_ENTRIES*PHT_TAG_W-1:0]     ft_ptag;
    reg  [FT_IDX_W-1:0]                 ft_ring;
    reg  [AT_ENTRIES-1:0]               at_vld;
    reg  [AT_ENTRIES*REGION_TAG_W-1:0]  at_tag;
    reg  [AT_ENTRIES*LIR_W-1:0]         at_line;
    reg  [AT_ENTRIES*SET_W-1:0]         at_pset;
    reg  [AT_ENTRIES*PHT_TAG_W-1:0]     at_ptag;
    reg  [AT_ENTRIES*PATTERN_W-1:0]     at_pat;
    reg  [AT_IDX_W-1:0]                 at_ring;
    reg  [PHT_TOTAL-1:0]                pht_vld;
    reg  [PHT_TOTAL*PHT_TAG_W-1:0]      pht_tag;
    reg  [PHT_TOTAL*LIR_W-1:0]          pht_line;
    reg  [PHT_TOTAL*PATTERN_W-1:0]      pht_rest;
    reg  [PHT_TOTAL*LIR_W-1:0]          pht_first;
    reg  [PHT_TOTAL*LIR_W-1:0]          pht_second;
    reg  [PHT_TOTAL*NB-1:0]             pht_fbank;
    reg  [PHT_TOTAL-1:0]                pht_nz;
    reg  [PHT_SETS*WAY_W-1:0]           pht_ptr;
    reg                      dtrig_q;
    reg                      dtrig_ld_q;
    reg                      way_any_q;
    reg                      dprom_q;
    reg                      ddisc_q;
    reg                      dend_q;
    reg                      dtick_q;
    reg                      dfclr_q;
    reg                      dft_none_q;
    reg                      dat_keep_q;
    reg                      dat_none_q;
    reg  [FT_ENTRIES-1:0]    ft_hit_q;
    reg  [FT_ENTRIES-1:0]    ft_vic_q;
    reg  [AT_ENTRIES-1:0]    at_hit_q;
    reg  [AT_ENTRIES-1:0]    at_vic_q;
    reg  [REGION_TAG_W-1:0]  p_rtag;
    reg  [LIR_W-1:0]         p_line;
    reg  [SET_W-1:0]         p_set;
    reg  [PHT_TAG_W-1:0]     p_ptag;
    reg  [PATTERN_W-1:0]     p_line_oh;
    wire [FT_ENTRIES-1:0]    p_ft_set;
    wire [FT_ENTRIES-1:0]    p_ft_clr;
    wire [AT_ENTRIES-1:0]    p_at_set;
    wire [AT_ENTRIES-1:0]    p_at_clr;
    wire [AT_ENTRIES-1:0]    p_at_tick;
    reg  [LIR_W-1:0]         pf_line;
    reg  [SET_W-1:0]         pf_pset;
    reg  [PHT_TAG_W-1:0]     pf_ptag;
    reg  [PATTERN_W-1:0]     pf_line_oh;
    reg  [AT_ENTRIES-1:0]    w_rd_sel;
    reg                      w_rd_vld;
    reg  [LIR_W-1:0]         w_rd_line;
    reg  [SET_W-1:0]         w_rd_pset;
    reg  [PHT_TAG_W-1:0]     w_rd_ptag;
    reg  [PATTERN_W-1:0]     w_rd_pat;
    reg  [PATTERN_W-1:0]     w_rd_line_oh;
    reg  [PATTERN_W-1:0]     w_rd_high;
    reg  [LIR_W-1:0]         p_bit_number;
    reg  [LIR_W-1:0]         w_end_line;
    reg  [LIR_W-1:0]         w_vic_line;
    wire                     same_rtag;
    wire                     same_line;
    wire                     same_pfline;
    wire [FT_ENTRIES-1:0]    ft_vld_e;
    wire [AT_ENTRIES-1:0]    at_vld_e;
    reg  [FT_ENTRIES-1:0]    ft_raw;
    reg  [FT_ENTRIES-1:0]    ft_leq_raw;
    reg  [FT_ENTRIES-1:0]    ft_hit_t;
    reg  [FT_ENTRIES-1:0]    ft_leq_t;
    reg  [FT_ENTRIES-1:0]    ft_hit_c;
    reg  [AT_ENTRIES-1:0]    at_raw;
    reg  [AT_ENTRIES-1:0]    at_has_raw;
    reg  [AT_ENTRIES-1:0]    at_hit_p;
    reg  [AT_ENTRIES-1:0]    at_has_p;
    reg  [AT_ENTRIES-1:0]    at_hit_e;
    reg  [AT_ENTRIES-1:0]    at_has_k;
    wire [FT_ENTRIES-1:0]    ft_hit;
    wire [AT_ENTRIES-1:0]    at_hit;
    wire                     any_ft;
    wire                     any_at;
    wire                     ft_same_line;
    wire                     at_rd_has;
    reg  [FT_ENTRIES-1:0]    ft_vic;
    reg  [AT_ENTRIES-1:0]    at_vic;
    wire [FT_ENTRIES-1:0]    ft_free_one;
    wire [AT_ENTRIES-1:0]    at_free_one;
    wire                     ft_free_seen;
    wire                     at_free_seen;
    wire [FT_ENTRIES-1:0]    ft_free_t;
    wire [FT_ENTRIES-1:0]    ft_free_c;
    wire [FT_ENTRIES-1:0]    ft_free_o;
    wire                     ft_seen_t;
    wire                     ft_seen_c;
    wire                     ft_seen_o;
    wire [AT_ENTRIES-1:0]    at_free_p;
    wire [AT_ENTRIES-1:0]    at_free_e;
    wire [AT_ENTRIES-1:0]    at_free_o;
    wire                     at_seen_p;
    wire                     at_seen_e;
    wire                     at_seen_o;
    wire                     do_promote;
    wire                     do_trigger;
    wire                     do_end_gen;
    wire                     do_ft_discard;
    wire [FT_ENTRIES-1:0]    ft_same_v;
    wire                     at_or_same;
    wire                     end_none;
    wire                     dft_none_nx;
    wire                     dat_keep_nx;
    wire                     dat_none_nx;
    wire                     dtick_nx;
    reg  [PHT_SETS-1:0]      e1_set1h;
    reg  [PHT_TOTAL-1:0]     pht_match;
    reg  [PHT_WAYS-1:0]      way_hit;
    reg  [PHT_WAYS*LIR_W-1:0] way_first;
    reg  [PHT_WAYS*LIR_W-1:0] way_second;
    reg  [PHT_WAYS*NB-1:0]   way_fbank;
    reg  [PHT_WAYS*PATTERN_W-1:0] way_rest;
    reg  [PHT_WAYS-1:0]      way_hit_q;
    reg  [PHT_WAYS*LIR_W-1:0] way_first_q;
    reg  [PHT_WAYS*LIR_W-1:0] way_second_q;
    reg  [PHT_WAYS*NB-1:0]   way_fbank_q;
    reg  [PHT_WAYS*PATTERN_W-1:0] way_rest_q;
    reg  [REGION_NUM_W-1:0]  e2_region;
    reg  [LIR_W-1:0]         e2_line;
    reg  [LIR_W-1:0]         s3_first;
    reg  [LIR_W-1:0]         s3_second;
    reg  [NB-1:0]            s3_fbank;
    reg  [PATTERN_W-1:0]     s3_rest;
    reg                      w_vld;
    reg  [SET_W-1:0]         w_set;
    reg  [PHT_TAG_W-1:0]     w_ptag;
    reg  [LIR_W-1:0]         w_line;
    reg  [PATTERN_W-1:0]     w_pat;
    reg  [PHT_SETS-1:0]      w_set1h;
    reg  [PHT_TOTAL-1:0]     w_tmatch;
    reg  [PATTERN_W-1:0]     w_line_oh;
    reg  [PATTERN_W-1:0]     w_high;
    reg  [PATTERN_W-1:0]     w_all;
    reg  [PATTERN_W-1:0]     w_above;
    wire [PATTERN_W-1:0]     w_ffs_above;
    wire [PATTERN_W-1:0]     w_ffs2_above;
    wire [PATTERN_W-1:0]     w_below;
    wire [PATTERN_W-1:0]     w_ffs_below;
    wire [PATTERN_W-1:0]     w_ffs2_below;
    reg  [PATTERN_W-1:0]     w_second_oh;
    reg  [PATTERN_W-1:0]     w_first_oh;
    wire                     w_seen_a;
    wire                     w_two_a;
    wire                     w_seen_b;
    reg  [LIR_W-1:0]         w1_bit_number;
    reg  [BANK_W-1:0]        w1_bank_idx;
    reg  [LIR_W-1:0]         w2_bit_number;
    reg                      x_vld;
    reg  [PHT_SETS-1:0]      x_set_go;
    reg  [PHT_TAG_W-1:0]     x_ptag;
    reg  [LIR_W-1:0]         x_line;
    reg  [PATTERN_W-1:0]     x_rest;
    reg  [PATTERN_W-1:0]     x_first_oh;
    reg  [PATTERN_W-1:0]     x_second_oh;
    reg  [LIR_W-1:0]         x_first;
    reg  [NB-1:0]            x_fbank;
    reg                      x_nz;
    reg  [PHT_TOTAL-1:0]     x_tmatch;
    wire [WP_COPIES*PHT_TOTAL-1:0] x_write_cp;
    wire [PHT_TOTAL-1:0]     x_write;
    wire [PHT_SETS-1:0]      x_adv;
    reg  [LIR_W-1:0]         x_second;
    always @* begin : hash_proc
        integer lane_idx;
        integer bit_idx;
        lane_rtag = {(NB*REGION_TAG_W){1'b0}};
        lane_set  = {(NB*SET_W){1'b0}};
        lane_ptag = {(NB*PHT_TAG_W){1'b0}};
        evl_rtag  = {(NB*REGION_TAG_W){1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            pch_one = {PC_HASH_W{1'b0}};
            for (bit_idx = 0; bit_idx < PC_W; bit_idx = bit_idx + 1) begin
                pch_one[bit_idx % PC_HASH_W] = pch_one[bit_idx % PC_HASH_W]
                    ^ train_pc[lane_idx*PC_W + bit_idx];
            end
            region_one = train_addr[lane_idx*ADDR_W + OFFSET_W + LIR_W +: REGION_NUM_W];
            for (bit_idx = 0; bit_idx < REGION_NUM_W; bit_idx = bit_idx + 1) begin
                lane_rtag[lane_idx*REGION_TAG_W + (bit_idx % REGION_TAG_W)] =
                    lane_rtag[lane_idx*REGION_TAG_W + (bit_idx % REGION_TAG_W)]
                    ^ region_one[bit_idx];
            end
            lane_key = {pch_one, train_addr[lane_idx*ADDR_W + OFFSET_W +: LIR_W]};
            for (bit_idx = 0; bit_idx < KEY_W; bit_idx = bit_idx + 1) begin
                lane_set[lane_idx*SET_W + (bit_idx % SET_W)] =
                    lane_set[lane_idx*SET_W + (bit_idx % SET_W)]
                    ^ lane_key[bit_idx];
                lane_ptag[lane_idx*PHT_TAG_W + ((bit_idx + 3) % PHT_TAG_W)] =
                    lane_ptag[lane_idx*PHT_TAG_W + ((bit_idx + 3) % PHT_TAG_W)]
                    ^ lane_key[bit_idx];
            end
            region_one = evict_line_addr[lane_idx*LINE_W + LIR_W +: REGION_NUM_W];
            for (bit_idx = 0; bit_idx < REGION_NUM_W; bit_idx = bit_idx + 1) begin
                evl_rtag[lane_idx*REGION_TAG_W + (bit_idx % REGION_TAG_W)] =
                    evl_rtag[lane_idx*REGION_TAG_W + (bit_idx % REGION_TAG_W)]
                    ^ region_one[bit_idx];
            end
        end
    end
    assign tr_ok    = train_valid & ~train_no_train;
    assign ev_ok    = evict_valid;
    assign tr_fresh = (|tr_ok) & pf_enable;
    assign ev_fresh = (|ev_ok) & pf_enable;
    always @* begin : pick_proc
        integer lane_idx;
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            tr_pick[lane_idx] = tr_ok[lane_idx]
                & ~(|(tr_ok & tr_before_q[lane_idx*NB +: NB]));
            ev_pick[lane_idx] = ev_ok[lane_idx]
                & ~(|(ev_ok & ev_before_q[lane_idx*NB +: NB]));
        end
        tr_word = {TREC_W{1'b0}};
        ev_word = {EREC_W{1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_loh[lane_idx*PATTERN_W +: PATTERN_W] =
                {{(PATTERN_W-1){1'b0}}, 1'b1}
                << train_addr[lane_idx*ADDR_W + OFFSET_W +: LIR_W];
            evl_loh[lane_idx*PATTERN_W +: PATTERN_W] =
                {{(PATTERN_W-1){1'b0}}, 1'b1}
                << evict_line_addr[lane_idx*LINE_W +: LIR_W];
            tr_word = tr_word | ({TREC_W{tr_pick[lane_idx]}} & {
                train_addr[lane_idx*ADDR_W + OFFSET_W + LIR_W +: REGION_NUM_W],
                train_addr[lane_idx*ADDR_W + OFFSET_W +: LIR_W],
                lane_rtag[lane_idx*REGION_TAG_W +: REGION_TAG_W],
                lane_set[lane_idx*SET_W +: SET_W],
                lane_ptag[lane_idx*PHT_TAG_W +: PHT_TAG_W]});
            ev_word = ev_word | ({EREC_W{ev_pick[lane_idx]}} & {
                evict_line_addr[lane_idx*LINE_W + LIR_W +: REGION_NUM_W],
                evict_line_addr[lane_idx*LINE_W +: LIR_W],
                evl_rtag[lane_idx*REGION_TAG_W +: REGION_TAG_W]});
        end
    end
    always @* begin : before_next_proc
        integer lane_idx;
        integer other_idx;
        integer start_idx;
        train_rot_nx = {train_rot_oh[NB-2:0], train_rot_oh[NB-1]};
        evt_rot_nx   = {evt_rot_oh[NB-2:0], evt_rot_oh[NB-1]};
        tr_before_nx = {(NB*NB){1'b0}};
        ev_before_nx = {(NB*NB){1'b0}};
        rst_before   = {(NB*NB){1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            for (other_idx = 0; other_idx < NB; other_idx = other_idx + 1) begin
                for (start_idx = 0; start_idx < NB; start_idx = start_idx + 1) begin
                    lane_pos  = lane_idx[PORT_W-1:0] - start_idx[PORT_W-1:0];
                    other_pos = other_idx[PORT_W-1:0] - start_idx[PORT_W-1:0];
                    if (other_pos < lane_pos) begin
                        tr_before_nx[lane_idx*NB + other_idx] =
                            tr_before_nx[lane_idx*NB + other_idx]
                            | train_rot_nx[start_idx];
                        ev_before_nx[lane_idx*NB + other_idx] =
                            ev_before_nx[lane_idx*NB + other_idx]
                            | evt_rot_nx[start_idx];
                        if (start_idx == 0) begin
                            rst_before[lane_idx*NB + other_idx] = 1'b1;
                        end
                    end
                end
            end
        end
    end
    always @(posedge clk) begin : rotation_proc
        if (rst) begin
            train_rot_oh <= {{(NB-1){1'b0}}, 1'b1};
            evt_rot_oh   <= {{(NB-1){1'b0}}, 1'b1};
            tr_before_q  <= rst_before;
            ev_before_q  <= rst_before;
        end
        else begin
            train_rot_oh <= train_rot_nx;
            evt_rot_oh   <= evt_rot_nx;
            tr_before_q  <= tr_before_nx;
            ev_before_q  <= ev_before_nx;
        end
    end
    assign tr_avail = tq_vld | tr_fresh;
    assign ev_avail = eq_vld | ev_fresh;
    assign take_evt = take_evt_cp[TE_N-1];
    assign take_tr  = tr_avail & ~take_evt;
    assign tq_pop   = take_tr & tq_vld;
    assign eq_pop   = take_evt & eq_vld;
    assign tq_push  = tr_fresh & ~(take_tr & ~tq_vld);
    assign eq_push  = ev_fresh & ~(take_evt & ~eq_vld);
    l1d_pf3_fifo #(
        .WIDTH (TREC_W),
        .DEPTH (TRAIN_FIFO_DEPTH)
    ) u_train_fifo (
        .clk        (clk),
        .rst        (rst),
        .stage_data (tr_word),
        .push_valid (tq_push),
        .pop_en     (tq_pop),
        .head_valid (tq_vld),
        .head_data  (tq_head)
    );
    l1d_pf3_fifo #(
        .WIDTH (EREC_W),
        .DEPTH (EVT_FIFO_DEPTH)
    ) u_evict_fifo (
        .clk        (clk),
        .rst        (rst),
        .stage_data (ev_word),
        .push_valid (eq_push),
        .pop_en     (eq_pop),
        .head_valid (eq_vld),
        .head_data  (eq_head)
    );
    assign tq_loh = {{(PATTERN_W-1){1'b0}}, 1'b1}
                    << tq_head[REGION_TAG_W+SET_W+PHT_TAG_W +: LIR_W];
    assign eq_loh = {{(PATTERN_W-1){1'b0}}, 1'b1}
                    << eq_head[REGION_TAG_W +: LIR_W];
    always @* begin : s1_select_proc
        integer lane_idx;
        tr_side = {SEL_W{1'b0}};
        ev_side = {SEL_W{1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            tr_side = tr_side | ({SEL_W{tr_pick[lane_idx]}} & {
                train_addr[lane_idx*ADDR_W + OFFSET_W + LIR_W +: REGION_NUM_W],
                train_addr[lane_idx*ADDR_W + OFFSET_W +: LIR_W],
                lane_rtag[lane_idx*REGION_TAG_W +: REGION_TAG_W],
                lane_loh[lane_idx*PATTERN_W +: PATTERN_W]});
            ev_side = ev_side | ({SEL_W{ev_pick[lane_idx]}} & {
                evict_line_addr[lane_idx*LINE_W + LIR_W +: REGION_NUM_W],
                evict_line_addr[lane_idx*LINE_W +: LIR_W],
                evl_rtag[lane_idx*REGION_TAG_W +: REGION_TAG_W],
                evl_loh[lane_idx*PATTERN_W +: PATTERN_W]});
        end
        if (tq_vld) begin
            tr_side = {tq_head[TREC_W-1 -: REGION_NUM_W],
                       tq_head[REGION_TAG_W+SET_W+PHT_TAG_W +: LIR_W],
                       tq_head[SET_W+PHT_TAG_W +: REGION_TAG_W],
                       tq_loh};
        end
        if (eq_vld) begin
            ev_side = {eq_head[EREC_W-1 -: REGION_NUM_W],
                       eq_head[REGION_TAG_W +: LIR_W],
                       eq_head[REGION_TAG_W-1:0],
                       eq_loh};
        end
        nx_set_r  = {SET_W{tq_vld}} & tq_head[PHT_TAG_W +: SET_W];
        nx_ptag_r = {PHT_TAG_W{tq_vld}} & tq_head[PHT_TAG_W-1:0];
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            nx_set_r = nx_set_r | ({SET_W{~tq_vld & tr_pick[lane_idx]}}
                       & lane_set[lane_idx*SET_W +: SET_W]);
            nx_ptag_r = nx_ptag_r | ({PHT_TAG_W{~tq_vld & tr_pick[lane_idx]}}
                       & lane_ptag[lane_idx*PHT_TAG_W +: PHT_TAG_W]);
        end
    end
    genvar te_g;
    generate
        for (te_g = 0; te_g < TE_N; te_g = te_g + 1) begin : g_take
            l1d_pf3_or_rows #(.W (1), .N (NB)) u_tr_or (
                .rows (tr_ok), .or_out (tr_fresh_cp[te_g])
            );
            l1d_pf3_or_rows #(.W (1), .N (NB)) u_ev_or (
                .rows (ev_ok), .or_out (ev_fresh_cp[te_g])
            );
            assign take_evt_cp[te_g] = (eq_vld | (ev_fresh_cp[te_g] & pf_enable))
                & (~(tq_vld | (tr_fresh_cp[te_g] & pf_enable)) | prefer_evt_q);
        end
    endgenerate
    l1d_pf3_mux2 #(.W (REGION_NUM_W)) u_sel_region (
        .sel (take_evt_cp[0]),
        .in1 (ev_side[SEL_W-1 -: REGION_NUM_W]),
        .in0 (tr_side[SEL_W-1 -: REGION_NUM_W]),
        .out (nx_side[SEL_W-1 -: REGION_NUM_W])
    );
    l1d_pf3_mux2 #(.W (LIR_W + REGION_TAG_W)) u_sel_tag (
        .sel (take_evt_cp[1]),
        .in1 (ev_side[PATTERN_W +: LIR_W + REGION_TAG_W]),
        .in0 (tr_side[PATTERN_W +: LIR_W + REGION_TAG_W]),
        .out (nx_side[PATTERN_W +: LIR_W + REGION_TAG_W])
    );
    l1d_pf3_mux2 #(.W (PATTERN_W)) u_sel_loh (
        .sel (take_evt_cp[2]),
        .in1 (ev_side[PATTERN_W-1:0]),
        .in0 (tr_side[PATTERN_W-1:0]),
        .out (nx_side[PATTERN_W-1:0])
    );
    assign nx_region  = nx_side[SEL_W-1 -: REGION_NUM_W];
    assign nx_line    = nx_side[PATTERN_W + REGION_TAG_W +: LIR_W];
    assign nx_rtag    = nx_side[PATTERN_W +: REGION_TAG_W];
    assign nx_set     = nx_set_r;
    assign nx_ptag    = nx_ptag_r;
    assign nx_line_oh = nx_side[PATTERN_W-1:0];
    always @(posedge clk) begin : s1_reg_proc
        if (rst) begin
            prefer_evt_q <= 1'b0;
            e1_vld       <= 1'b0;
        end
        else begin
            if (tr_avail && ev_avail) begin
                prefer_evt_q <= ~prefer_evt_q;
            end
            e1_vld <= (take_tr | take_evt) & pf_enable;
        end
        e1_evt    <= take_evt;
        e1_region <= nx_region;
        e1_line   <= nx_line;
        e1_rtag   <= nx_rtag;
        e1_set    <= nx_set;
        e1_ptag   <= nx_ptag;
    end
    genvar copy_g;
    generate
        for (copy_g = 0; copy_g < KEY_COPIES; copy_g = copy_g + 1) begin : g_key_copy
            l1d_pf3_rep #(.W (KEY_CW)) u_key (
                .clk (clk), .rst (rst), .en (1'b1),
                .d   ({nx_ptag, nx_line, nx_set}),
                .q   (kc_word[copy_g*KEY_CW +: KEY_CW])
            );
        end
        for (copy_g = 0; copy_g < TAG_COPIES; copy_g = copy_g + 1) begin : g_tag_copy
            l1d_pf3_rep #(.W (TAG_CW)) u_tag (
                .clk (clk), .rst (rst), .en (1'b1),
                .d   ({nx_rtag, nx_line}),
                .q   (tc_word[copy_g*TAG_CW +: TAG_CW])
            );
            l1d_pf3_rep #(.W (PATTERN_W)) u_line_oh (
                .clk (clk), .rst (rst), .en (1'b1),
                .d   (nx_line_oh),
                .q   (lc_oh[copy_g*PATTERN_W +: PATTERN_W])
            );
        end
    endgenerate
    assign p_ft_set  = ft_vic_q & {FT_ENTRIES{dtrig_q}};
    assign p_ft_clr  = ft_hit_q & {FT_ENTRIES{dprom_q | ddisc_q}};
    assign p_at_set  = at_vic_q & {AT_ENTRIES{dprom_q}};
    assign p_at_clr  = at_hit_q & {AT_ENTRIES{dend_q}};
    assign p_at_tick = at_hit_q & {AT_ENTRIES{dtick_q}};
    always @* begin : p_read_proc
        integer ent_idx;
        pf_line = {LIR_W{1'b0}};
        pf_line_oh = {PATTERN_W{1'b0}};
        pf_pset = {SET_W{1'b0}};
        pf_ptag = {PHT_TAG_W{1'b0}};
        for (ent_idx = 0; ent_idx < FT_ENTRIES; ent_idx = ent_idx + 1) begin
            pf_line = pf_line | ({LIR_W{ft_hit_q[ent_idx]}}
                      & ft_line[ent_idx*LIR_W +: LIR_W]);
            pf_line_oh = pf_line_oh | ({PATTERN_W{ft_hit_q[ent_idx]}}
                      & ({{(PATTERN_W-1){1'b0}}, 1'b1}
                         << ft_line[ent_idx*LIR_W +: LIR_W]));
            pf_pset = pf_pset | ({SET_W{ft_hit_q[ent_idx]}}
                      & ft_pset[ent_idx*SET_W +: SET_W]);
            pf_ptag = pf_ptag | ({PHT_TAG_W{ft_hit_q[ent_idx]}}
                      & ft_ptag[ent_idx*PHT_TAG_W +: PHT_TAG_W]);
        end
        w_rd_sel  = dend_q ? at_hit_q : at_vic_q;
        w_rd_vld  = 1'b0;
        w_rd_line = {LIR_W{1'b0}};
        w_rd_pset = {SET_W{1'b0}};
        w_rd_ptag = {PHT_TAG_W{1'b0}};
        w_rd_pat  = {PATTERN_W{1'b0}};
        w_end_line = {LIR_W{1'b0}};
        w_vic_line = {LIR_W{1'b0}};
        for (ent_idx = 0; ent_idx < AT_ENTRIES; ent_idx = ent_idx + 1) begin
            w_rd_vld  = w_rd_vld | (w_rd_sel[ent_idx] & at_vld[ent_idx]);
            w_end_line = w_end_line | ({LIR_W{at_hit_q[ent_idx]}}
                         & at_line[ent_idx*LIR_W +: LIR_W]);
            w_vic_line = w_vic_line | ({LIR_W{at_vic_q[ent_idx]}}
                         & at_line[ent_idx*LIR_W +: LIR_W]);
            w_rd_pset = w_rd_pset | ({SET_W{w_rd_sel[ent_idx]}}
                        & at_pset[ent_idx*SET_W +: SET_W]);
            w_rd_ptag = w_rd_ptag | ({PHT_TAG_W{w_rd_sel[ent_idx]}}
                        & at_ptag[ent_idx*PHT_TAG_W +: PHT_TAG_W]);
            w_rd_pat  = w_rd_pat | ({PATTERN_W{w_rd_sel[ent_idx]}}
                        & at_pat[ent_idx*PATTERN_W +: PATTERN_W]);
        end
        w_rd_line = dend_q ? w_end_line : w_vic_line;
        for (ent_idx = 0; ent_idx < PATTERN_W; ent_idx = ent_idx + 1) begin
            p_bit_number          = ent_idx[LIR_W-1:0];
            w_rd_line_oh[ent_idx] = dend_q ? (p_bit_number == w_end_line)
                                           : (p_bit_number == w_vic_line);
            w_rd_high[ent_idx]    = dend_q ? (p_bit_number > w_end_line)
                                           : (p_bit_number > w_vic_line);
        end
    end
    assign same_rtag   = (p_rtag == e1_rtag);
    assign same_line   = (p_line == e1_line);
    assign same_pfline = (pf_line == e1_line);
    assign ft_vld_e    = (ft_vld & ~p_ft_clr) | p_ft_set;
    assign at_vld_e    = (at_vld & ~p_at_clr) | p_at_set;
    always @* begin : s2_cam_proc
        integer ent_idx;
        integer copy_idx;
        for (ent_idx = 0; ent_idx < FT_ENTRIES; ent_idx = ent_idx + 1) begin
            copy_idx = ent_idx / FT_PER;
            ft_raw[ent_idx] = ft_vld[ent_idx]
                & (ft_tag[ent_idx*REGION_TAG_W +: REGION_TAG_W]
                   == tc_word[copy_idx*TAG_CW + LIR_W +: REGION_TAG_W]);
            ft_leq_raw[ent_idx] = (ft_line[ent_idx*LIR_W +: LIR_W]
                   == tc_word[copy_idx*TAG_CW +: LIR_W]);
            ft_hit_t[ent_idx] = ft_vic_q[ent_idx] ? same_rtag
                                                  : ft_raw[ent_idx];
            ft_leq_t[ent_idx] = ft_vic_q[ent_idx] ? same_line
                                                  : ft_leq_raw[ent_idx];
            ft_hit_c[ent_idx] = ft_raw[ent_idx] & ~ft_hit_q[ent_idx];
        end
        for (ent_idx = 0; ent_idx < AT_ENTRIES; ent_idx = ent_idx + 1) begin
            copy_idx = ent_idx / AT_PER;
            at_raw[ent_idx] = at_vld[ent_idx]
                & (at_tag[ent_idx*REGION_TAG_W +: REGION_TAG_W]
                   == tc_word[copy_idx*TAG_CW + LIR_W +: REGION_TAG_W]);
            at_has_raw[ent_idx] = |(at_pat[ent_idx*PATTERN_W +: PATTERN_W]
                                    & lc_oh[copy_idx*PATTERN_W +: PATTERN_W]);
            at_hit_p[ent_idx] = at_vic_q[ent_idx] ? same_rtag
                                                  : at_raw[ent_idx];
            at_has_p[ent_idx] = at_vic_q[ent_idx] ? (same_pfline | same_line)
                                                  : at_has_raw[ent_idx];
            at_hit_e[ent_idx] = at_raw[ent_idx] & ~at_hit_q[ent_idx];
            at_has_k[ent_idx] = at_has_raw[ent_idx]
                                | (at_hit_q[ent_idx] & same_line);
        end
        for (ent_idx = 0; ent_idx < PHT_SETS; ent_idx = ent_idx + 1) begin
            copy_idx = (ent_idx * PHT_WAYS) / KEY_PER;
            e1_set1h[ent_idx] = (kc_word[copy_idx*KEY_CW +: SET_W]
                                 == ent_idx[SET_W-1:0]);
        end
        for (ent_idx = 0; ent_idx < PHT_TOTAL; ent_idx = ent_idx + 1) begin
            copy_idx = ent_idx / KEY_PER;
            pht_match[ent_idx] = pht_vld[ent_idx] & pht_nz[ent_idx]
                & (pht_tag[ent_idx*PHT_TAG_W +: PHT_TAG_W]
                   == kc_word[copy_idx*KEY_CW + LIR_W + SET_W +: PHT_TAG_W])
                & (pht_line[ent_idx*LIR_W +: LIR_W]
                   == kc_word[copy_idx*KEY_CW + SET_W +: LIR_W]);
        end
    end
    assign ft_hit = (ft_hit_t & {FT_ENTRIES{dtrig_q}})
                  | (ft_hit_c & {FT_ENTRIES{dfclr_q}})
                  | (ft_raw   & {FT_ENTRIES{dft_none_q}});
    assign at_hit = (at_hit_p & {AT_ENTRIES{dprom_q}})
                  | (at_hit_e & {AT_ENTRIES{dend_q}})
                  | (at_raw   & {AT_ENTRIES{dat_keep_q}});
    assign any_ft = (dtrig_q & (|ft_hit_t)) | (dfclr_q & (|ft_hit_c))
                  | (dft_none_q & (|ft_raw));
    assign any_at = (dprom_q & (|at_hit_p)) | (dend_q & (|at_hit_e))
                  | (dat_keep_q & (|at_raw));
    assign ft_same_line = (dtrig_q & (|(ft_hit_t & ft_leq_t)))
                        | (dfclr_q & (|(ft_hit_c & ft_leq_raw)))
                        | (dft_none_q & (|(ft_raw & ft_leq_raw)));
    assign at_rd_has = (dprom_q & (|(at_hit_p & at_has_p)))
                     | (dend_q & (|(at_hit_e & at_has_raw)))
                     | (dtick_q & (|(at_raw & at_has_k)))
                     | (dat_none_q & (|(at_raw & at_has_raw)));
    always @* begin : s2_way_proc
        integer set_idx;
        integer way_idx;
        integer ent_idx;
        way_hit    = {PHT_WAYS{1'b0}};
        way_first  = {(PHT_WAYS*LIR_W){1'b0}};
        way_second = {(PHT_WAYS*LIR_W){1'b0}};
        way_fbank  = {(PHT_WAYS*NB){1'b0}};
        way_rest   = {(PHT_WAYS*PATTERN_W){1'b0}};
        for (set_idx = 0; set_idx < PHT_SETS; set_idx = set_idx + 1) begin
            for (way_idx = 0; way_idx < PHT_WAYS; way_idx = way_idx + 1) begin
                ent_idx = set_idx * PHT_WAYS + way_idx;
                way_hit[way_idx] = way_hit[way_idx]
                    | (e1_set1h[set_idx] & pht_match[ent_idx]);
                way_first[way_idx*LIR_W +: LIR_W] =
                    way_first[way_idx*LIR_W +: LIR_W]
                    | ({LIR_W{e1_set1h[set_idx]}}
                       & pht_first[ent_idx*LIR_W +: LIR_W]);
                way_second[way_idx*LIR_W +: LIR_W] =
                    way_second[way_idx*LIR_W +: LIR_W]
                    | ({LIR_W{e1_set1h[set_idx]}}
                       & pht_second[ent_idx*LIR_W +: LIR_W]);
                way_fbank[way_idx*NB +: NB] = way_fbank[way_idx*NB +: NB]
                    | ({NB{e1_set1h[set_idx]}} & pht_fbank[ent_idx*NB +: NB]);
                way_rest[way_idx*PATTERN_W +: PATTERN_W] =
                    way_rest[way_idx*PATTERN_W +: PATTERN_W]
                    | ({PATTERN_W{e1_set1h[set_idx]}}
                       & pht_rest[ent_idx*PATTERN_W +: PATTERN_W]);
            end
        end
    end
    l1d_pf3_ffs #(.W (FT_ENTRIES)) u_ft_free_t (
        .in_vec (~(ft_vld | ft_vic_q)), .low_oh (ft_free_t), .low2_oh (),
        .any_set (ft_seen_t), .two_set ()
    );
    l1d_pf3_ffs #(.W (FT_ENTRIES)) u_ft_free_c (
        .in_vec (~(ft_vld & ~ft_hit_q)), .low_oh (ft_free_c), .low2_oh (),
        .any_set (ft_seen_c), .two_set ()
    );
    l1d_pf3_ffs #(.W (FT_ENTRIES)) u_ft_free_o (
        .in_vec (~ft_vld), .low_oh (ft_free_o), .low2_oh (),
        .any_set (ft_seen_o), .two_set ()
    );
    l1d_pf3_ffs #(.W (AT_ENTRIES)) u_at_free_p (
        .in_vec (~(at_vld | at_vic_q)), .low_oh (at_free_p), .low2_oh (),
        .any_set (at_seen_p), .two_set ()
    );
    l1d_pf3_ffs #(.W (AT_ENTRIES)) u_at_free_e (
        .in_vec (~(at_vld & ~at_hit_q)), .low_oh (at_free_e), .low2_oh (),
        .any_set (at_seen_e), .two_set ()
    );
    l1d_pf3_ffs #(.W (AT_ENTRIES)) u_at_free_o (
        .in_vec (~at_vld), .low_oh (at_free_o), .low2_oh (),
        .any_set (at_seen_o), .two_set ()
    );
    assign ft_free_one  = (ft_free_t & {FT_ENTRIES{dtrig_q}})
                        | (ft_free_c & {FT_ENTRIES{dfclr_q}})
                        | (ft_free_o & {FT_ENTRIES{dft_none_q}});
    assign ft_free_seen = (dtrig_q & ft_seen_t) | (dfclr_q & ft_seen_c)
                        | (dft_none_q & ft_seen_o);
    assign at_free_one  = (at_free_p & {AT_ENTRIES{dprom_q}})
                        | (at_free_e & {AT_ENTRIES{dend_q}})
                        | (at_free_o & {AT_ENTRIES{dat_keep_q}});
    assign at_free_seen = (dprom_q & at_seen_p) | (dend_q & at_seen_e)
                        | (dat_keep_q & at_seen_o);
    always @* begin : s2_victim_proc
        ft_vic = ft_free_one;
        at_vic = at_free_one;
        if (!ft_free_seen) begin
            ft_vic[ft_ring] = 1'b1;
        end
        if (!at_free_seen) begin
            at_vic[at_ring] = 1'b1;
        end
    end
    assign do_promote    = e1_vld & ~e1_evt & ~any_at & any_ft & ~ft_same_line;
    assign do_trigger    = e1_vld & ~e1_evt & ~any_at & ~any_ft;
    assign do_end_gen    = e1_vld & e1_evt & any_at & at_rd_has;
    assign do_ft_discard = e1_vld & e1_evt & ~any_at & any_ft;
    assign ft_same_v   = (ft_hit_t & ft_leq_t   & {FT_ENTRIES{dtrig_q}})
                       | (ft_hit_c & ft_leq_raw & {FT_ENTRIES{dfclr_q}})
                       | (ft_raw   & ft_leq_raw & {FT_ENTRIES{dft_none_q}});
    assign at_or_same  = |{at_hit, ft_same_v};
    assign end_none    = ~any_at | ~at_rd_has;
    assign dft_none_nx = ~e1_vld | (e1_evt ? (any_at | ~any_ft) : at_or_same);
    assign dat_keep_nx = ~e1_vld | (e1_evt ? end_none : (~any_ft | at_or_same));
    assign dat_none_nx = ~e1_vld
                       | (e1_evt ? end_none : (~any_at & (~any_ft | ft_same_line)));
    assign dtick_nx    = e1_vld & ~e1_evt & any_at;
    always @* begin : s3_proc
        integer way_idx;
        s3_first  = {LIR_W{1'b0}};
        s3_second = {LIR_W{1'b0}};
        s3_fbank  = {NB{1'b0}};
        s3_rest   = {PATTERN_W{1'b0}};
        for (way_idx = 0; way_idx < PHT_WAYS; way_idx = way_idx + 1) begin
            s3_first  = s3_first | ({LIR_W{way_hit_q[way_idx]}}
                        & way_first_q[way_idx*LIR_W +: LIR_W]);
            s3_second = s3_second | ({LIR_W{way_hit_q[way_idx]}}
                        & way_second_q[way_idx*LIR_W +: LIR_W]);
            s3_fbank  = s3_fbank | ({NB{way_hit_q[way_idx]}}
                        & way_fbank_q[way_idx*NB +: NB]);
            s3_rest   = s3_rest | ({PATTERN_W{way_hit_q[way_idx]}}
                        & way_rest_q[way_idx*PATTERN_W +: PATTERN_W]);
        end
    end
    assign load_valid      = dtrig_ld_q & way_any_q;
    assign load_region     = e2_region;
    assign load_line       = e2_line;
    assign load_first      = s3_first;
    assign load_first_bank = s3_fbank;
    assign load_second     = s3_second;
    assign load_rest       = s3_rest;
    always @(posedge clk) begin : table_proc
        integer ent_idx;
        if (rst) begin
            ft_vld  <= {FT_ENTRIES{1'b0}};
            at_vld  <= {AT_ENTRIES{1'b0}};
            ft_ring <= {FT_IDX_W{1'b0}};
            at_ring <= {AT_IDX_W{1'b0}};
            dtrig_q <= 1'b0;
            dtrig_ld_q <= 1'b0;
            dprom_q <= 1'b0;
            ddisc_q <= 1'b0;
            dend_q  <= 1'b0;
            dtick_q <= 1'b0;
            dfclr_q    <= 1'b0;
            dft_none_q <= 1'b1;
            dat_keep_q <= 1'b1;
            dat_none_q <= 1'b1;
            w_vld   <= 1'b0;
        end
        else begin
            if (e1_vld && !e1_evt && !ft_free_seen) begin
                ft_ring <= ft_ring + 1'b1;
            end
            if (e1_vld && !e1_evt && !at_free_seen) begin
                at_ring <= at_ring + 1'b1;
            end
            dtrig_q <= do_trigger;
            dtrig_ld_q <= do_trigger;
            dprom_q <= do_promote;
            ddisc_q <= do_ft_discard;
            dend_q  <= do_end_gen;
            dtick_q <= dtick_nx;
            dfclr_q    <= do_promote | do_ft_discard;
            dft_none_q <= dft_none_nx;
            dat_keep_q <= dat_keep_nx;
            dat_none_q <= dat_none_nx;
            ft_vld <= ft_vld_e;
            at_vld <= at_vld_e;
            w_vld  <= dend_q | (dprom_q & w_rd_vld);
        end
        ft_hit_q  <= ft_hit;
        ft_vic_q  <= ft_vic;
        at_hit_q  <= at_hit;
        at_vic_q  <= at_vic;
        p_rtag    <= e1_rtag;
        p_line    <= e1_line;
        p_set     <= e1_set;
        p_ptag    <= e1_ptag;
        p_line_oh <= lc_oh[PATTERN_W-1:0];
        way_hit_q    <= way_hit;
        way_any_q    <= |way_hit;
        way_first_q  <= way_first;
        way_second_q <= way_second;
        way_fbank_q  <= way_fbank;
        way_rest_q   <= way_rest;
        e2_region    <= e1_region;
        e2_line      <= e1_line;
        for (ent_idx = 0; ent_idx < FT_ENTRIES; ent_idx = ent_idx + 1) begin
            if (p_ft_set[ent_idx]) begin
                ft_tag[ent_idx*REGION_TAG_W +: REGION_TAG_W] <= p_rtag;
                ft_line[ent_idx*LIR_W +: LIR_W]              <= p_line;
                ft_pset[ent_idx*SET_W +: SET_W]              <= p_set;
                ft_ptag[ent_idx*PHT_TAG_W +: PHT_TAG_W]      <= p_ptag;
            end
        end
        for (ent_idx = 0; ent_idx < AT_ENTRIES; ent_idx = ent_idx + 1) begin
            if (p_at_set[ent_idx]) begin
                at_tag[ent_idx*REGION_TAG_W +: REGION_TAG_W] <= p_rtag;
                at_line[ent_idx*LIR_W +: LIR_W]              <= pf_line;
                at_pset[ent_idx*SET_W +: SET_W]              <= pf_pset;
                at_ptag[ent_idx*PHT_TAG_W +: PHT_TAG_W]      <= pf_ptag;
                at_pat[ent_idx*PATTERN_W +: PATTERN_W]       <= pf_line_oh | p_line_oh;
            end
            else if (p_at_tick[ent_idx]) begin
                at_pat[ent_idx*PATTERN_W +: PATTERN_W] <=
                    at_pat[ent_idx*PATTERN_W +: PATTERN_W] | p_line_oh;
            end
        end
        w_set  <= w_rd_pset;
        w_ptag <= w_rd_ptag;
        w_line    <= w_rd_line;
        w_line_oh <= w_rd_line_oh;
        w_high    <= w_rd_high;
        w_pat     <= w_rd_pat;
    end
    always @* begin : w1_match_proc
        integer ent_idx;
        for (ent_idx = 0; ent_idx < PHT_SETS; ent_idx = ent_idx + 1) begin
            w_set1h[ent_idx] = (w_set == ent_idx[SET_W-1:0]);
        end
        for (ent_idx = 0; ent_idx < PHT_TOTAL; ent_idx = ent_idx + 1) begin
            if (x_vld && x_write[ent_idx]) begin
                w_tmatch[ent_idx] = (x_ptag == w_ptag);
            end
            else begin
                w_tmatch[ent_idx] = pht_vld[ent_idx]
                    & (pht_tag[ent_idx*PHT_TAG_W +: PHT_TAG_W] == w_ptag);
            end
        end
    end
    always @* begin : w1_mask_proc
        w_all   = w_pat & ~w_line_oh;
        w_above = w_all & w_high;
    end
    assign w_below = w_all & ~w_high;
    l1d_pf3_ffs #(.W (PATTERN_W)) u_w_above (
        .in_vec (w_above), .low_oh (w_ffs_above), .low2_oh (w_ffs2_above),
        .any_set (w_seen_a), .two_set (w_two_a)
    );
    l1d_pf3_ffs #(.W (PATTERN_W)) u_w_below (
        .in_vec (w_below), .low_oh (w_ffs_below), .low2_oh (w_ffs2_below),
        .any_set (w_seen_b), .two_set ()
    );
    always @* begin : w1_first_proc
        w_first_oh  = w_seen_a ? w_ffs_above : w_ffs_below;
        w_second_oh = w_two_a  ? w_ffs2_above
                    : (w_seen_a ? w_ffs_below : w_ffs2_below);
    end
    always @(posedge clk) begin : w1_reg_proc
        if (rst) begin
            x_vld    <= 1'b0;
            x_set_go <= {PHT_SETS{1'b0}};
        end
        else begin
            x_vld    <= w_vld;
            x_set_go <= w_set1h & {PHT_SETS{w_vld}};
        end
        x_ptag   <= w_ptag;
        x_line   <= w_line;
        x_rest   <= w_all & ~w_first_oh;
        x_first_oh  <= w_first_oh;
        x_second_oh <= w_second_oh;
        x_nz     <= w_seen_a | w_seen_b;
        x_tmatch <= w_tmatch;
    end
    always @* begin : w2_encode_proc
        integer bit_idx;
        x_first  = {LIR_W{1'b0}};
        x_second = {LIR_W{1'b0}};
        for (bit_idx = 0; bit_idx < PATTERN_W; bit_idx = bit_idx + 1) begin
            w1_bit_number = bit_idx[LIR_W-1:0];
            x_first  = x_first | ({LIR_W{x_first_oh[bit_idx]}} & w1_bit_number);
            w2_bit_number = bit_idx[LIR_W-1:0];
            x_second = x_second | ({LIR_W{x_second_oh[bit_idx]}} & w2_bit_number);
        end
        x_fbank = {NB{1'b0}};
        for (bit_idx = 0; bit_idx < PATTERN_W; bit_idx = bit_idx + 1) begin
            w1_bank_idx = bit_idx[BANK_W-1:0];
            x_fbank[w1_bank_idx] = x_fbank[w1_bank_idx] | x_first_oh[bit_idx];
        end
    end
    genvar set_g;
    genvar wcp_g;
    generate
        for (set_g = 0; set_g < PHT_SETS; set_g = set_g + 1) begin : g_way_set
            for (wcp_g = 0; wcp_g < WP_COPIES; wcp_g = wcp_g + 1) begin : g_way_copy
                if (wcp_g == 0) begin : g_main
                    l1d_pf3_sms_way_pick #(.WAYS (PHT_WAYS)) u_pick (
                        .set_go    (x_set_go[set_g]),
                        .tag_match (x_tmatch[set_g*PHT_WAYS +: PHT_WAYS]),
                        .way_valid (pht_vld[set_g*PHT_WAYS +: PHT_WAYS]),
                        .ring_way  (pht_ptr[set_g*WAY_W +: WAY_W]),
                        .way_write (x_write_cp[set_g*PHT_WAYS +: PHT_WAYS]),
                        .ring_adv  (x_adv[set_g])
                    );
                end
                else begin : g_extra
                    l1d_pf3_sms_way_pick #(.WAYS (PHT_WAYS)) u_pick (
                        .set_go    (x_set_go[set_g]),
                        .tag_match (x_tmatch[set_g*PHT_WAYS +: PHT_WAYS]),
                        .way_valid (pht_vld[set_g*PHT_WAYS +: PHT_WAYS]),
                        .ring_way  (pht_ptr[set_g*WAY_W +: WAY_W]),
                        .way_write (x_write_cp[wcp_g*PHT_TOTAL
                                               + set_g*PHT_WAYS +: PHT_WAYS]),
                        .ring_adv  ()
                    );
                end
            end
        end
    endgenerate
    assign x_write = x_write_cp[PHT_TOTAL-1:0];
    always @(posedge clk) begin : pht_proc
        integer ent_idx;
        integer set_idx;
        if (rst) begin
            pht_vld <= {PHT_TOTAL{1'b0}};
            pht_ptr <= {(PHT_SETS*WAY_W){1'b0}};
        end
        else begin
            for (ent_idx = 0; ent_idx < PHT_TOTAL; ent_idx = ent_idx + 1) begin
                if (x_write[ent_idx]) begin
                    pht_vld[ent_idx] <= 1'b1;
                end
            end
            for (set_idx = 0; set_idx < PHT_SETS; set_idx = set_idx + 1) begin
                if (x_adv[set_idx]) begin
                    pht_ptr[set_idx*WAY_W +: WAY_W] <=
                        pht_ptr[set_idx*WAY_W +: WAY_W] + 1'b1;
                end
            end
        end
        for (ent_idx = 0; ent_idx < PHT_TOTAL; ent_idx = ent_idx + 1) begin
            if (x_write[ent_idx]) begin
                pht_tag[ent_idx*PHT_TAG_W +: PHT_TAG_W]  <= x_ptag;
                pht_line[ent_idx*LIR_W +: LIR_W]         <= x_line;
                pht_first[ent_idx*LIR_W +: LIR_W]        <= x_first;
                pht_second[ent_idx*LIR_W +: LIR_W]       <= x_second;
                pht_fbank[ent_idx*NB +: NB]              <= x_fbank;
                pht_nz[ent_idx]                          <= x_nz;
            end
        end
        for (ent_idx = 0; ent_idx < PHT_TOTAL; ent_idx = ent_idx + 1) begin
            if (x_write_cp[PHT_TOTAL + ent_idx]) begin
                pht_rest[ent_idx*PATTERN_W +: PAT_HALF] <= x_rest[PAT_HALF-1:0];
            end
            if (x_write_cp[2*PHT_TOTAL + ent_idx]) begin
                pht_rest[ent_idx*PATTERN_W + PAT_HALF +: PATTERN_W - PAT_HALF]
                    <= x_rest[PATTERN_W-1:PAT_HALF];
            end
        end
    end
`ifdef L1D_DEBUG
    always @(posedge clk) begin : debug_proc
        integer set_idx;
        if (!rst && x_vld) begin
            for (set_idx = 0; set_idx < PHT_SETS; set_idx = set_idx + 1) begin
                if (x_set_go[set_idx]
                    && ((x_tmatch[set_idx*PHT_WAYS +: PHT_WAYS]
                         & (x_tmatch[set_idx*PHT_WAYS +: PHT_WAYS] - 1'b1))
                        != {PHT_WAYS{1'b0}})) begin
                    $display("[%0t] L1D PF3_SMS ERROR two PHT ways hold one tag (A5)", $time);
                end
            end
        end
        if (!rst && e1_vld) begin
            if ((ft_hit & (ft_hit - 1'b1)) != {FT_ENTRIES{1'b0}}) begin
                $display("[%0t] L1D PF3_SMS ERROR two FT entries", $time);
            end
            if ((at_hit & (at_hit - 1'b1)) != {AT_ENTRIES{1'b0}}) begin
                $display("[%0t] L1D PF3_SMS ERROR two AT entries", $time);
            end
            if (any_ft && any_at) begin
                $display("[%0t] L1D PF3_SMS ERROR region in FT and AT", $time);
            end
            if ((way_hit & (way_hit - 1'b1)) != {PHT_WAYS{1'b0}}) begin
                $display("[%0t] L1D PF3_SMS ERROR two PHT ways", $time);
            end
        end
    end
`endif
endmodule
`default_nettype wire
`endif
