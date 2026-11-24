`ifndef L1D_PF3_STRIDE_V
`define L1D_PF3_STRIDE_V
`default_nettype none
module l1d_pf3_stride #(
    parameter NB         = 4,
    parameter PC_W       = 64,
    parameter ADDR_W     = 64,
    parameter PA_W       = 56,
    parameter LINE_BYTES = 64,
    parameter PAGE_BYTES = 4096,
    parameter RPT_SETS   = 16,
    parameter RPT_WAYS   = 4,
    parameter TAG_W      = 12,
    parameter PC_LSB     = 1,
    parameter DIST       = 2,
    parameter FILT_DEPTH = 4,
    parameter PH_W       = 10,
    parameter E1_COPIES  = 16,
    parameter LC_COPIES  = 8,
    parameter PICK_COPIES = 8,
    parameter FOLD_COPIES = 4
) (
    input  wire                      clk,
    input  wire                      rst,
    input  wire                      pf_enable,
    input  wire                      clear,
    input  wire [NB-1:0]             train_valid,
    input  wire [NB-1:0]             train_no_train,
    input  wire [NB-1:0]             train_is_store,
    input  wire [NB*PC_W-1:0]        train_pc,
    input  wire [NB*ADDR_W-1:0]      train_addr,
    output wire                      cand_valid,
    output wire [NB:0]               cand_valid_dp,
    output wire [ADDR_W-$clog2(LINE_BYTES)-1:0] cand_line,
    output reg  [NB-1:0]             cand_bank,
    output wire                      cand_unique
);
    localparam NS       = RPT_SETS;
    localparam NW       = RPT_WAYS;
    localparam ENTRIES  = NS * NW;
    localparam SET_W    = $clog2(NS);
    localparam OFFSET_W = $clog2(LINE_BYTES);
    localparam PAGE_W   = $clog2(PAGE_BYTES);
    localparam LIP_W    = PAGE_W - OFFSET_W;
    localparam PPN_W    = PA_W - PAGE_W;
    localparam PAD_W    = ADDR_W - PA_W;
    localparam LO3_W    = PAGE_W + 2;
    localparam FOLD_LSB = PC_LSB + SET_W;
    localparam FOLD_W   = PC_W - FOLD_LSB;
    localparam BANK_W   = $clog2(NB);
    localparam SPC      = NS / E1_COPIES;
    localparam LC_N     = LC_COPIES;
    localparam CPP      = E1_COPIES / PICK_COPIES;
    localparam CV_N     = LC_N + NB + 3;
    localparam SPL      = NS / LC_N;
    localparam SPG      = 4;
    localparam NG       = NS / SPG;
    localparam SPF      = NS / FOLD_COPIES;
    localparam EV_W     = TAG_W + 1 + PAGE_W + LO3_W + PH_W;
    localparam FL_W     = FILT_DEPTH * LIP_W;
    reg  [NB-1:0]        lane_pick;
    reg                  lane_seen;
    reg  [NB*TAG_W-1:0]  lane_tag;
    reg  [NB*PH_W-1:0]   lane_ph;
    wire [NB*LO3_W-1:0]  lane_lo3;
    reg                  pick_any;
    reg  [SET_W-1:0]     pick_set;
    reg                  pick_st;
    reg  [PH_W-1:0]      pick_ph;
    reg  [PPN_W-1:0]     pick_ppn;
    reg  [NS-1:0]        pick_set_on;
    wire                 pick_take;
    reg  [NB*EV_W-1:0]   lane_event;
    wire [PICK_COPIES*EV_W-1:0] pk_event;
    wire [FOLD_COPIES*NB*TAG_W-1:0] cp_s1_tag;
    wire [FOLD_COPIES*NB*PH_W-1:0] cp_s1_ph;
    reg  [NB-1:0]        lane_ph_ev;
    reg  [NB-1:0]        lane_ph_p;
    reg                  e1_vld;
    reg  [PH_W-1:0]      e1_ph;
    reg                  e1_st;
    reg  [PPN_W-1:0]     e1_ppn;
    wire [E1_COPIES*SPC-1:0] cp_set_on;
    wire [E1_COPIES*EV_W-1:0] cp_event;
    wire [E1_COPIES*EV_W-1:0] cp_pevent;
    wire [E1_COPIES*NB-1:0]  cp_lane;
    wire [E1_COPIES*NB-1:0]  cp_p2_key;
    wire [E1_COPIES*NB-1:0]  cp_ph_ev;
    wire [E1_COPIES*NB-1:0]  cp_ph_p;
    reg  [NB-1:0]            lane_p2_key;
    wire [E1_COPIES-1:0]            cp_a_key;
    wire [E1_COPIES-1:0]            cp_a_correct;
    wire [E1_COPIES*(PAGE_W+1)-1:0] cp_a_stride;
    wire [E1_COPIES*(PAGE_W+1)-1:0] cp_a_exp;
    wire [E1_COPIES*(PAGE_W+3)-1:0] cp_a_s3;
    wire [E1_COPIES*(PAGE_W+3)-1:0] cp_a_tgt;
    wire [E1_COPIES-1:0]            cp_a_nz;
    reg                  e2_vld;
    reg                  e2_st;
    reg  [PH_W-1:0]      e2_ph;
    reg  [PPN_W-1:0]     e2_ppn;
    reg  [NS-1:0]        fire_new_q;
    reg  [NS-1:0]        fire_dup_q;
    wire [CV_N-1:0]      cand_cp;
    wire [LIP_W-1:0]     cand_lip_q;
    wire [NS*LIP_W-1:0]  set_lip_nx;
    wire [NS*NB-1:0]     set_bank_nx;
    reg  [NG*LIP_W-1:0]  grp_lip_q;
    wire [LIP_W-1:0]     out_lip;
    reg  [NG*NB-1:0]     grp_bank_oh_q;
    wire [NB-1:0]        bank_oh;
    wire [LC_N*LIP_W-1:0]      cp_prev_lip;
    wire [ENTRIES-1:0]       ent_hit;
    wire [ENTRIES*3-1:0]     ent_class;
    wire [ENTRIES-1:0]       ent_fire_new;
    wire [ENTRIES-1:0]       ent_fire_dup;
    wire [ENTRIES*LIP_W-1:0] ent_tgt_lip;
    wire [ENTRIES-1:0]       ent_alloc;
    reg  [FILT_DEPTH-1:0]       filt_vld;
    reg  [FILT_DEPTH*PH_W-1:0]  filt_ph;
    wire [FL_W-1:0]             filt_lip_next;
    wire [LC_N*FL_W-1:0]        cp_filt_lip;
    reg                         prev_ins_q;
    always @* begin : lane_fold_proc
        integer lane_idx;
        integer bit_idx;
        lane_tag = {(NB*TAG_W){1'b0}};
        lane_ph  = {(NB*PH_W){1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            for (bit_idx = 0; bit_idx < FOLD_W; bit_idx = bit_idx + 1) begin
                lane_tag[lane_idx*TAG_W + (bit_idx % TAG_W)] =
                    lane_tag[lane_idx*TAG_W + (bit_idx % TAG_W)]
                    ^ train_pc[lane_idx*PC_W + FOLD_LSB + bit_idx];
            end
            for (bit_idx = 0; bit_idx < PPN_W; bit_idx = bit_idx + 1) begin
                lane_ph[lane_idx*PH_W + (bit_idx % PH_W)] =
                    lane_ph[lane_idx*PH_W + (bit_idx % PH_W)]
                    ^ train_addr[lane_idx*ADDR_W + PAGE_W + bit_idx];
            end
        end
    end
    genvar lane_g;
    generate
        for (lane_g = 0; lane_g < NB; lane_g = lane_g + 1) begin : g_lo3
            wire [LO3_W-1:0] lo_x1;
            wire [LO3_W-1:0] lo_x2;
            assign lo_x1 = {2'b00, train_addr[lane_g*ADDR_W +: PAGE_W]};
            assign lo_x2 = {1'b0, train_addr[lane_g*ADDR_W +: PAGE_W], 1'b0};
            l1d_pf3_ks_add #(.W(LO3_W), .SUB(0)) u_lo3 (
                .a   (lo_x1),
                .b   (lo_x2),
                .sum (lane_lo3[lane_g*LO3_W +: LO3_W])
            );
        end
    endgenerate
    always @* begin : lane_pick_proc
        integer lane_idx;
        integer set_idx;
        lane_seen = 1'b0;
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_pick[lane_idx] = train_valid[lane_idx]
                                  & ~train_no_train[lane_idx] & ~lane_seen;
            lane_seen = lane_seen
                        | (train_valid[lane_idx] & ~train_no_train[lane_idx]);
        end
        pick_any = 1'b0;
        pick_set = {SET_W{1'b0}};
        pick_st  = 1'b0;
        pick_ph  = {PH_W{1'b0}};
        pick_ppn = {PPN_W{1'b0}};
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            pick_any = pick_any | lane_pick[lane_idx];
            pick_set = pick_set | ({SET_W{lane_pick[lane_idx]}}
                       & train_pc[lane_idx*PC_W + PC_LSB +: SET_W]);
            pick_st  = pick_st | (lane_pick[lane_idx] & train_is_store[lane_idx]);
            pick_ph  = pick_ph | ({PH_W{lane_pick[lane_idx]}}
                       & lane_ph[lane_idx*PH_W +: PH_W]);
            pick_ppn = pick_ppn | ({PPN_W{lane_pick[lane_idx]}}
                       & train_addr[lane_idx*ADDR_W + PAGE_W +: PPN_W]);
        end
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_event[lane_idx*EV_W +: EV_W] = {EV_W{lane_pick[lane_idx]}} & {
                lane_tag[lane_idx*TAG_W +: TAG_W],
                train_is_store[lane_idx],
                train_addr[lane_idx*ADDR_W +: PAGE_W],
                lane_lo3[lane_idx*LO3_W +: LO3_W],
                lane_ph[lane_idx*PH_W +: PH_W]};
        end
        for (set_idx = 0; set_idx < NS; set_idx = set_idx + 1) begin
            pick_set_on[set_idx] = pick_take & (pick_set == set_idx[SET_W-1:0]);
        end
    end
    assign pick_take  = pick_any & pf_enable & ~clear;
    always @* begin : p2_key_proc
        integer lane_idx;
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_p2_key[lane_idx] =
                (lane_tag[lane_idx*TAG_W +: TAG_W]
                 == cp_pevent[EV_W-1 -: TAG_W])
                & (train_is_store[lane_idx] == cp_pevent[EV_W-1-TAG_W]);
        end
    end
    always @* begin : ph_ev_proc
        integer lane_idx;
        for (lane_idx = 0; lane_idx < NB; lane_idx = lane_idx + 1) begin
            lane_ph_ev[lane_idx] = (lane_ph[lane_idx*PH_W +: PH_W] == e1_ph);
            lane_ph_p[lane_idx]  = (lane_ph[lane_idx*PH_W +: PH_W] == e2_ph);
        end
    end
    genvar fold_g;
    genvar flane_g;
    generate
        for (fold_g = 0; fold_g < FOLD_COPIES; fold_g = fold_g + 1) begin : g_fold
            for (flane_g = 0; flane_g < NB; flane_g = flane_g + 1) begin : g_lane
                l1d_pf3_fold #(.IN_W(FOLD_W), .OUT_W(TAG_W)) u_tag_fold (
                    .din  (train_pc[flane_g*PC_W + FOLD_LSB +: FOLD_W]),
                    .dout (cp_s1_tag[(fold_g*NB + flane_g)*TAG_W +: TAG_W])
                );
                l1d_pf3_fold #(.IN_W(PPN_W), .OUT_W(PH_W)) u_ph_fold (
                    .din  (train_addr[flane_g*ADDR_W + PAGE_W +: PPN_W]),
                    .dout (cp_s1_ph[(fold_g*NB + flane_g)*PH_W +: PH_W])
                );
            end
        end
    endgenerate
    genvar pick_g;
    generate
        for (pick_g = 0; pick_g < PICK_COPIES; pick_g = pick_g + 1) begin : g_pick
            l1d_pf3_or_rows #(.W (EV_W), .N (NB)) u_pick (
                .rows   (lane_event),
                .or_out (pk_event[pick_g*EV_W +: EV_W])
            );
        end
    endgenerate
    always @(posedge clk) begin : s1_reg_proc
        if (rst) begin
            e1_vld <= 1'b0;
        end
        else begin
            e1_vld <= pick_take;
        end
        e1_ph  <= pick_ph;
        e1_st  <= pick_st;
        e1_ppn <= pick_ppn;
    end
    genvar copy_g;
    generate
        for (copy_g = 0; copy_g < E1_COPIES; copy_g = copy_g + 1) begin : g_copy
            l1d_pf3_rep #(.W(SPC), .HAS_RST(1), .HAS_EN(0)) u_set_on (
                .clk (clk),
                .rst (rst),
                .en  (1'b1),
                .d   (pick_set_on[copy_g*SPC +: SPC]),
                .q   (cp_set_on[copy_g*SPC +: SPC])
            );
            l1d_pf3_rep #(.W(EV_W), .HAS_RST(0), .HAS_EN(0)) u_event (
                .clk (clk),
                .rst (1'b0),
                .en  (1'b1),
                .d   (pk_event[(copy_g/CPP)*EV_W +: EV_W]),
                .q   (cp_event[copy_g*EV_W +: EV_W])
            );
            l1d_pf3_rep #(.W(4*NB), .HAS_RST(0), .HAS_EN(0)) u_lane (
                .clk (clk),
                .rst (1'b0),
                .en  (1'b1),
                .d   ({lane_pick, lane_p2_key, lane_ph_ev, lane_ph_p}),
                .q   ({cp_lane[copy_g*NB +: NB], cp_p2_key[copy_g*NB +: NB],
                       cp_ph_ev[copy_g*NB +: NB], cp_ph_p[copy_g*NB +: NB]})
            );
            l1d_pf3_rep #(.W(EV_W), .HAS_RST(0), .HAS_EN(0)) u_pevent (
                .clk (clk),
                .rst (1'b0),
                .en  (1'b1),
                .d   (cp_event[copy_g*EV_W +: EV_W]),
                .q   (cp_pevent[copy_g*EV_W +: EV_W])
            );
            l1d_pf3_stride_anew #(
                .TAG_W  (TAG_W),
                .PAGE_W (PAGE_W),
                .PH_W   (PH_W)
            ) u_anew (
                .ev_tag    (cp_event[copy_g*EV_W + EV_W-1 -: TAG_W]),
                .ev_st     (cp_event[copy_g*EV_W + EV_W-1-TAG_W]),
                .ev_lo     (cp_event[copy_g*EV_W + LO3_W+PH_W +: PAGE_W]),
                .ev_lo3    (cp_event[copy_g*EV_W + PH_W +: LO3_W]),
                .ev_ph     (cp_event[copy_g*EV_W +: PH_W]),
                .p_tag     (cp_pevent[copy_g*EV_W + EV_W-1 -: TAG_W]),
                .p_st      (cp_pevent[copy_g*EV_W + EV_W-1-TAG_W]),
                .p_lo      (cp_pevent[copy_g*EV_W + LO3_W+PH_W +: PAGE_W]),
                .p_lo3     (cp_pevent[copy_g*EV_W + PH_W +: LO3_W]),
                .p_ph      (cp_pevent[copy_g*EV_W +: PH_W]),
                .a_key     (cp_a_key[copy_g]),
                .a_correct (cp_a_correct[copy_g]),
                .a_stride  (cp_a_stride[copy_g*(PAGE_W+1) +: PAGE_W+1]),
                .a_exp     (cp_a_exp[copy_g*(PAGE_W+1) +: PAGE_W+1]),
                .a_s3      (cp_a_s3[copy_g*(PAGE_W+3) +: PAGE_W+3]),
                .a_tgt     (cp_a_tgt[copy_g*(PAGE_W+3) +: PAGE_W+3]),
                .a_nz      (cp_a_nz[copy_g])
            );
        end
    endgenerate
    genvar lcopy_g;
    generate
        for (lcopy_g = 0; lcopy_g < LC_N; lcopy_g = lcopy_g + 1) begin : g_lcopy
            l1d_pf3_or_rows #(.W(LIP_W), .N(NG)) u_prev_lip (
                .rows   (grp_lip_q),
                .or_out (cp_prev_lip[lcopy_g*LIP_W +: LIP_W])
            );
            l1d_pf3_rep #(.W(FL_W), .HAS_RST(0), .HAS_EN(1)) u_filt_lip (
                .clk (clk),
                .rst (1'b0),
                .en  (cand_cp[NB + 3 + lcopy_g]),
                .d   (filt_lip_next),
                .q   (cp_filt_lip[lcopy_g*FL_W +: FL_W])
            );
        end
    endgenerate
    genvar set_g;
    generate
        for (set_g = 0; set_g < NS; set_g = set_g + 1) begin : g_set
            wire                 set_on;
            wire [TAG_W-1:0]     ev_tag;
            wire                 ev_st;
            wire [PAGE_W-1:0]    ev_lo;
            wire [LO3_W-1:0]     ev_lo3;
            wire [PH_W-1:0]      ev_ph;
            wire [TAG_W-1:0]     p_tag;
            wire                 p_st;
            wire [PAGE_W-1:0]    p_lo;
            wire [LO3_W-1:0]     p_lo3;
            wire [PH_W-1:0]      p_ph;
            wire [FILT_DEPTH-1:0] filt_pm;
            wire                 prev_pm;
            wire                 set_miss;
            reg  [NW*3-1:0]      way_class;
            reg  [NW-1:0]        victim_way;
            reg                  way_beaten;
            genvar way_g;
            genvar slot_g;
            assign set_on = cp_set_on[(set_g / SPC)*SPC + (set_g % SPC)];
            assign {ev_tag, ev_st, ev_lo, ev_lo3, ev_ph} =
                   cp_event[(set_g / SPC)*EV_W +: EV_W];
            assign {p_tag, p_st, p_lo, p_lo3, p_ph} =
                   cp_pevent[(set_g / SPC)*EV_W +: EV_W];
            for (slot_g = 0; slot_g < FILT_DEPTH; slot_g = slot_g + 1) begin : g_pm
                assign filt_pm[slot_g] = filt_vld[slot_g]
                                         & (filt_ph[slot_g*PH_W +: PH_W] == ev_ph);
            end
            assign prev_pm = e2_vld & (e2_ph == ev_ph);
            for (way_g = 0; way_g < NW; way_g = way_g + 1) begin : g_way
                assign ent_alloc[(set_g * NW + way_g)] = set_on & set_miss & victim_way[way_g];
                l1d_pf3_stride_entry #(
                    .TAG_W      (TAG_W),
                    .PAGE_W     (PAGE_W),
                    .OFFSET_W   (OFFSET_W),
                    .PH_W       (PH_W),
                    .FILT_DEPTH (FILT_DEPTH),
                    .NL         (NB)
                ) u_entry (
                    .clk        (clk),
                    .rst        (rst),
                    .clear      (clear),
                    .ev_set_on  (set_on),
                    .s1_tag     (cp_s1_tag[(set_g/SPF)*NB*TAG_W +: NB*TAG_W]),
                    .s1_st      (train_is_store),
                    .ev_lane    (cp_lane[(set_g / SPC)*NB +: NB]),
                    .ev_p2_key  (cp_p2_key[(set_g / SPC)*NB +: NB]),
                    .s1_ph      (cp_s1_ph[(set_g/SPF)*NB*PH_W +: NB*PH_W]),
                    .ev_ph_ev   (cp_ph_ev[(set_g / SPC)*NB +: NB]),
                    .ev_ph_p    (cp_ph_p[(set_g / SPC)*NB +: NB]),
                    .ev_tag     (ev_tag),
                    .ev_st      (ev_st),
                    .ev_lo      (ev_lo),
                    .ev_lo3     (ev_lo3),
                    .ev_ph      (ev_ph),
                    .filt_pm    (filt_pm),
                    .filt_lip   (cp_filt_lip[(set_g/SPL)*FL_W +: FL_W]),
                    .prev_pm    (prev_pm),
                    .prev_lip   (cp_prev_lip[(set_g/SPL)*LIP_W +: LIP_W]),
                    .p_tag      (p_tag),
                    .p_st       (p_st),
                    .p_lo       (p_lo),
                    .p_lo3      (p_lo3),
                    .p_ph       (p_ph),
                    .a_key      (cp_a_key[(set_g / SPC)]),
                    .a_correct  (cp_a_correct[(set_g / SPC)]),
                    .a_stride   (cp_a_stride[(set_g / SPC)*(PAGE_W+1) +: PAGE_W+1]),
                    .a_exp      (cp_a_exp[(set_g / SPC)*(PAGE_W+1) +: PAGE_W+1]),
                    .a_s3       (cp_a_s3[(set_g / SPC)*(PAGE_W+3) +: PAGE_W+3]),
                    .a_tgt      (cp_a_tgt[(set_g / SPC)*(PAGE_W+3) +: PAGE_W+3]),
                    .a_nz       (cp_a_nz[(set_g / SPC)]),
                    .alloc_en   (ent_alloc[(set_g * NW + way_g)]),
                    .hit        (ent_hit[(set_g * NW + way_g)]),
                    .class_rank (ent_class[(set_g * NW + way_g)*3 +: 3]),
                    .fire_new   (ent_fire_new[(set_g * NW + way_g)]),
                    .fire_dup   (ent_fire_dup[(set_g * NW + way_g)]),
                    .tgt_lip    (ent_tgt_lip[(set_g * NW + way_g)*LIP_W +: LIP_W])
                );
            end
            assign set_miss = ~(|ent_hit[set_g*NW +: NW]);
            always @* begin : victim_proc
                integer way_idx;
                integer other_idx;
                way_class = ent_class[set_g*NW*3 +: NW*3];
                for (way_idx = 0; way_idx < NW; way_idx = way_idx + 1) begin
                    way_beaten = 1'b0;
                    for (other_idx = 0; other_idx < NW; other_idx = other_idx + 1) begin
                        if (other_idx != way_idx) begin
                            if ((way_class[other_idx*3 +: 3] < way_class[way_idx*3 +: 3])
                                || ((way_class[other_idx*3 +: 3] == way_class[way_idx*3 +: 3])
                                    && (other_idx < way_idx))) begin
                                way_beaten = 1'b1;
                            end
                        end
                    end
                    victim_way[way_idx] = ~way_beaten;
                end
            end
            always @(posedge clk) begin : set_fire_proc
                if (rst || clear) begin
                    fire_new_q[set_g] <= 1'b0;
                    fire_dup_q[set_g] <= 1'b0;
                end
                else begin
                    fire_new_q[set_g] <= |ent_fire_new[set_g*NW +: NW];
                    fire_dup_q[set_g] <= |ent_fire_dup[set_g*NW +: NW];
                end
            end
            reg [LIP_W-1:0] set_lip_next;
            always @* begin : set_lip_next_proc
                integer way_idx;
                set_lip_next = {LIP_W{1'b0}};
                for (way_idx = 0; way_idx < NW; way_idx = way_idx + 1) begin
                    set_lip_next = set_lip_next
                        | ({LIP_W{ent_hit[set_g*NW + way_idx]}}
                           & ent_tgt_lip[(set_g*NW + way_idx)*LIP_W +: LIP_W]);
                end
            end
            reg [NB-1:0] set_bank_next;
            always @* begin : set_bank_next_proc
                integer way_idx;
                integer bank_idx;
                set_bank_next = {NB{1'b0}};
                for (way_idx = 0; way_idx < NW; way_idx = way_idx + 1) begin
                    for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
                        set_bank_next[bank_idx] = set_bank_next[bank_idx]
                            | (ent_hit[set_g*NW + way_idx]
                               & (ent_tgt_lip[(set_g*NW + way_idx)*LIP_W +: BANK_W]
                                  == bank_idx[BANK_W-1:0]));
                    end
                end
            end
            assign set_lip_nx[set_g*LIP_W +: LIP_W] = set_lip_next;
            assign set_bank_nx[set_g*NB +: NB]     = set_bank_next;
        end
    endgenerate
    reg  [NG*LIP_W-1:0] grp_lip_next;
    reg  [NG*NB-1:0]    grp_bank_next;
    always @* begin : grp_next_proc
        integer grp_idx;
        integer sub_idx;
        grp_lip_next  = {(NG*LIP_W){1'b0}};
        grp_bank_next = {(NG*NB){1'b0}};
        for (grp_idx = 0; grp_idx < NG; grp_idx = grp_idx + 1) begin
            for (sub_idx = 0; sub_idx < SPG; sub_idx = sub_idx + 1) begin
                grp_lip_next[grp_idx*LIP_W +: LIP_W] =
                    grp_lip_next[grp_idx*LIP_W +: LIP_W]
                    | set_lip_nx[(grp_idx*SPG + sub_idx)*LIP_W +: LIP_W];
                grp_bank_next[grp_idx*NB +: NB] =
                    grp_bank_next[grp_idx*NB +: NB]
                    | set_bank_nx[(grp_idx*SPG + sub_idx)*NB +: NB];
            end
        end
    end
    always @(posedge clk) begin : grp_lip_proc
        grp_lip_q     <= grp_lip_next;
        grp_bank_oh_q <= grp_bank_next;
    end
    l1d_pf3_or_rows #(.W(LIP_W), .N(NG)) u_cand_lip (
        .rows   (grp_lip_q),
        .or_out (cand_lip_q)
    );
    l1d_pf3_or_rows #(.W(LIP_W), .N(NG)) u_out_lip (
        .rows   (grp_lip_q),
        .or_out (out_lip)
    );
    l1d_pf3_or_rows #(.W(NB), .N(NG)) u_bank_oh (
        .rows   (grp_bank_oh_q),
        .or_out (bank_oh)
    );
    always @(posedge clk) begin : s2_reg_proc
        if (rst) begin
            e2_vld <= 1'b0;
        end
        else begin
            e2_vld <= e1_vld & ~clear;
        end
        e2_st      <= e1_st;
        e2_ph      <= e1_ph;
        e2_ppn     <= e1_ppn;
    end
    genvar cvcopy_g;
    generate
        for (cvcopy_g = 0; cvcopy_g < CV_N; cvcopy_g = cvcopy_g + 1) begin : g_cv
            wire any_dup;
            wire any_new;
            l1d_pf3_or_rows #(.W (1), .N (NS)) u_dup_or (
                .rows (fire_dup_q), .or_out (any_dup)
            );
            l1d_pf3_or_rows #(.W (1), .N (NS)) u_new_or (
                .rows (fire_new_q), .or_out (any_new)
            );
            assign cand_cp[cvcopy_g] = (prev_ins_q ? any_dup : any_new)
                                       & pf_enable & ~clear;
        end
    endgenerate
    assign cand_valid    = cand_cp[0];
    assign cand_valid_dp = cand_cp[NB+1:1];
    assign cand_line   = {{PAD_W{1'b0}}, e2_ppn, out_lip};
    assign cand_unique = e2_st;
    always @* begin : bank_proc
        integer bank_idx;
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin
            cand_bank[bank_idx] = bank_oh[bank_idx];
        end
    end
    assign filt_lip_next = {cp_filt_lip[FL_W-LIP_W-1:0], cand_lip_q};
    always @(posedge clk) begin : filter_proc
        if (rst || clear) begin
            filt_vld   <= {FILT_DEPTH{1'b0}};
            prev_ins_q <= 1'b0;
        end
        else begin
            prev_ins_q <= cand_cp[NB+2];
            if (cand_cp[NB+2]) begin
                filt_vld <= {filt_vld[FILT_DEPTH-2:0], 1'b1};
            end
        end
        if (cand_cp[NB+2]) begin
            filt_ph <= {filt_ph[(FILT_DEPTH-1)*PH_W-1:0], e2_ph};
        end
    end
`ifdef L1D_DEBUG
    initial begin : dist_check
        if (DIST != 2) begin
            $display("L1D PF3_STRIDE ERROR DIST must be 2 (A4), is %0d", DIST);
        end
    end
    always @(posedge clk) begin : debug_proc
        if (!rst) begin
            if ((ent_hit & (ent_hit - 1'b1)) != {ENTRIES{1'b0}}) begin
                $display("[%0t] L1D PF3_STRIDE ERROR two entries hit", $time);
            end
            if ((fire_new_q & (fire_new_q - 1'b1)) != {NS{1'b0}}) begin
                $display("[%0t] L1D PF3_STRIDE ERROR two sets fired", $time);
            end
            if ((ent_alloc & ent_hit) != {ENTRIES{1'b0}}) begin
                $display("[%0t] L1D PF3_STRIDE ERROR allocation on a hit", $time);
            end
        end
    end
`endif
endmodule
`default_nettype wire
`endif
