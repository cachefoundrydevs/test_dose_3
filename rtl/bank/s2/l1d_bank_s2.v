`ifndef L1D_BANK_S2_V
`define L1D_BANK_S2_V
`default_nettype none
`include "l1d_defines.vh"
module l1d_bank_s2 #(
    `L1D_PARAMS,
    parameter BANK_ID = 0
) (
    clk, rst,
    s2_valid, s2_kind, s2_seat, s2_req, s2_mid, s2_line, s2_sinv,
    s2_hit, s2_scfail, s2_snp_hit, s2_hand, s2_way, s2_held, s2_swap, s2_kill,
    s2_words, s2_lines, s2_tags, s2_st_be, s2_st_din,
    s2_wbpend, s2_vvalid, s2_vdirty, s2_vshared, s2_vline, s2_slot,
    s2_raw_hit, s2_pf_hit, s2_tag_blocked,
    s2_data_synd, s2_data_fix, s2_data_ce, s2_data_ue,
    s2_tag_synd, s2_tag_ce, s2_tag_ue,
    s2_seat_trained, s2_seat_pc,
    q2_data,
    mshr_hit, mshr_hid, mshr_hasd, mshr_can, mshr_aid, prefetch_can_allocate,
    vc_hit, vc_slot, vc_state, vc_data, vc_dirty,
    nc_can,
    s2_refill, s2_maint, s2_st_wr, s2_tag_fix_wr, s2_write_way, s2_write_line, s2_write_tag,
    s2_uniq, s2_amo1, s2_alloc_is_write, s2_alloc_no_response, s2_alloc_is_prefetch,
    s2_line_answer, vec_release_valid, vec_release_slot,
    line_q, st_merged,
    act_valid, act_code, act_id,
    s3_load, s2_pkt,
    m_alloc, m_swap, vc_swap, n_alloc,
    ev_lr, ev_sc, ev_wr, ev_amor, ev_amow, rsv_event_pulse,
    snoop_response_valid, snoop_response_packet,
    snp_act_valid, snp_act_code, snp_act_id,
    vc_snp, vc_ins, vc_rel,
    s3_amo_v, s3_line, s3_way, s3_old, s3_be, s3_off, s3_new,
    s2_train_sent, train_valid, train_port, train_pc, train_addr, train_is_store, train_miss, train_pf_hit, train_no_train,
    evict_valid, evict_line_addr,
    pf_fb_valid, pf_fb_line_addr, pf_fb_result,
    ecc_correctable, ecc_uncorrectable
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"
    localparam [OFFSET_W-1:0] OFF0 = {OFFSET_W{1'b0}};
    localparam [WRITE_DATA_WIDTH-1:0] SC_FAIL_W = {{(WRITE_DATA_WIDTH-1){1'b0}}, SC_FAIL};
    localparam SYND_W  = WAYS * GRANULES * DATA_HAM_W;
    localparam HSYND_W = GRANULES * DATA_HAM_W;
    localparam FIX_W   = WAYS * GRANULES;
    localparam TSYND_W = WAYS * TAG_HAM_W;
    input  wire                        clk;
    input  wire                        rst;
    input  wire                        s2_valid;
    input  wire [1:0]                  s2_kind;
    input  wire [SEAT_W-1:0]           s2_seat;
    input  wire [REQ_W-1:0]            s2_req;
    input  wire [MSHR_ID_W-1:0]        s2_mid;
    input  wire [LINE_ADDR_W-1:0]      s2_line;
    input  wire                        s2_sinv;
    input  wire                        s2_hit;
    input  wire                        s2_scfail;
    input  wire                        s2_snp_hit;
    input  wire                        s2_hand;
    input  wire [WAY_W-1:0]            s2_way;
    input  wire                        s2_held;
    input  wire                        s2_swap;
    input  wire                        s2_kill;
    input  wire [WAYS*WRITE_DATA_WIDTH-1:0] s2_words;
    input  wire [WAYS*LINE_W-1:0]      s2_lines;
    input  wire [WAYS*TAG_W-1:0]       s2_tags;
    input  wire [LINE_BYTES-1:0]       s2_st_be;
    input  wire [LINE_W-1:0]           s2_st_din;
    input  wire                        s2_wbpend;
    input  wire                        s2_vvalid;
    input  wire                        s2_vdirty;
    input  wire                        s2_vshared;
    input  wire [LINE_ADDR_W-1:0]      s2_vline;
    input  wire [VC_ID_W-1:0]          s2_slot;
    input  wire                        s2_raw_hit;
    input  wire                        s2_pf_hit;
    input  wire                        s2_tag_blocked;
    input  wire [SYND_W-1:0]           s2_data_synd;
    input  wire [FIX_W-1:0]            s2_data_fix;
    input  wire [WAYS-1:0]             s2_data_ce;
    input  wire [WAYS-1:0]             s2_data_ue;
    input  wire [TSYND_W-1:0]          s2_tag_synd;
    input  wire [WAYS-1:0]             s2_tag_ce;
    input  wire [WAYS-1:0]             s2_tag_ue;
    input  wire                        s2_seat_trained;
    input  wire [PC_W-1:0]             s2_seat_pc;
    input  wire [LINE_W-1:0]           q2_data;
    input  wire                        mshr_hit;
    input  wire [MSHR_ID_W-1:0]        mshr_hid;
    input  wire                        mshr_hasd;
    input  wire                        mshr_can;
    input  wire [MSHR_ID_W-1:0]        mshr_aid;
    input  wire                        prefetch_can_allocate;
    input  wire                        vc_hit;
    input  wire [VC_ID_W-1:0]          vc_slot;
    input  wire [2:0]                  vc_state;
    input  wire [LINE_W-1:0]           vc_data;
    input  wire                        vc_dirty;
    input  wire                        nc_can;
    output wire                        s2_refill;
    output wire                        s2_maint;
    output wire                        s2_st_wr;
    output wire                        s2_tag_fix_wr;
    output wire [WAY_W-1:0]            s2_write_way;
    output wire [LINE_W-1:0]           s2_write_line;
    output wire [TAG_W-1:0]            s2_write_tag;
    output wire                        s2_uniq;
    output wire                        s2_amo1;
    output wire                        s2_alloc_is_write;
    output wire                        s2_alloc_no_response;
    output wire                        s2_alloc_is_prefetch;
    output wire [LINE_W-1:0]           s2_line_answer;
    output wire                        vec_release_valid;
    output wire [VEC_SLOT_W-1:0]       vec_release_slot;
    output reg  [LINE_W-1:0]           line_q;
    output reg  [LINE_W-1:0]           st_merged;
    output reg                         act_valid;
    output reg  [2:0]                  act_code;
    output reg  [WID_W-1:0]            act_id;
    output reg                         s3_load;
    output wire [RESP_W-1:0]           s2_pkt;
    output reg                         m_alloc;
    output reg                         m_swap;
    output reg                         vc_swap;
    output reg                         n_alloc;
    output wire                        ev_lr;
    output wire                        ev_sc;
    output wire                        ev_wr;
    output wire                        ev_amor;
    output wire                        ev_amow;
    output wire                        rsv_event_pulse;
    output wire                        snoop_response_valid;
    output wire [SNPR_W-1:0]           snoop_response_packet;
    output reg                         snp_act_valid;
    output reg  [1:0]                  snp_act_code;
    output reg  [WID_W-1:0]            snp_act_id;
    output reg                         vc_snp;
    output wire                        vc_ins;
    output wire                        vc_rel;
    output reg                         s3_amo_v;
    output reg  [LINE_ADDR_W-1:0]      s3_line;
    output reg  [WAY_W-1:0]            s3_way;
    output reg  [LINE_W-1:0]           s3_old;
    output reg  [LINE_BYTES-1:0]       s3_be;
    output reg  [OFFSET_W-1:0]         s3_off;
    output reg  [WRITE_DATA_WIDTH-1:0] s3_new;
    output wire                        s2_train_sent;
    output reg                         train_valid;
    output reg  [PORT_W-1:0]           train_port;
    output reg  [PC_W-1:0]             train_pc;
    output reg  [ADDR_W-1:0]           train_addr;
    output reg                         train_is_store;
    output reg                         train_miss;
    output reg                         train_pf_hit;
    output reg                         train_no_train;
    output reg                         evict_valid;
    output reg  [LINE_ADDR_W-1:0]      evict_line_addr;
    output reg                         pf_fb_valid;
    output reg  [LINE_ADDR_W-1:0]      pf_fb_line_addr;
    output reg  [1:0]                  pf_fb_result;
    output reg                         ecc_correctable;
    output reg                         ecc_uncorrectable;
    wire               s2_lsu;
    wire               s2_pf;
    wire               s2_pfu;
    wire               s2_pf_op;
    wire               s2_cpu;
    wire               s2_live;
    wire               s2_line_op;
    wire               s2_cache;
    wire               s2_store;
    wire               s2_amo;
    wire               s2_seq;
    wire               s2_is_lr;
    wire               s2_is_sc;
    wire [ADDR_W-1:0]  s2_addr;
    wire [OFFSET_W-1:0]         s2_off;
    wire [MEM_SIZE_BITS-1:0]    s2_size;
    wire [MASTER_ID_BITS-1:0]   s2_mstid;
    wire [IDX_BITS-1:0]         s2_idx;
    wire [PORT_W-1:0]           s2_port;
    wire [WRITE_DATA_WIDTH-1:0] s2_wdata;
    assign s2_lsu     = s2_valid && (s2_kind == K_LSU);
    assign s2_pf      = s2_req[REQ_PF_BIT];
    assign s2_pfu     = s2_req[REQ_PFU_BIT];
    assign s2_pf_op   = s2_lsu && s2_pf;
    assign s2_cpu     = s2_lsu && !s2_pf;
    assign s2_live    = s2_cpu && !s2_kill;
    assign s2_line_op = s2_req[REQ_LINE_BIT];
    assign s2_cache   = s2_req[REQ_CACHE_BIT];
    assign s2_store   = (s2_req[REQ_RDWR_BIT] == OP_WRITE);
    assign s2_amo     = s2_req[REQ_AMO_BIT];
    assign s2_seq     = s2_req[REQ_SEQ_BIT];
    assign s2_is_lr   = s2_amo && !s2_seq && !s2_store;
    assign s2_is_sc   = s2_amo && !s2_seq &&  s2_store;
    assign s2_amo1    = s2_amo && s2_seq;
    assign s2_addr    = s2_req[REQ_ADDR_LSB +: ADDR_W];
    assign s2_off     = s2_addr[OFFSET_W-1:0];
    assign s2_size    = s2_req[REQ_SIZE_LSB +: MEM_SIZE_BITS];
    assign s2_mstid   = s2_req[REQ_MID_LSB +: MASTER_ID_BITS];
    assign s2_idx     = s2_req[REQ_IDX_LSB +: IDX_BITS];
    assign s2_port    = s2_req[REQ_PORT_LSB +: PORT_W];
    assign s2_wdata   = s2_req[REQ_DATA_LSB +: WRITE_DATA_WIDTH];
    assign s2_uniq               = s2_pf ? s2_pfu : (s2_store || s2_amo);
    assign s2_alloc_is_write     = s2_req[REQ_RDWR_BIT] && !s2_amo1 && !s2_line_op;
    assign s2_alloc_no_response  = s2_amo1 || s2_line_op || s2_pf;
    assign s2_alloc_is_prefetch  = s2_pf;
    wire s2_snp;
    assign s2_refill = s2_valid && (s2_kind == K_REFILL) && !s2_held;
    assign s2_maint  = s2_valid && (s2_kind == K_MAINT);
    assign s2_snp    = s2_valid && (s2_kind == K_SNOOP);
    integer lw;
    always @* begin
        line_q = {LINE_W{1'b0}};
        for (lw = 0; lw < WAYS; lw = lw + 1)
            line_q = line_q | ({LINE_W{s2_way == lw}} & s2_lines[lw*LINE_W +: LINE_W]);
    end
    integer mb;
    always @* begin
        for (mb = 0; mb < LINE_BYTES; mb = mb + 1)
            st_merged[mb*8 +: 8] = s2_st_be[mb] ? s2_st_din[mb*8 +: 8] : line_q[mb*8 +: 8];
    end
    wire [WAYS-1:0] hit_way_onehot;
    generate
        genvar hit_way_idx;
        for (hit_way_idx = 0; hit_way_idx < WAYS; hit_way_idx = hit_way_idx + 1) begin : g_hit_way_onehot
            assign hit_way_onehot[hit_way_idx] = (s2_way == hit_way_idx);
        end
    endgenerate
    reg  [HSYND_W-1:0] hit_synd;
    reg  [GRANULES-1:0] hit_fix;
    reg                hit_data_ce;
    reg                hit_data_ue;
    integer ew;
    always @* begin
        hit_synd    = {HSYND_W{1'b0}};
        hit_fix     = {GRANULES{1'b0}};
        hit_data_ce = 1'b0;
        hit_data_ue = 1'b0;
        for (ew = 0; ew < WAYS; ew = ew + 1) begin
            hit_synd    = hit_synd    | ({HSYND_W{s2_way == ew}}  & s2_data_synd[ew*HSYND_W +: HSYND_W]);
            hit_fix     = hit_fix     | ({GRANULES{s2_way == ew}} & s2_data_fix[ew*GRANULES +: GRANULES]);
            hit_data_ce = hit_data_ce | ((s2_way == ew) & s2_data_ce[ew]);
            hit_data_ue = hit_data_ue | ((s2_way == ew) & s2_data_ue[ew]);
        end
    end
    reg  [WAYS-1:0]    tag_fix_onehot;
    reg                tag_fix_seen;
    reg  [WAY_W-1:0]   tag_fix_way;
    reg  [TAG_W-1:0]   tag_err_tag;
    reg  [TAG_HAM_W-1:0] tag_err_synd;
    wire               tag_fix_valid;
    wire               tag_ue_any;
    integer tw;
    always @* begin
        tag_fix_seen = 1'b0;
        tag_fix_way  = {WAY_W{1'b0}};
        tag_err_tag  = {TAG_W{1'b0}};
        tag_err_synd = {TAG_HAM_W{1'b0}};
        for (tw = 0; tw < WAYS; tw = tw + 1) begin
            tag_fix_onehot[tw] = s2_tag_ce[tw] & ~tag_fix_seen;
            tag_fix_seen       = tag_fix_seen | s2_tag_ce[tw];
            tag_fix_way        = tag_fix_way  | ({WAY_W{tag_fix_onehot[tw]}} & tw[WAY_W-1:0]);
            tag_err_tag        = tag_err_tag  | ({TAG_W{tag_fix_onehot[tw]}} & s2_tags[tw*TAG_W +: TAG_W]);
            tag_err_synd       = tag_err_synd | ({TAG_HAM_W{tag_fix_onehot[tw]}} & s2_tag_synd[tw*TAG_HAM_W +: TAG_HAM_W]);
        end
    end
    assign tag_fix_valid = |s2_tag_ce;
    assign tag_ue_any    = |s2_tag_ue;
    wire ecc_data_fix;
    wire ecc_data_ue;
    wire ecc_tag_blk;
    wire ecc_tag_fix_wr;
    wire ecc_tag_ue;
    wire            live_cacheable_hit;
    wire [WAYS-1:0] way_data_fix_flag;
    wire [WAYS-1:0] way_data_ue_flag;
    assign live_cacheable_hit = s2_live && s2_cache && s2_hit;
    assign way_data_fix_flag  = {WAYS{live_cacheable_hit}} & s2_data_ce & ~s2_data_ue;
    assign way_data_ue_flag   = {WAYS{live_cacheable_hit}} & s2_data_ue;
    assign ecc_data_ue    = |(hit_way_onehot & way_data_ue_flag);
    assign ecc_data_fix   = |(hit_way_onehot & way_data_fix_flag);
    assign ecc_tag_blk    = s2_live && s2_tag_blocked;
    assign ecc_tag_fix_wr = ecc_tag_blk && tag_fix_valid;
    assign ecc_tag_ue     = ecc_tag_blk && !tag_fix_valid && tag_ue_any;
    wire [LINE_W-1:0] fixed_line;
    wire [TAG_W-1:0]  fixed_tag;
    generate
        if (HAS_ECC != 0) begin : g_ecc_fix
            genvar fg;
            for (fg = 0; fg < GRANULES; fg = fg + 1) begin : g_fix_gran
                l1d_ecc_fix #(.DATA_W(GRAN_BITS), .HAM_W(DATA_HAM_W)) u_fix (
                    .data       (line_q[fg*GRAN_BITS +: GRAN_BITS]),
                    .syndrome   (hit_synd[fg*DATA_HAM_W +: DATA_HAM_W]),
                    .fix_enable (hit_fix[fg]),
                    .fixed      (fixed_line[fg*GRAN_BITS +: GRAN_BITS])
                );
            end
            l1d_ecc_fix #(.DATA_W(TAG_W), .HAM_W(TAG_HAM_W)) u_tag_fix (
                .data       (tag_err_tag),
                .syndrome   (tag_err_synd),
                .fix_enable (tag_fix_valid),
                .fixed      (fixed_tag)
            );
        end else begin : g_no_ecc_fix
            assign fixed_line = line_q;
            assign fixed_tag  = tag_err_tag;
        end
    endgenerate
    wire [WRITE_DATA_WIDTH-1:0] vcl_data;
    reg  [WRITE_DATA_WIDTH-1:0] hit_data;
    integer hw;
    always @* begin
        hit_data = {WRITE_DATA_WIDTH{1'b0}};
        for (hw = 0; hw < WAYS; hw = hw + 1)
            hit_data = hit_data | ({WRITE_DATA_WIDTH{s2_way == hw}} & s2_words[hw*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH]);
    end
    l1d_resp_gen #(.IN_BYTES(LINE_BYTES), .WRITE_DATA_WIDTH(WRITE_DATA_WIDTH),
                   .MEM_SIZE_BITS(MEM_SIZE_BITS), .OFF_W(OFFSET_W)) u_vc_data (
        .source_block (vc_data), .byte_offset (s2_off), .access_size (s2_size), .load_data (vcl_data)
    );
    reg  s3_from_vc;
    reg  s3_scfail;
    reg  s3_zero;
    reg  [1:0] pf_result;
    wire vc_line_present;
    assign vc_line_present = vc_hit && ((vc_state == VC_VALID) || (vc_state == VC_WBPEND));
    always @* begin
        s3_load    = 1'b0;
        s3_from_vc = 1'b0;
        s3_scfail  = 1'b0;
        s3_zero    = 1'b0;
        m_alloc    = 1'b0;
        m_swap     = 1'b0;
        vc_swap    = 1'b0;
        n_alloc    = 1'b0;
        act_valid  = 1'b0;
        act_code   = ACT_NONE;
        act_id     = {WID_W{1'b0}};
        pf_result  = PF_FB_DROPPED;
        if (s2_pf_op) begin
            if (s2_raw_hit || vc_line_present) begin
                pf_result = PF_FB_IN_CACHE;
            end else if (mshr_hit) begin
                pf_result = PF_FB_IN_FLIGHT;
            end else if (mshr_can && prefetch_can_allocate) begin
                m_alloc   = 1'b1;
                pf_result = PF_FB_ISSUED;
            end else begin
                pf_result = PF_FB_DROPPED;
            end
        end else if (s2_lsu && s2_kill) begin
            act_valid = 1'b1;
            act_code  = ACT_NONE;
        end else if (s2_lsu) begin
            if (s2_tag_blocked) begin
                act_valid = 1'b1;
                act_code  = ACT_WECC;
            end else if (!s2_cache) begin
                if (s2_is_sc) begin
                    s3_load   = 1'b1;
                    s3_scfail = 1'b1;
                end else if (s2_amo1) begin
                    s3_load = 1'b1;
                    s3_zero = 1'b1;
                end else begin
                    act_valid = 1'b1;
                    if (nc_can) begin
                        n_alloc  = 1'b1;
                        act_code = ACT_OWNED;
                    end else begin
                        act_code = ACT_WNC;
                    end
                end
            end else if (s2_scfail) begin
                s3_load   = 1'b1;
                s3_scfail = 1'b1;
            end else if (s2_hit && ecc_data_fix) begin
                act_valid = 1'b1;
                act_code  = ACT_WECC;
            end else if (s2_hit) begin
                s3_load = 1'b1;
            end else if (mshr_hit) begin
                act_valid = 1'b1;
                act_code  = ACT_WMSHR;
                act_id    = mshr_hid;
            end else if (vc_hit && vc_state == VC_VALID) begin
                if (!s2_uniq) begin
                    s3_load    = 1'b1;
                    s3_from_vc = 1'b1;
                    if (mshr_can) begin
                        m_alloc = 1'b1;
                        m_swap  = 1'b1;
                        vc_swap = 1'b1;
                    end
                end else begin
                    act_valid = 1'b1;
                    if (mshr_can) begin
                        m_alloc  = 1'b1;
                        m_swap   = 1'b1;
                        vc_swap  = 1'b1;
                        act_code = ACT_WMSHR;
                        act_id   = mshr_aid;
                    end else begin
                        act_code = ACT_WANY;
                    end
                end
            end else if (vc_hit && vc_state == VC_WBPEND) begin
                if (!s2_uniq) begin
                    s3_load    = 1'b1;
                    s3_from_vc = 1'b1;
                end else begin
                    act_valid = 1'b1;
                    act_code  = ACT_WVC;
                    act_id    = vc_slot;
                end
            end else if (vc_hit) begin
                act_valid = 1'b1;
                act_code  = ACT_WANY;
            end else begin
                act_valid = 1'b1;
                if (mshr_can) begin
                    m_alloc  = 1'b1;
                    if (s2_amo1 || s2_line_op) begin
                        act_code = ACT_WMSHR;
                        act_id   = mshr_aid;
                    end else begin
                        act_code = ACT_OWNED;
                    end
                end else begin
                    act_code = ACT_WANY;
                end
            end
        end
    end
    wire store_hit_wr;
    assign store_hit_wr  = s2_live && s2_hit && s2_store && !s2_amo1 && !ecc_data_fix;
    assign s2_st_wr      = store_hit_wr || ecc_data_fix;
    assign s2_tag_fix_wr = ecc_tag_fix_wr;
    assign s2_write_way  = ecc_tag_fix_wr ? tag_fix_way : s2_way;
    assign s2_write_line = ({LINE_W{s2_refill}}     & q2_data)    |
                           ({LINE_W{ecc_data_fix}}  & fixed_line) |
                           ({LINE_W{store_hit_wr}}  & st_merged);
    assign s2_write_tag  = ecc_tag_fix_wr ? fixed_tag : s2_line[LA_TAG_LSB +: TAG_W];
    wire [WRITE_DATA_WIDTH-1:0] s2_rdata;
    assign s2_rdata = s3_scfail ? SC_FAIL_W :
                      s3_zero   ? {WRITE_DATA_WIDTH{1'b0}} :
                      (s2_store && !s2_amo1) ? {WRITE_DATA_WIDTH{1'b0}} :
                      (s3_from_vc ? vcl_data : hit_data);
    assign s2_pkt = {s2_port, (s2_req[REQ_RDWR_BIT] && !s2_amo1), s2_size, s2_mstid, s2_idx, s2_rdata};
    assign s2_line_answer    = s3_from_vc ? vc_data : line_q;
    assign vec_release_valid = s2_cpu && s3_load && s2_line_op && s2_store;
    assign vec_release_slot  = s2_req[REQ_DATA_LSB +: VEC_SLOT_W];
    wire s2_done_lsu;
    assign s2_done_lsu = s2_cpu && (s3_load || (act_valid && act_code == ACT_OWNED));
    assign ev_lr   = s2_done_lsu && s2_cache && s2_is_lr;
    assign ev_sc   = s2_done_lsu && s2_is_sc;
    assign rsv_event_pulse = ev_lr || ev_sc;
    assign ev_wr   = s2_done_lsu && s2_cache && s2_store && !s2_is_sc;
    assign ev_amor = 1'b0;
    assign ev_amow = 1'b0;
    wire [WRITE_DATA_WIDTH-1:0] amo_new;
    localparam AMO_CMP_GROUPS = WRITE_DATA_WIDTH / 8;
    localparam AMO_LESS_TERMS = AMO_CMP_GROUPS + 1;
    wire [AMOOP_W-1:0] amo_op_code;
    wire               amo_size_is64;
    wire               amo_op_add;
    wire               amo_op_xor;
    wire               amo_op_and;
    wire               amo_op_or;
    wire               amo_op_min_any;
    wire               amo_op_max_any;
    wire               amo_op_signed;
    wire               amo_op_swap_other;
    wire               amo_op_logic;
    wire [WRITE_DATA_WIDTH-1:0] amo_logic_operand;
    wire [WRITE_DATA_WIDTH-1:0] amo_min_operand;
    wire [WRITE_DATA_WIDTH-1:0] amo_max_operand;
    assign amo_op_code       = s2_req[REQ_AMOOP_LSB +: AMOOP_W];
    assign amo_size_is64     = (s2_size == 2'd3);
    assign amo_op_add        = (amo_op_code == AMO_ADD);
    assign amo_op_xor        = (amo_op_code == AMO_XOR);
    assign amo_op_and        = (amo_op_code == AMO_AND);
    assign amo_op_or         = (amo_op_code == AMO_OR);
    assign amo_op_min_any    = (amo_op_code == AMO_MIN) | (amo_op_code == AMO_MINU);
    assign amo_op_max_any    = (amo_op_code == AMO_MAX) | (amo_op_code == AMO_MAXU);
    assign amo_op_signed     = (amo_op_code == AMO_MIN) | (amo_op_code == AMO_MAX);
    assign amo_op_swap_other = ~(amo_op_add | amo_op_xor | amo_op_and | amo_op_or | amo_op_min_any | amo_op_max_any);
    assign amo_op_logic      = amo_op_add | amo_op_xor;
    assign amo_logic_operand = {WRITE_DATA_WIDTH{amo_op_logic}}   & s2_wdata;
    assign amo_min_operand   = {WRITE_DATA_WIDTH{amo_op_min_any}} & s2_wdata;
    assign amo_max_operand   = {WRITE_DATA_WIDTH{amo_op_max_any}} & s2_wdata;
    wire [WAYS*WRITE_DATA_WIDTH-1:0] amo_new_way;
    wire [WAYS*WRITE_DATA_WIDTH-1:0] amo_old_if_less_way;
    wire [WAYS*WRITE_DATA_WIDTH-1:0] amo_old_if_not_less_way;
    wire [WAYS*AMO_LESS_TERMS-1:0]   amo_less_term_way;
    wire [WAYS*WRITE_DATA_WIDTH-1:0] amo_xor_way;
    wire [WAYS*WRITE_DATA_WIDTH-1:0] amo_sum_if_carry_way;
    wire [WAYS*WRITE_DATA_WIDTH-1:0] amo_carry_way;
    wire [WRITE_DATA_WIDTH-1:0]      amo_new_no_way;
    wire                             hit_way_none;
    generate
        genvar amo_way_idx;
        for (amo_way_idx = 0; amo_way_idx < WAYS; amo_way_idx = amo_way_idx + 1) begin : g_amo_way
            wire [WRITE_DATA_WIDTH-1:0] old_word;
            wire way_and;
            wire way_or;
            wire way_min;
            wire way_max;
            wire way_logic;
            wire way_add;
            assign old_word  = s2_words[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];
            assign way_and   = hit_way_onehot[amo_way_idx] & amo_op_and;
            assign way_or    = hit_way_onehot[amo_way_idx] & amo_op_or;
            assign way_min   = hit_way_onehot[amo_way_idx] & amo_op_min_any;
            assign way_max   = hit_way_onehot[amo_way_idx] & amo_op_max_any;
            assign way_logic = hit_way_onehot[amo_way_idx] & amo_op_logic;
            assign way_add   = hit_way_onehot[amo_way_idx] & amo_op_add;
            wire [WRITE_DATA_WIDTH-1:0] cmp_gen;
            wire [WRITE_DATA_WIDTH-1:0] cmp_prop;
            wire cmp_gen_top;
            wire cmp_prop_top;
            wire cmp_gen_half;
            wire cmp_prop_half;
            wire [AMO_CMP_GROUPS-1:0] cmp_group_gen;
            wire [AMO_CMP_GROUPS-1:0] cmp_group_factor;
            wire cmp_hit_early;
            assign cmp_gen         = ~old_word & s2_wdata;
            assign cmp_prop        = ~old_word | s2_wdata;
            assign cmp_gen_top     = (amo_op_signed & old_word[63] & ~s2_wdata[63]) | (~amo_op_signed & cmp_gen[63]);
            assign cmp_prop_top    = (amo_op_signed & (old_word[63] | ~s2_wdata[63])) | (~amo_op_signed & cmp_prop[63]);
            assign cmp_gen_half    = (amo_op_signed & old_word[31] & ~s2_wdata[31]) | (~amo_op_signed & cmp_gen[31]);
            assign cmp_prop_half   = (amo_op_signed & (old_word[31] | ~s2_wdata[31])) | (~amo_op_signed & cmp_prop[31]);
            assign cmp_group_gen   = {
                (cmp_gen[62] | (&cmp_prop[62:61] & (cmp_gen[61] | cmp_gen[60])) | (&cmp_prop[62:59] & (cmp_gen[59] | cmp_gen[58])) | (&cmp_prop[62:57] & (cmp_gen[57] | cmp_gen[56]))),
                ((cmp_gen[55] | cmp_gen[54]) | (&cmp_prop[54:53] & (cmp_gen[53] | cmp_gen[52])) | (&cmp_prop[54:51] & (cmp_gen[51] | cmp_gen[50])) | (&cmp_prop[54:49] & (cmp_gen[49] | cmp_gen[48]))),
                ((cmp_gen[47] | cmp_gen[46]) | (&cmp_prop[46:45] & (cmp_gen[45] | cmp_gen[44])) | (&cmp_prop[46:43] & (cmp_gen[43] | cmp_gen[42])) | (&cmp_prop[46:41] & (cmp_gen[41] | cmp_gen[40]))),
                ((cmp_gen[39] | cmp_gen[38]) | (&cmp_prop[38:37] & (cmp_gen[37] | cmp_gen[36])) | (&cmp_prop[38:35] & (cmp_gen[35] | cmp_gen[34])) | (&cmp_prop[38:33] & (cmp_gen[33] | cmp_gen[32]))),
                (cmp_gen[30] | (&cmp_prop[30:29] & (cmp_gen[29] | cmp_gen[28])) | (&cmp_prop[30:27] & (cmp_gen[27] | cmp_gen[26])) | (&cmp_prop[30:25] & (cmp_gen[25] | cmp_gen[24]))),
                ((cmp_gen[23] | cmp_gen[22]) | (&cmp_prop[22:21] & (cmp_gen[21] | cmp_gen[20])) | (&cmp_prop[22:19] & (cmp_gen[19] | cmp_gen[18])) | (&cmp_prop[22:17] & (cmp_gen[17] | cmp_gen[16]))),
                ((cmp_gen[15] | cmp_gen[14]) | (&cmp_prop[14:13] & (cmp_gen[13] | cmp_gen[12])) | (&cmp_prop[14:11] & (cmp_gen[11] | cmp_gen[10])) | (&cmp_prop[14:9] & (cmp_gen[9] | cmp_gen[8]))),
                ((cmp_gen[7] | cmp_gen[6]) | (&cmp_prop[6:5] & (cmp_gen[5] | cmp_gen[4])) | (&cmp_prop[6:3] & (cmp_gen[3] | cmp_gen[2])) | (&cmp_prop[6:1] & (cmp_gen[1] | cmp_gen[0])))
            };
            assign cmp_group_factor = {AMO_CMP_GROUPS{hit_way_onehot[amo_way_idx]}} & {
                (amo_size_is64 & cmp_prop_top),
                (amo_size_is64 & cmp_prop_top & (&cmp_prop[62:55])),
                (amo_size_is64 & cmp_prop_top & (&cmp_prop[62:47])),
                (amo_size_is64 & cmp_prop_top & (&cmp_prop[62:39])),
                (amo_size_is64 & cmp_prop_top & (&cmp_prop[62:31])) | (~amo_size_is64 & cmp_prop_half),
                (amo_size_is64 & cmp_prop_top & (&cmp_prop[62:23])) | (~amo_size_is64 & cmp_prop_half & (&cmp_prop[30:23])),
                (amo_size_is64 & cmp_prop_top & (&cmp_prop[62:15])) | (~amo_size_is64 & cmp_prop_half & (&cmp_prop[30:15])),
                (amo_size_is64 & cmp_prop_top & (&cmp_prop[62:7]))  | (~amo_size_is64 & cmp_prop_half & (&cmp_prop[30:7]))
            };
            assign cmp_hit_early   = hit_way_onehot[amo_way_idx] &
                                     ((amo_size_is64 & cmp_gen_top) | (amo_size_is64 & cmp_prop_top & (&cmp_prop[62:32]) & cmp_gen[31]) |
                                      (~amo_size_is64 & cmp_gen_half));
            wire [WRITE_DATA_WIDTH-1:0] add_bit_gen;
            wire [WRITE_DATA_WIDTH-1:0] add_bit_prop;
            wire [WRITE_DATA_WIDTH-1:0] add_span2_gen;
            wire [WRITE_DATA_WIDTH-1:0] add_span2_prop;
            wire [WRITE_DATA_WIDTH-1:0] add_span4_gen;
            wire [WRITE_DATA_WIDTH-1:0] add_span4_prop;
            wire [WRITE_DATA_WIDTH-1:0] add_span8_gen;
            wire [WRITE_DATA_WIDTH-1:0] add_span8_prop;
            wire [WRITE_DATA_WIDTH-1:0] add_span16_gen;
            wire [WRITE_DATA_WIDTH-1:0] add_span16_prop;
            wire [WRITE_DATA_WIDTH-1:0] add_span32_gen;
            wire [WRITE_DATA_WIDTH-1:0] add_span32_prop;
            wire [WRITE_DATA_WIDTH-1:0] old_xor_operand;
            assign add_bit_gen     = old_word & s2_wdata;
            assign add_bit_prop    = old_word | s2_wdata;
            assign add_span2_gen   = add_bit_gen     | (add_bit_gen      << 1);
            assign add_span2_prop  = (add_bit_prop   << 1) & (add_bit_prop << 2);
            assign add_span4_gen   = add_span2_gen   | (add_span2_prop   & (add_span2_gen   << 2));
            assign add_span4_prop  = add_span2_prop  & (add_span2_prop   << 2);
            assign add_span8_gen   = add_span4_gen   | (add_span4_prop   & (add_span4_gen   << 4));
            assign add_span8_prop  = add_span4_prop  & (add_span4_prop   << 4);
            assign add_span16_gen  = add_span8_gen   | (add_span8_prop   & (add_span8_gen   << 8));
            assign add_span16_prop = add_span8_prop  & (add_span8_prop   << 8);
            assign add_span32_gen  = add_span16_gen  | (add_span16_prop  & (add_span16_gen  << 16));
            assign add_span32_prop = add_span16_prop & (add_span16_prop  << 16);
            assign old_xor_operand = ({WRITE_DATA_WIDTH{way_logic}} & old_word) ^ amo_logic_operand;
            assign amo_new_way[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH] =
                       ({WRITE_DATA_WIDTH{way_and}} & add_bit_gen)
                     | ({WRITE_DATA_WIDTH{way_or}}  & add_bit_prop);
            assign amo_old_if_less_way[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH] =
                       {WRITE_DATA_WIDTH{way_min}} & old_word;
            assign amo_old_if_not_less_way[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH] =
                       {WRITE_DATA_WIDTH{way_max}} & old_word;
            assign amo_less_term_way[amo_way_idx*AMO_LESS_TERMS +: AMO_LESS_TERMS] =
                       {cmp_hit_early, cmp_group_factor & cmp_group_gen};
            assign amo_xor_way[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH] =
                       {WRITE_DATA_WIDTH{hit_way_onehot[amo_way_idx]}} & old_xor_operand;
            assign amo_sum_if_carry_way[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH] =
                       {WRITE_DATA_WIDTH{hit_way_onehot[amo_way_idx]}} & (old_xor_operand ^ (add_bit_prop << 1));
            assign amo_carry_way[amo_way_idx*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH] =
                       ({WRITE_DATA_WIDTH{way_add}} & (add_span32_gen << 1))
                     | ({WRITE_DATA_WIDTH{way_add}} & (add_span32_prop << 1) & (add_span32_gen << 33));
        end
        if (WAYS < (1 << WAY_W)) begin : g_amo_no_way
            l1d_amo_alu #(.W(WRITE_DATA_WIDTH), .MEM_SIZE_BITS(MEM_SIZE_BITS)) u_amo_zero (
                .amo_op      (s2_req[REQ_AMOOP_LSB +: AMOOP_W]),
                .access_size (s2_size),
                .old_value   ({WRITE_DATA_WIDTH{1'b0}}),
                .operand     (s2_wdata),
                .new_value   (amo_new_no_way)
            );
            assign hit_way_none = ~|hit_way_onehot;
        end else begin : g_amo_all_ways
            assign amo_new_no_way = {WRITE_DATA_WIDTH{1'b0}};
            assign hit_way_none   = 1'b0;
        end
    endgenerate
    reg  [WRITE_DATA_WIDTH-1:0] amo_new_hit_way;
    reg  [WRITE_DATA_WIDTH-1:0] amo_old_if_less_hit;
    reg  [WRITE_DATA_WIDTH-1:0] amo_old_if_not_less_hit;
    reg  [WRITE_DATA_WIDTH-1:0] amo_xor_hit_way;
    reg  [WRITE_DATA_WIDTH-1:0] amo_sum_if_carry_hit_way;
    reg  [WRITE_DATA_WIDTH-1:0] amo_carry_hit_way;
    integer amo_sel_way;
    always @* begin
        amo_new_hit_way          = {WRITE_DATA_WIDTH{hit_way_none}} & amo_new_no_way;
        amo_old_if_less_hit      = {WRITE_DATA_WIDTH{1'b0}};
        amo_old_if_not_less_hit  = {WRITE_DATA_WIDTH{1'b0}};
        amo_xor_hit_way          = {WRITE_DATA_WIDTH{1'b0}};
        amo_sum_if_carry_hit_way = {WRITE_DATA_WIDTH{1'b0}};
        amo_carry_hit_way        = {WRITE_DATA_WIDTH{1'b0}};
        for (amo_sel_way = 0; amo_sel_way < WAYS; amo_sel_way = amo_sel_way + 1) begin
            amo_new_hit_way          = amo_new_hit_way          | amo_new_way[amo_sel_way*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];
            amo_old_if_less_hit      = amo_old_if_less_hit      | amo_old_if_less_way[amo_sel_way*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];
            amo_old_if_not_less_hit  = amo_old_if_not_less_hit  | amo_old_if_not_less_way[amo_sel_way*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];
            amo_xor_hit_way          = amo_xor_hit_way          | amo_xor_way[amo_sel_way*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];
            amo_sum_if_carry_hit_way = amo_sum_if_carry_hit_way | amo_sum_if_carry_way[amo_sel_way*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];
            amo_carry_hit_way        = amo_carry_hit_way        | amo_carry_way[amo_sel_way*WRITE_DATA_WIDTH +: WRITE_DATA_WIDTH];
        end
    end
    wire amo_less_hit;
    assign amo_less_hit = |amo_less_term_way;
    assign amo_new = amo_new_hit_way
                   | ({WRITE_DATA_WIDTH{amo_op_swap_other}} & s2_wdata)
                   | ((~amo_carry_hit_way & amo_xor_hit_way) | (amo_carry_hit_way & amo_sum_if_carry_hit_way))
                   | (({WRITE_DATA_WIDTH{ amo_less_hit}} & (amo_old_if_less_hit     | ({WRITE_DATA_WIDTH{~hit_way_none}} & amo_max_operand)))
                    | ({WRITE_DATA_WIDTH{~amo_less_hit}} & (amo_old_if_not_less_hit | ({WRITE_DATA_WIDTH{~hit_way_none}} & amo_min_operand))));
    always @(posedge clk) begin : s2_to_s3_proc
        if (rst) s3_amo_v <= 1'b0;
        else     s3_amo_v <= s2_live && s2_hit && s2_amo1 && !ecc_data_fix;
        s3_line <= s2_line;
        s3_way  <= s2_way;
        s3_old  <= line_q;
        s3_be   <= s2_st_be;
        s3_off  <= s2_off;
        s3_new  <= amo_new;
    end
    wire train_now;
    wire evict_now;
    wire [LINE_ADDR_W-1:0] evict_line_now;
    assign train_now      = s2_live && !s2_tag_blocked && !s2_seat_trained;
    assign s2_train_sent  = train_now;
    assign evict_now      = (s2_refill && s2_vvalid) || (s2_snp && s2_snp_hit && s2_sinv) || (s2_maint && !s2_held && s2_vvalid);
    assign evict_line_now = s2_snp ? s2_line : s2_vline;
    always @(posedge clk) begin : records_proc
        if (rst) begin
            train_valid       <= 1'b0;
            evict_valid       <= 1'b0;
            pf_fb_valid       <= 1'b0;
            ecc_correctable   <= 1'b0;
            ecc_uncorrectable <= 1'b0;
        end else begin
            train_valid       <= train_now;
            evict_valid       <= evict_now;
            pf_fb_valid       <= s2_pf_op;
            ecc_correctable   <= ecc_data_fix || ecc_tag_fix_wr;
            ecc_uncorrectable <= ecc_data_ue || ecc_tag_ue;
        end
        train_port      <= s2_port;
        train_pc        <= s2_seat_pc;
        train_addr      <= s2_addr;
        train_is_store  <= s2_store;
        train_miss      <= !s2_hit;
        train_pf_hit    <= s2_pf_hit;
        train_no_train  <= s2_amo || !s2_cache || (s2_mstid != MID_LSU);
        evict_line_addr <= evict_line_now;
        pf_fb_line_addr <= s2_line;
        pf_fb_result    <= pf_result;
    end
    reg  snp_push;
    reg  snp_present;
    reg  snp_dirty;
    reg  snp_from_vc;
    always @* begin
        snp_push      = 1'b0;
        snp_present   = 1'b0;
        snp_dirty     = 1'b0;
        snp_from_vc   = 1'b0;
        vc_snp        = 1'b0;
        snp_act_valid = 1'b0;
        snp_act_code  = SACT_DONE;
        snp_act_id    = {WID_W{1'b0}};
        if (s2_snp) begin
            snp_act_valid = 1'b1;
            if (s2_snp_hit) begin
                snp_push    = 1'b1;
                snp_present = 1'b1;
                snp_dirty   = s2_hand;
            end else if (mshr_hit) begin
                if (mshr_hasd) begin
                    snp_act_code = SACT_WMSHR;
                    snp_act_id   = mshr_hid;
                end else begin
                    snp_push = 1'b1;
                end
            end else if (vc_hit && vc_state == VC_VALID) begin
                snp_push    = 1'b1;
                snp_present = 1'b1;
                snp_dirty   = vc_dirty;
                snp_from_vc = 1'b1;
                vc_snp      = 1'b1;
            end else if (vc_hit && vc_state == VC_WBPEND) begin
                snp_act_code = SACT_WVC;
                snp_act_id   = vc_slot;
            end else if (vc_hit) begin
                snp_act_code = SACT_WMSHR;
                snp_act_id   = mshr_hid;
            end else begin
                snp_push = 1'b1;
            end
        end
    end
    assign snoop_response_valid  = snp_push;
    assign snoop_response_packet = {snp_present, snp_dirty, s2_line, (snp_from_vc ? vc_data : line_q)};
    assign vc_ins = (s2_refill && s2_vvalid) || (s2_maint && !s2_held && s2_vvalid);
    assign vc_rel = s2_refill && s2_swap && !s2_vvalid;
`ifdef L1D_DEBUG
    function [8*4-1:0] dbg_cls;
        input [REQ_W-1:0] r;
        begin
            if (r[REQ_PF_BIT])        dbg_cls = "PF  ";
            else if (!r[REQ_AMO_BIT]) dbg_cls = r[REQ_RDWR_BIT] ? "ST  " : "LD  ";
            else if (!r[REQ_SEQ_BIT]) dbg_cls = r[REQ_RDWR_BIT] ? "SC  " : "LR  ";
            else                      dbg_cls = r[REQ_RDWR_BIT] ? "AMOW" : "AMOR";
        end
    endfunction
    always @(posedge clk) begin
        if (!rst) begin
            if (s2_pf_op)
                $display("[%0t] L1D B%0d S2 PF   line=%h unique=%b -> %s", $time, BANK_ID, {s2_line, OFF0}, s2_pfu,
                         (pf_result == PF_FB_ISSUED) ? "ISSUED (prefetch MSHR entry)" :
                         (pf_result == PF_FB_IN_CACHE) ? "dropped: in the cache" :
                         (pf_result == PF_FB_IN_FLIGHT) ? "dropped: already in the MSHR" : "dropped: no room");
            if (s2_cpu && s2_kill)
                $display("[%0t] L1D B%0d S2 LSU  entry=%0d idx=%0d ghost (cancelled in R): entry READY again", $time, BANK_ID, s2_seat, s2_idx);
            if (s2_live) begin
                if (s2_tag_blocked)
                    $display("[%0t] L1D B%0d S2 LSU  entry=%0d idx=%0d %0s addr=%h -> TAG ECC %s way %0d, re-issue (WECC)",
                             $time, BANK_ID, s2_seat, s2_idx, dbg_cls(s2_req), s2_addr,
                             tag_fix_valid ? "corrected" : "UNCORRECTABLE", tag_fix_way);
                else if (!s2_cache)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d NC %0s addr=%h -> %s",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr,
                             s3_scfail ? "SC fail (non-cacheable)" :
                             (n_alloc ? "NC queue" : "NC queue full, entry waits (WNC)"));
                else if (s2_scfail)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d SC addr=%h -> FAIL (answer 1)", $time, BANK_ID, s2_idx, s2_addr);
                else if (s2_hit && ecc_data_fix)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> HIT way=%0d DATA ECC corrected (granules %b), re-issue (WECC)",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr, s2_way, hit_fix);
                else if (s2_hit)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> HIT way=%0d data=%h%s",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr, s2_way, s2_rdata,
                             ecc_data_ue ? " (DATA ECC UNCORRECTABLE)" : "");
                else if (mshr_hit)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> MISS, line already in MSHR %0d: entry waits (WMSHR)",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr, mshr_hid);
                else if (vc_hit)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> victim-cache hit slot=%0d state=%0d: answer=%b swap=%b act=%0d",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr, vc_slot, vc_state,
                             s3_load, m_swap, act_code);
                else if (m_alloc)
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> MISS: MSHR entry %0d allocated, %s fetch of line %h",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr, mshr_aid,
                             s2_uniq ? "unique" : "shared", {s2_line, OFF0});
                else
                    $display("[%0t] L1D B%0d S2 LSU  idx=%0d %0s addr=%h -> MISS, all MSHR entries busy: entry waits (WANY)",
                             $time, BANK_ID, s2_idx, dbg_cls(s2_req), s2_addr);
            end
            if (train_now)
                $display("[%0t] L1D B%0d S2 TRAIN port=%0d pc=%h addr=%h %s miss=%b pf_hit=%b%s", $time, BANK_ID, s2_port, s2_seat_pc,
                         s2_addr, s2_store ? "ST" : "LD", !s2_hit, s2_pf_hit,
                         (s2_amo || !s2_cache || (s2_mstid != MID_LSU)) ? " (no train)" : "");
            if (evict_now)
                $display("[%0t] L1D B%0d S2 EVICT line=%h", $time, BANK_ID, {evict_line_now, OFF0});
            if (s2_valid && s2_kind == K_REFILL)
                $display("[%0t] L1D B%0d S2 REFILL mshr=%0d line=%h %s",
                         $time, BANK_ID, s2_mid, {s2_line, OFF0},
                         s2_held ? "held, token goes back to the MSHR" : "INSTALLED (line + tag written), entry freed");
            if (s2_refill && s2_vvalid)
                $display("[%0t] L1D B%0d S2 victim line %h (dirty=%b shared=%b) -> victim-cache slot %0d",
                         $time, BANK_ID, {s2_vline, OFF0}, s2_vdirty, s2_vshared, s2_slot);
            if (s2_maint)
                $display("[%0t] L1D B%0d S2 MAINT %s", $time, BANK_ID,
                         s2_held ? "held (retry)" : (s2_vvalid ? "dirty line -> victim cache (WBPEND)" : "done"));
            if (s2_snp) begin
                if (snp_push)
                    $display("[%0t] L1D B%0d S2 SNOOP %s line=%h -> answer present=%b dirty=%b%s",
                             $time, BANK_ID, s2_sinv ? "INV" : "READ", {s2_line, OFF0}, snp_present, snp_dirty,
                             snp_from_vc ? " (from victim-cache slot)" : (mshr_hit ? " (MSHR still waiting for data)" : ""));
                else
                    $display("[%0t] L1D B%0d S2 SNOOP %s line=%h -> waits for %s %0d",
                             $time, BANK_ID, s2_sinv ? "INV" : "READ", {s2_line, OFF0},
                             snp_act_code == SACT_WVC ? "victim-cache writeback, slot" : "install, MSHR entry",
                             snp_act_id);
                if (vc_hit && vc_state == VC_MOVING && !mshr_hit)
                    $display("[%0t] L1D B%0d S2 SNOOP ERROR MOVING slot without an MSHR entry", $time, BANK_ID);
            end
        end
    end
`endif
endmodule
`default_nettype wire
`endif
