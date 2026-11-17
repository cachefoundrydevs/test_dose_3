`ifndef L1D_REQ_TRACKER_V
`define L1D_REQ_TRACKER_V
`default_nettype none
`include "l1d_defines.vh"
module l1d_req_tracker #(
    `L1D_PARAMS,
    parameter BANK_ID = 0
) (
    clk, rst, arrival_request_0, arrival_request_1, arrival_request_2, arrival_request_3,
    arrival_packet_0, arrival_packet_1, arrival_packet_2, arrival_packet_3, arrival_pc_0, arrival_pc_1,
    arrival_pc_2, arrival_pc_3, drain_request, drain_packet, drain_pc, vec_slot_available,
    vec_write_slot, vec_write_valid, fresh_allowed_q, credit_q_0, credit_q_1, credit_q_2,
    credit_q_3, credit_book_next, fresh_issue, fresh_port_onehot, fresh_entry_onehot, any_pins_arrival, arrival_allocated_0,
    arrival_allocated_1, arrival_allocated_2, arrival_allocated_3, drain_allocated, oldest_ready_valid, oldest_ready_onehot,
    oldest_ready_set, oldest_ready_line, oldest_ready_port, oldest_ready_is_write, oldest_ready_is_line_op, oldest_ready_is_amo,
    arrival_book_valid, arrival_book_entry_onehot, arrival_book_set, arrival_book_line, arrival_book_port, arrival_book_is_write,
    arrival_book_is_line_op, arrival_book_is_amo, arrival_book_credit_ok, book_entry_onehot, issue_entry_onehot, entry_packet_flat, entry_pc_flat,
    s2_cpu_valid, s2_seat, s2_kill, s2_answered, act_valid, act_code,
    act_id, s2_train_sent, s2_seat_trained, s2_seat_pc, mshr_freed, vc_freed,
    nc_freed, free_count_q, entry_valid_q, tracker_empty,
    r_seat, r_seat_deps
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"
    input  wire clk;
    input  wire rst;
    input  wire arrival_request_0;
    input  wire arrival_request_1;
    input  wire arrival_request_2;
    input  wire arrival_request_3;
    input  wire [REQ_W-1:0]  arrival_packet_0;
    input  wire [REQ_W-1:0]  arrival_packet_1;
    input  wire [REQ_W-1:0]  arrival_packet_2;
    input  wire [REQ_W-1:0]  arrival_packet_3;
    input  wire [PC_W-1:0]             arrival_pc_0;
    input  wire [PC_W-1:0]             arrival_pc_1;
    input  wire [PC_W-1:0]             arrival_pc_2;
    input  wire [PC_W-1:0]             arrival_pc_3;
    input  wire drain_request;
    input  wire [REQ_W-1:0]  drain_packet;
    input  wire [PC_W-1:0]             drain_pc;
    input  wire vec_slot_available;
    input  wire [VEC_SLOT_W-1:0] vec_write_slot;
    output wire vec_write_valid;
    input  wire fresh_allowed_q;
    input  wire credit_q_0;
    input  wire credit_q_1;
    input  wire credit_q_2;
    input  wire credit_q_3;
    input  wire [LSU_PORTS-1:0] credit_book_next;
    output wire fresh_issue;
    output wire [3:0]                  fresh_port_onehot;
    output wire [TRK_ENTRIES-1:0]      fresh_entry_onehot;
    output wire any_pins_arrival;
    output wire arrival_allocated_0;
    output wire arrival_allocated_1;
    output wire arrival_allocated_2;
    output wire arrival_allocated_3;
    output wire drain_allocated;
    output wire oldest_ready_valid;
    output wire [TRK_ENTRIES-1:0]      oldest_ready_onehot;
    output wire [INDEX_W-1:0] oldest_ready_set;
    output wire [LINE_ADDR_W-1:0] oldest_ready_line;
    output wire [PORT_W-1:0] oldest_ready_port;
    output wire oldest_ready_is_write;
    output wire oldest_ready_is_line_op;
    output wire oldest_ready_is_amo;
    output wire arrival_book_valid;
    output wire [TRK_ENTRIES-1:0]      arrival_book_entry_onehot;
    output wire [INDEX_W-1:0] arrival_book_set;
    output wire [LINE_ADDR_W-1:0] arrival_book_line;
    output wire [PORT_W-1:0] arrival_book_port;
    output wire arrival_book_is_write;
    output wire arrival_book_is_line_op;
    output wire arrival_book_is_amo;
    output wire arrival_book_credit_ok;
    input  wire [TRK_ENTRIES-1:0]      book_entry_onehot;
    input  wire [TRK_ENTRIES-1:0]      issue_entry_onehot;
    output wire [TRK_ENTRIES*REQ_W-1:0] entry_packet_flat;
    output wire [TRK_ENTRIES*PC_W-1:0] entry_pc_flat;
    input  wire s2_cpu_valid;
    input  wire [SEAT_W-1:0] s2_seat;
    input  wire s2_kill;
    input  wire s2_answered;
    input  wire act_valid;
    input  wire [2:0]                  act_code;
    input  wire [WID_W-1:0]  act_id;
    input  wire s2_train_sent;
    output wire s2_seat_trained;
    output wire [PC_W-1:0]             s2_seat_pc;
    input  wire [MSHR_ENTRIES-1:0]     mshr_freed;
    input  wire [VC_SLOTS-1:0]         vc_freed;
    input  wire nc_freed;
    input  wire [SEAT_W-1:0] r_seat;
    output wire [TRK_ENTRIES-1:0] r_seat_deps;
    output reg  [TRK_CNT_W-1:0] free_count_q;
    output wire [TRK_ENTRIES-1:0]      entry_valid_q;
    output wire tracker_empty;
    localparam NE          = TRK_ENTRIES;
    localparam NS          = LSU_PORTS + 1;
    localparam RANK_W      = 3;
    localparam NA          = (TRK_ALLOC_MAX < NS) ? TRK_ALLOC_MAX : NS;
    localparam SET_IN_REQ  = REQ_ADDR_LSB + OFFSET_W + LA_SET_LSB;
    localparam LINE_IN_REQ = REQ_ADDR_LSB + OFFSET_W;
    localparam [TRK_CNT_W-1:0] NE_C = NE;
    localparam [REQ_W-1:0] SLOT_MASK = {{(REQ_W-VEC_SLOT_W){1'b0}}, {VEC_SLOT_W{1'b1}}} << REQ_DATA_LSB;
    reg  [NE-1:0]           valid_q;
    reg  [NE-1:0]           valid_dup_q;
    reg  [TRK_STATE_W-1:0]  state_q [0:NE-1];
    reg  [REQ_W-1:0]        packet_q [0:NE-1];
    reg  [PC_W-1:0]         pc_q [0:NE-1];
    reg  [INDEX_W-1:0]      set_q [0:NE-1];
    reg  [NE-1:0]           write_q;
    reg  [NE-1:0]           deps_q [0:NE-1];
    reg  [MSHR_ENTRIES-1:0] wait_mshr_q [0:NE-1];
    reg  [VC_SLOTS-1:0]     wait_vc_q [0:NE-1];
    reg  [NE-1:0]           wait_nc_q;
    reg  [NE-1:0]           trained_q;
    wire [NS-1:0]       src_valid;
    wire [NS*REQ_W-1:0] src_packet;
    wire [NS*PC_W-1:0]  src_pc;
    wire [NS-1:0]       src_write;
    wire [NS-1:0]       src_line_op;
    wire [NS-1:0]       src_line_store;
    wire [NS-1:0]       src_place_ok;
    wire [NS-1:0]       eligible;
    wire [NS*INDEX_W-1:0] src_set;
    wire [NS-1:0]       src_credit_ok;
    assign src_valid  = {drain_request, arrival_request_3, arrival_request_2, arrival_request_1, arrival_request_0};
    assign src_packet = {drain_packet, arrival_packet_3, arrival_packet_2, arrival_packet_1, arrival_packet_0};
    assign src_pc     = {drain_pc, arrival_pc_3, arrival_pc_2, arrival_pc_1, arrival_pc_0};
    genvar src_idx;
    generate
        for (src_idx = 0; src_idx < NS; src_idx = src_idx + 1) begin : g_src
            assign src_write[src_idx]      = src_packet[src_idx*REQ_W + REQ_RDWR_BIT];
            assign src_line_op[src_idx]    = src_packet[src_idx*REQ_W + REQ_LINE_BIT];
            assign src_line_store[src_idx] = src_line_op[src_idx] & src_write[src_idx];
            assign src_place_ok[src_idx]   = !src_line_store[src_idx] | vec_slot_available;
            assign eligible[src_idx]       = src_valid[src_idx] & src_place_ok[src_idx];
            assign src_set[src_idx*INDEX_W +: INDEX_W] = src_packet[src_idx*REQ_W + SET_IN_REQ +: INDEX_W];
            assign src_credit_ok[src_idx]  = credit_book_next[src_packet[src_idx*REQ_W + REQ_PORT_LSB +: PORT_W]];
        end
    endgenerate
    assign any_pins_arrival = |src_valid[LSU_PORTS-1:0];
    reg  [RANK_W-1:0] rank [0:NS-1];
    integer rank_src;
    integer rank_lower;
    always @* begin
        for (rank_src = 0; rank_src < NS; rank_src = rank_src + 1) begin
            rank[rank_src] = {RANK_W{1'b0}};
            for (rank_lower = 0; rank_lower < rank_src; rank_lower = rank_lower + 1)
                rank[rank_src] = eligible[rank_lower] ?
                                 {rank[rank_src][1] | rank[rank_src][2],
                                  rank[rank_src][0], ~rank[rank_src][2]} :
                                 rank[rank_src];
        end
    end
    wire [NS-1:0] alloc;
    wire [NE:0]   free_room;
    reg  [NE:0]   free_room_q;
    reg  [NE:0]   free_room_dup_q;
    assign free_room = free_room_q;
    genvar al_idx;
    generate
        for (al_idx = 0; al_idx < NS; al_idx = al_idx + 1) begin : g_alloc
            wire rank_in_room;
            wire rank_in_max;
            assign rank_in_room = ((rank[al_idx] == 3'b000) && free_room[0]) ||
                                  ((rank[al_idx] == 3'b001) && free_room[1]) ||
                                  ((rank[al_idx] == 3'b011) && free_room[2]) ||
                                  ((rank[al_idx] == 3'b111) && free_room[3]) ||
                                  ((rank[al_idx] == 3'b110) && free_room[4]);
            assign rank_in_max  = ((rank[al_idx] == 3'b000) && (0 < NA)) ||
                                  ((rank[al_idx] == 3'b001) && (1 < NA)) ||
                                  ((rank[al_idx] == 3'b011) && (2 < NA)) ||
                                  ((rank[al_idx] == 3'b111) && (3 < NA)) ||
                                  ((rank[al_idx] == 3'b110) && (4 < NA));
            assign alloc[al_idx] = eligible[al_idx] & rank_in_room & rank_in_max;
        end
    endgenerate
    assign arrival_allocated_0 = alloc[0];
    assign arrival_allocated_1 = alloc[1];
    assign arrival_allocated_2 = alloc[2];
    assign arrival_allocated_3 = alloc[3];
    assign drain_allocated     = alloc[LSU_PORTS];
    assign vec_write_valid     = |(alloc & src_line_store);
    wire [NA*NE-1:0] free_pick;
    reg  [NA*NE-1:0] free_pick_q;
    generate
        if (NA <= 2) begin : g_pick_chain
            wire [NE-1:0] free_now;
            wire [NE-1:0] first_free;
            wire [NE-1:0] rest_free;
            wire [NE-1:0] second_free;
            assign free_now = ~valid_q;
            genvar ch_idx;
            for (ch_idx = 0; ch_idx < NE; ch_idx = ch_idx + 1) begin : g_chain
                if (ch_idx == 0) begin : g_first
                    assign first_free[ch_idx]  = free_now[ch_idx];
                    assign second_free[ch_idx] = 1'b0;
                end else begin : g_rest
                    assign first_free[ch_idx]  = free_now[ch_idx] & ~|free_now[ch_idx-1:0];
                    assign second_free[ch_idx] = rest_free[ch_idx] & ~|rest_free[ch_idx-1:0];
                end
            end
            assign rest_free = free_now & ~first_free;
            assign free_pick[0 +: NE] = first_free;
            if (NA == 2) begin : g_two
                assign free_pick[NE +: NE] = second_free;
            end
        end else begin : g_pick_count
            reg [RANK_W-1:0] free_below [0:NE-1];
            integer fb_entry;
            integer fb_lower;
            always @* begin
                for (fb_entry = 0; fb_entry < NE; fb_entry = fb_entry + 1) begin
                    free_below[fb_entry] = {RANK_W{1'b0}};
                    for (fb_lower = 0; fb_lower < fb_entry; fb_lower = fb_lower + 1)
                        free_below[fb_entry] = ~valid_q[fb_lower] ?
                                               {free_below[fb_entry][1] | free_below[fb_entry][2],
                                                free_below[fb_entry][0], ~free_below[fb_entry][2]} :
                                               free_below[fb_entry];
                end
            end
            genvar pk_k, pk_e;
            for (pk_k = 0; pk_k < NA; pk_k = pk_k + 1) begin : g_pick_k
                for (pk_e = 0; pk_e < NE; pk_e = pk_e + 1) begin : g_pick_e
                    assign free_pick[pk_k*NE + pk_e] = free_pick_q[pk_k*NE + pk_e];
                end
            end
        end
    endgenerate
    reg  [NA-1:0] rank_onehot [0:NS-1];
    reg  [NA-1:0] rank_alloc;
    integer ro_src;
    integer ro_rank;
    always @* begin
        rank_alloc = {NA{1'b0}};
        for (ro_src = 0; ro_src < NS; ro_src = ro_src + 1)
            for (ro_rank = 0; ro_rank < NA; ro_rank = ro_rank + 1) begin
                rank_onehot[ro_src][ro_rank] = eligible[ro_src] &&
                    (rank[ro_src] == ((ro_rank < RANK_W) ? ((1 << ro_rank) - 1) :
                                      (((1 << RANK_W) - 1) & ~((1 << (ro_rank - RANK_W)) - 1))));
                rank_alloc[ro_rank] = rank_alloc[ro_rank] | (rank_onehot[ro_src][ro_rank] & free_room_dup_q[ro_rank]);
            end
    end
    reg  [NE-1:0] src_entry [0:NS-1];
    reg  [NS-1:0] data_sel [0:NE-1];
    reg  [NE-1:0] alloc_en;
    reg  [NA*NE-1:0] age_groups;
    integer se_src;
    integer se_rank;
    integer se_entry;
    always @* begin
        for (se_src = 0; se_src < NS; se_src = se_src + 1) begin
            src_entry[se_src] = {NE{1'b0}};
            for (se_rank = 0; se_rank < NA; se_rank = se_rank + 1)
                if (rank[se_src] == ((se_rank < RANK_W) ? ((1 << se_rank) - 1) :
                                     (((1 << RANK_W) - 1) & ~((1 << (se_rank - RANK_W)) - 1))))
                    src_entry[se_src] = src_entry[se_src] | free_pick[se_rank*NE +: NE];
        end
        for (se_entry = 0; se_entry < NE; se_entry = se_entry + 1) begin
            alloc_en[se_entry] = 1'b0;
            for (se_src = 0; se_src < NS; se_src = se_src + 1) begin
                data_sel[se_entry][se_src] = eligible[se_src] & src_entry[se_src][se_entry];
                alloc_en[se_entry] = alloc_en[se_entry] |
                    ((se_src < NA) &&
                     rank_alloc[(se_src < NA) ? se_src : 0] &&
                     free_pick[((se_src < NA) ? se_src : 0)*NE + se_entry]);
            end
        end
        for (se_rank = 0; se_rank < NA; se_rank = se_rank + 1) begin
            age_groups[se_rank*NE +: NE] = {NE{1'b0}};
            for (se_src = 0; se_src < NS; se_src = se_src + 1)
                if ((se_src == se_rank) && rank_alloc[se_rank])
                    age_groups[se_rank*NE +: NE] = age_groups[se_rank*NE +: NE] | free_pick[se_rank*NE +: NE];
        end
    end
    reg  [REQ_W-1:0] load_packet [0:NE-1];
    reg  [PC_W-1:0]  load_pc [0:NE-1];
    reg  [NE-1:0]    load_line_store;
    integer ld_entry;
    integer ld_src;
    always @* begin
        for (ld_entry = 0; ld_entry < NE; ld_entry = ld_entry + 1) begin
            load_packet[ld_entry]     = {REQ_W{1'b0}};
            load_pc[ld_entry]         = {PC_W{1'b0}};
            load_line_store[ld_entry] = 1'b0;
            for (ld_src = 0; ld_src < NS; ld_src = ld_src + 1) begin
                load_packet[ld_entry] = load_packet[ld_entry] | ({REQ_W{data_sel[ld_entry][ld_src]}} & src_packet[ld_src*REQ_W +: REQ_W]);
                load_pc[ld_entry]     = load_pc[ld_entry]     | ({PC_W{data_sel[ld_entry][ld_src]}}  & src_pc[ld_src*PC_W +: PC_W]);
                load_line_store[ld_entry] = load_line_store[ld_entry] | (data_sel[ld_entry][ld_src] & src_line_store[ld_src]);
            end
            if (load_line_store[ld_entry])
                load_packet[ld_entry] = (load_packet[ld_entry] & ~SLOT_MASK) |
                                        ({{(REQ_W-VEC_SLOT_W){1'b0}}, vec_write_slot} << REQ_DATA_LSB);
        end
    end
    wire [3:0] credit_q;
    reg  [LSU_PORTS-1:0] lowest_req;
    reg  [LSU_PORTS-1:0] lowest_port;
    reg                  lowest_credit;
    reg                  lower_req_seen;
    integer lo_port;
    assign credit_q = {credit_q_3, credit_q_2, credit_q_1, credit_q_0};
    always @* begin
        lowest_credit  = 1'b0;
        lower_req_seen = 1'b0;
        for (lo_port = 0; lo_port < LSU_PORTS; lo_port = lo_port + 1) begin
            lowest_req[lo_port]  = src_valid[lo_port] & ~|(src_valid[LSU_PORTS-1:0] & ~({LSU_PORTS{1'b1}} << lo_port));
            lower_req_seen       = lower_req_seen | src_valid[lo_port];
            lowest_port[lo_port] = lowest_req[lo_port] & alloc[lo_port];
            lowest_credit        = lowest_credit | (lowest_port[lo_port] & credit_q[lo_port]);
        end
    end
    assign fresh_issue        = fresh_allowed_q & (|lowest_port) & lowest_credit;
    assign fresh_port_onehot  = lowest_port;
    assign fresh_entry_onehot = free_pick[0 +: NE];
    wire [NE-1:0] freed_now;
    reg  [NE-1:0] deps_new [0:NS-1];
    reg           same_set_pair;
    integer dp_src;
    integer dp_entry;
    integer dp_lower;
    reg  [NE-1:0] deps_old [0:NS-1];
    reg  [NS-1:0] lower_pair [0:NS-1];
    always @* begin
        for (dp_src = 0; dp_src < NS; dp_src = dp_src + 1) begin
            for (dp_entry = 0; dp_entry < NE; dp_entry = dp_entry + 1)
                deps_old[dp_src][dp_entry] = valid_q[dp_entry] & ~freed_now[dp_entry] &
                                             (set_q[dp_entry] == src_set[dp_src*INDEX_W +: INDEX_W]) &
                                             (write_q[dp_entry] | src_write[dp_src]);
            deps_new[dp_src]   = deps_old[dp_src];
            lower_pair[dp_src] = {NS{1'b0}};
            for (dp_lower = 0; dp_lower < dp_src; dp_lower = dp_lower + 1) begin
                same_set_pair = alloc[dp_lower] &&
                                (src_set[dp_lower*INDEX_W +: INDEX_W] == src_set[dp_src*INDEX_W +: INDEX_W]) &&
                                (src_write[dp_lower] || src_write[dp_src]);
                lower_pair[dp_src][dp_lower] = eligible[dp_lower] &&
                    (src_set[dp_lower*INDEX_W +: INDEX_W] == src_set[dp_src*INDEX_W +: INDEX_W]) &&
                    (src_write[dp_lower] || src_write[dp_src]);
                if (same_set_pair)
                    deps_new[dp_src] = deps_new[dp_src] | src_entry[dp_lower];
            end
        end
    end
    reg  [NE-1:0] rank_deps_old [0:NA-1];
    reg  [NA-1:0] rank_pair [0:NA-1];
    reg  [NE-1:0] deps_load [0:NE-1];
    integer rd_rank;
    integer rd_src;
    integer rd_lower;
    integer dl_entry;
    integer dl_rank;
    integer dl_lower;
    always @* begin
        for (rd_rank = 0; rd_rank < NA; rd_rank = rd_rank + 1) begin
            rank_deps_old[rd_rank] = {NE{1'b0}};
            rank_pair[rd_rank]     = {NA{1'b0}};
            for (rd_src = 0; rd_src < NS; rd_src = rd_src + 1) begin
                if (rank_onehot[rd_src][rd_rank])
                    rank_deps_old[rd_rank] = rank_deps_old[rd_rank] | deps_old[rd_src];
                for (rd_lower = 0; rd_lower < NS; rd_lower = rd_lower + 1)
                    if (rank_onehot[rd_src][rd_rank] && lower_pair[rd_src][rd_lower])
                        rank_pair[rd_rank] = rank_pair[rd_rank] | rank_onehot[rd_lower];
            end
        end
        for (dl_entry = 0; dl_entry < NE; dl_entry = dl_entry + 1) begin
            deps_load[dl_entry] = {NE{1'b0}};
            for (dl_rank = 0; dl_rank < NA; dl_rank = dl_rank + 1) begin
                if (free_pick[dl_rank*NE + dl_entry])
                    deps_load[dl_entry] = deps_load[dl_entry] | rank_deps_old[dl_rank];
                for (dl_lower = 0; dl_lower < NA; dl_lower = dl_lower + 1)
                    if (free_pick[dl_rank*NE + dl_entry] && rank_pair[dl_rank][dl_lower])
                        deps_load[dl_entry] = deps_load[dl_entry] | free_pick[dl_lower*NE +: NE];
            end
        end
    end
    reg  [NS-1:0] arrival_cand;
    reg  [NS-1:0] arrival_pick;
    reg           arrival_seen;
    integer ab_src;
    reg  [NA-1:0] pick_any;
    reg  [NS-1:0] pair_block;
    reg  [NS-1:0] cand_nofresh;
    reg  [NS-1:0] cand_fresh;
    reg  [NS-1:0] pick_nofresh;
    reg  [NS-1:0] pick_fresh;
    integer pb_rank;
    integer pb_lower;
    always @* begin
        for (pb_rank = 0; pb_rank < NA; pb_rank = pb_rank + 1)
            pick_any[pb_rank] = |free_pick[pb_rank*NE +: NE];
        arrival_seen = 1'b0;
        for (ab_src = 0; ab_src < NS; ab_src = ab_src + 1) begin
            pair_block[ab_src] = 1'b0;
            for (pb_lower = 0; pb_lower < NS; pb_lower = pb_lower + 1)
                if (lower_pair[ab_src][pb_lower])
                    pair_block[ab_src] = pair_block[ab_src] | (|(rank_onehot[pb_lower] & pick_any));
            cand_nofresh[ab_src] = alloc[ab_src] && (deps_old[ab_src] == {NE{1'b0}}) && !pair_block[ab_src];
            cand_fresh[ab_src]   = cand_nofresh[ab_src] &&
                                   !((rank[ab_src] == {RANK_W{1'b0}}) && (ab_src < LSU_PORTS));
        end
        for (ab_src = 0; ab_src < NS; ab_src = ab_src + 1) begin
            pick_nofresh[ab_src] = cand_nofresh[ab_src] && !(|(cand_nofresh & ~({NS{1'b1}} << ab_src)));
            pick_fresh[ab_src]   = cand_fresh[ab_src]   && !(|(cand_fresh   & ~({NS{1'b1}} << ab_src)));
            arrival_cand[ab_src] = fresh_issue ? cand_fresh[ab_src] : cand_nofresh[ab_src];
            arrival_pick[ab_src] = fresh_issue ? pick_fresh[ab_src] : pick_nofresh[ab_src];
            arrival_seen         = arrival_seen || arrival_cand[ab_src];
        end
    end
    reg  [NE-1:0]          ab_entry;
    reg  [REQ_W-1:0]       ab_packet;
    reg  [NE-1:0]          ab_entry_nofresh;
    reg  [NE-1:0]          ab_entry_fresh;
    reg  [REQ_W-1:0]       ab_packet_nofresh;
    reg  [REQ_W-1:0]       ab_packet_fresh;
    integer ab_mux;
    always @* begin
        ab_entry_nofresh  = {NE{1'b0}};
        ab_entry_fresh    = {NE{1'b0}};
        ab_packet_nofresh = {REQ_W{1'b0}};
        ab_packet_fresh   = {REQ_W{1'b0}};
        for (ab_mux = 0; ab_mux < NS; ab_mux = ab_mux + 1) begin
            ab_entry_nofresh  = ab_entry_nofresh  | ({NE{pick_nofresh[ab_mux]}}    & src_entry[ab_mux]);
            ab_entry_fresh    = ab_entry_fresh    | ({NE{pick_fresh[ab_mux]}}      & src_entry[ab_mux]);
            ab_packet_nofresh = ab_packet_nofresh | ({REQ_W{pick_nofresh[ab_mux]}} & src_packet[ab_mux*REQ_W +: REQ_W]);
            ab_packet_fresh   = ab_packet_fresh   | ({REQ_W{pick_fresh[ab_mux]}}   & src_packet[ab_mux*REQ_W +: REQ_W]);
        end
        ab_entry  = fresh_issue ? ab_entry_fresh  : ab_entry_nofresh;
        ab_packet = fresh_issue ? ab_packet_fresh : ab_packet_nofresh;
    end
    assign arrival_book_valid        = |arrival_pick;
    assign arrival_book_entry_onehot = ab_entry;
    assign arrival_book_set          = ab_packet[SET_IN_REQ +: INDEX_W];
    assign arrival_book_line         = ab_packet[LINE_IN_REQ +: LINE_ADDR_W];
    assign arrival_book_port         = ab_packet[REQ_PORT_LSB +: PORT_W];
    assign arrival_book_is_write     = ab_packet[REQ_RDWR_BIT];
    assign arrival_book_is_line_op   = ab_packet[REQ_LINE_BIT];
    assign arrival_book_is_amo       = ab_packet[REQ_AMO_BIT] & ab_packet[REQ_SEQ_BIT];
    assign arrival_book_credit_ok    = |(arrival_pick & src_credit_ok);
    reg  [NE-1:0] ready_cand;
    integer rc_entry;
    always @* begin
        for (rc_entry = 0; rc_entry < NE; rc_entry = rc_entry + 1)
            ready_cand[rc_entry] = valid_q[rc_entry] & state_q[rc_entry][TRK_READY_B] & (deps_q[rc_entry] == {NE{1'b0}});
    end
    wire [NE*NE-1:0] older_than_flat;
    wire [SEAT_W-1:0] oldest_index_unused;
    l1d_age_matrix #(.N(NE), .IDX_W(SEAT_W), .ONEHOT_ALLOC(1), .ONEHOT_ENC(1), .MULTI_ALLOC(1), .MULTI_GROUPS(NA)) u_age (
        .clk                   (clk),
        .rst                   (rst),
        .allocate_valid        (1'b0),
        .allocate_index        ({SEAT_W{1'b0}}),
        .allocate_onehot       ({NE{1'b0}}),
        .allocate_group_onehot (age_groups),
        .candidate_mask        (ready_cand),
        .older_than_flat       (older_than_flat),
        .oldest_onehot         (oldest_ready_onehot),
        .oldest_index          (oldest_index_unused),
        .any_candidate         (oldest_ready_valid)
    );
    reg  [REQ_W-1:0] or_packet;
    integer or_entry;
    always @* begin
        or_packet = {REQ_W{1'b0}};
        for (or_entry = 0; or_entry < NE; or_entry = or_entry + 1)
            or_packet = or_packet | ({REQ_W{oldest_ready_onehot[or_entry]}} & packet_q[or_entry]);
    end
    assign oldest_ready_set        = or_packet[SET_IN_REQ +: INDEX_W];
    assign oldest_ready_line       = or_packet[LINE_IN_REQ +: LINE_ADDR_W];
    assign oldest_ready_port       = or_packet[REQ_PORT_LSB +: PORT_W];
    assign oldest_ready_is_write   = or_packet[REQ_RDWR_BIT];
    assign oldest_ready_is_line_op = or_packet[REQ_LINE_BIT];
    assign oldest_ready_is_amo     = or_packet[REQ_AMO_BIT] & or_packet[REQ_SEQ_BIT];
    genvar fl_idx;
    generate
        for (fl_idx = 0; fl_idx < NE; fl_idx = fl_idx + 1) begin : g_flat
            assign entry_packet_flat[fl_idx*REQ_W +: REQ_W] = packet_q[fl_idx];
            assign entry_pc_flat[fl_idx*PC_W +: PC_W]       = pc_q[fl_idx];
        end
    endgenerate
    reg  [NE-1:0] s2_entry_onehot;
    integer s2_dec;
    always @* begin
        for (s2_dec = 0; s2_dec < NE; s2_dec = s2_dec + 1)
            s2_entry_onehot[s2_dec] = s2_cpu_valid && (s2_seat == s2_dec);
    end
    wire s2_free_entry;
    wire s2_to_ready;
    wire s2_to_wait;
    assign s2_free_entry = !s2_kill && (s2_answered || (act_valid && (act_code == ACT_OWNED)));
    assign s2_to_ready   = s2_kill || (act_valid && (act_code == ACT_WECC));
    assign s2_to_wait    = !s2_kill && act_valid && ((act_code == ACT_WMSHR) || (act_code == ACT_WANY) ||
                                                     (act_code == ACT_WVC)   || (act_code == ACT_WNC));
    assign freed_now = s2_entry_onehot & {NE{s2_free_entry}};
    wire [MSHR_ENTRIES-1:0] push_mshr_mask;
    wire [VC_SLOTS-1:0]     push_vc_mask;
    wire                    push_nc_bit;
    wire                    push_ready_now;
    assign push_mshr_mask = (act_code == ACT_WMSHR) ? ({{(MSHR_ENTRIES-1){1'b0}}, 1'b1} << act_id[MSHR_ID_W-1:0]) :
                            (act_code == ACT_WANY)  ? {MSHR_ENTRIES{1'b1}} : {MSHR_ENTRIES{1'b0}};
    assign push_vc_mask   = (act_code == ACT_WVC) ? ({{(VC_SLOTS-1){1'b0}}, 1'b1} << act_id[VC_ID_W-1:0]) : {VC_SLOTS{1'b0}};
    assign push_nc_bit    = (act_code == ACT_WNC);
    assign push_ready_now = (|(push_mshr_mask & mshr_freed)) || (|(push_vc_mask & vc_freed)) || (push_nc_bit && nc_freed);
    assign s2_seat_trained = trained_q[s2_seat];
    assign s2_seat_pc      = pc_q[s2_seat];
    assign r_seat_deps     = deps_q[r_seat];
    reg  [RANK_W-1:0]    alloc_cnt;
    integer ac_src;
    always @* begin
        alloc_cnt = {RANK_W{1'b0}};
        for (ac_src = 0; ac_src < NS; ac_src = ac_src + 1)
            alloc_cnt = alloc[ac_src] ? {rank[ac_src][2], ~rank[ac_src][2] & rank[ac_src][0],
                                         ~rank[ac_src][0] | (rank[ac_src][1] & ~rank[ac_src][2])} : alloc_cnt;
    end
    reg  [TRK_CNT_W-1:0] free_count_pick;
    reg                  s2_frees;
    integer fc_count;
    reg  [TRK_CNT_W-1:0] count_plus;
    reg  [TRK_CNT_W-1:0] count_minus;
    reg  [NE:0]          free_room_next;
    reg  [NE:0]          room_now;
    reg  [NE:0]          room_plus;
    reg  [NE:0]          room_minus;
    always @* begin
        s2_frees        = s2_cpu_valid && s2_free_entry && (s2_seat < NE);
        free_count_pick = {TRK_CNT_W{1'b0}};
        free_room_next  = {(NE+1){1'b0}};
        room_now        = ~({(NE+1){1'b1}} << free_count_q);
        for (fc_count = 0; fc_count <= NA; fc_count = fc_count + 1) begin
            count_plus  = free_count_q + 1 - fc_count;
            count_minus = free_count_q - fc_count;
            if ((fc_count > 0) && !room_now[(fc_count > 0) ? fc_count - 1 : 0])
                room_minus = {(NE+1){1'b1}};
            else
                room_minus = room_now >> fc_count;
            if (fc_count == 0)
                room_plus = (free_count_q == {TRK_CNT_W{1'b1}}) ? {(NE+1){1'b0}} :
                            ((room_now << 1) | {{NE{1'b0}}, 1'b1});
            else if ((fc_count > 1) && !room_now[(fc_count > 1) ? fc_count - 2 : 0])
                room_plus = {(NE+1){1'b1}};
            else
                room_plus = room_now >> ((fc_count > 0) ? fc_count - 1 : 0);
            if (((fc_count == 0)  || rank_alloc[(fc_count > 0) ? fc_count - 1 : 0]) &&
                ((fc_count == NA) || !rank_alloc[(fc_count < NA) ? fc_count : 0])) begin
                free_count_pick = free_count_pick | (s2_frees ? count_plus : count_minus);
                free_room_next  = free_room_next | (s2_frees ? room_plus : room_minus);
            end
        end
    end
    always @(posedge clk) begin : free_room_proc
        if (rst) begin
            free_room_q     <= ~({(NE+1){1'b1}} << NE_C);
            free_room_dup_q <= ~({(NE+1){1'b1}} << NE_C);
        end else begin
            free_room_q     <= free_room_next;
            free_room_dup_q <= free_room_next;
        end
    end
    wire [TRK_CNT_W-1:0] free_count_next;
    assign free_count_next = free_count_pick;
    localparam GE_W = 2 * NA + 1;
    reg  [GE_W-1:0]  free_ge [0:NE-1];
    reg  [2*NA*NE-1:0] free_rank;
    reg  [GE_W-1:0]  freed_ge;
    reg              freed_any;
    reg  [NA:0]      alloc_n_onehot;
    reg  [NA*NE-1:0] free_pick_next;
    integer fg_entry;
    integer fg_lower;
    integer fr_rank;
    integer fn_pick;
    integer fn_count;
    localparam KS_LV = $clog2(NE);
    reg  [GE_W-1:0] ks_ge [0:(KS_LV+1)*NE-1];
    integer ks_lv;
    integer ks_entry;
    integer ks_bit;
    integer ks_part;
    always @* begin
        for (ks_entry = 0; ks_entry < NE; ks_entry = ks_entry + 1)
            ks_ge[ks_entry] = {{(GE_W-2){1'b0}}, !valid_dup_q[ks_entry], 1'b1};
        for (ks_lv = 1; ks_lv <= KS_LV; ks_lv = ks_lv + 1)
            for (ks_entry = 0; ks_entry < NE; ks_entry = ks_entry + 1) begin
                ks_ge[ks_lv*NE + ks_entry] = {GE_W{1'b0}};
                for (ks_bit = 0; ks_bit < GE_W; ks_bit = ks_bit + 1)
                    for (ks_part = 0; ks_part <= ks_bit; ks_part = ks_part + 1)
                        ks_ge[ks_lv*NE + ks_entry][ks_bit] = ks_ge[ks_lv*NE + ks_entry][ks_bit] |
                            (ks_ge[(ks_lv-1)*NE + ks_entry][ks_part] &
                             ((ks_entry >= (1 << (ks_lv-1))) ?
                                  ks_ge[(ks_lv-1)*NE + ((ks_entry >= (1 << (ks_lv-1))) ? ks_entry - (1 << (ks_lv-1)) : 0)][ks_bit-ks_part] :
                                  (ks_bit == ks_part)));
            end
    end
    always @* begin
        for (fg_entry = 0; fg_entry < NE; fg_entry = fg_entry + 1) begin
            free_ge[fg_entry] = (fg_entry == 0) ? {{(GE_W-1){1'b0}}, 1'b1} :
                                ks_ge[KS_LV*NE + ((fg_entry > 0) ? fg_entry - 1 : 0)];
            for (fr_rank = 0; fr_rank < 2 * NA; fr_rank = fr_rank + 1)
                free_rank[fr_rank*NE + fg_entry] = !valid_dup_q[fg_entry] && free_ge[fg_entry][fr_rank] &&
                                                   !free_ge[fg_entry][fr_rank+1];
        end
        freed_any = |(freed_now & valid_q);
        freed_ge  = {GE_W{1'b0}};
        for (fg_entry = 0; fg_entry < NE; fg_entry = fg_entry + 1)
            if (freed_now[fg_entry] && valid_q[fg_entry])
                freed_ge = freed_ge | free_ge[fg_entry];
        for (fn_count = 0; fn_count <= NA; fn_count = fn_count + 1)
            alloc_n_onehot[fn_count] = ((fn_count == 0) || rank_alloc[(fn_count > 0) ? fn_count - 1 : 0]) &&
                                       ((fn_count == NA) || !rank_alloc[(fn_count < NA) ? fn_count : 0]);
        free_pick_next = {NA*NE{1'b0}};
        for (fn_pick = 0; fn_pick < NA; fn_pick = fn_pick + 1)
            for (fn_count = 0; fn_count <= NA; fn_count = fn_count + 1)
                if (alloc_n_onehot[fn_count]) begin
                    if (freed_any && ((fn_pick == 0) || freed_ge[fn_pick+fn_count]) && !freed_ge[fn_pick+fn_count+1])
                        free_pick_next[fn_pick*NE +: NE] = free_pick_next[fn_pick*NE +: NE] | (freed_now & valid_q);
                    else if (freed_any && (fn_pick > 0) && !freed_ge[fn_pick+fn_count])
                        free_pick_next[fn_pick*NE +: NE] = free_pick_next[fn_pick*NE +: NE] |
                            free_rank[(((fn_pick > 0) ? fn_pick - 1 : 0) + fn_count)*NE +: NE];
                    else if (fn_pick + fn_count < 2 * NA)
                        free_pick_next[fn_pick*NE +: NE] = free_pick_next[fn_pick*NE +: NE] |
                            free_rank[(fn_pick+fn_count)*NE +: NE];
                end
    end
    integer fq_pick;
    always @(posedge clk) begin : free_pick_proc
        if (rst) begin
            for (fq_pick = 0; fq_pick < NA; fq_pick = fq_pick + 1)
                free_pick_q[fq_pick*NE +: NE] <= {{(NE-1){1'b0}}, 1'b1} << fq_pick;
        end else begin
            free_pick_q <= free_pick_next;
        end
    end
    integer ent;
    always @(posedge clk) begin : tracker_proc
        if (rst) begin
            valid_q      <= {NE{1'b0}};
            valid_dup_q  <= {NE{1'b0}};
            write_q      <= {NE{1'b0}};
            wait_nc_q    <= {NE{1'b0}};
            trained_q    <= {NE{1'b0}};
            free_count_q <= NE_C;
            for (ent = 0; ent < NE; ent = ent + 1) begin
                state_q[ent]     <= {TRK_STATE_W{1'b0}};
                deps_q[ent]      <= {NE{1'b0}};
                wait_mshr_q[ent] <= {MSHR_ENTRIES{1'b0}};
                wait_vc_q[ent]   <= {VC_SLOTS{1'b0}};
            end
        end else begin
            free_count_q <= free_count_next;
            valid_dup_q  <= valid_q;
            for (ent = 0; ent < NE; ent = ent + 1) begin
                if (!valid_q[ent]) begin
                    packet_q[ent] <= load_packet[ent];
                    pc_q[ent]     <= load_pc[ent];
                    set_q[ent]    <= load_packet[ent][SET_IN_REQ +: INDEX_W];
                    write_q[ent]  <= load_packet[ent][REQ_RDWR_BIT];
                end
                if (alloc_en[ent]) begin
                    valid_q[ent]     <= 1'b1;
                    valid_dup_q[ent] <= 1'b1;
                    trained_q[ent]   <= 1'b0;
                    wait_nc_q[ent]   <= 1'b0;
                    wait_mshr_q[ent] <= {MSHR_ENTRIES{1'b0}};
                    wait_vc_q[ent]   <= {VC_SLOTS{1'b0}};
                    state_q[ent]     <= {TRK_STATE_W{1'b0}};
                    if (fresh_issue && fresh_entry_onehot[ent])
                        state_q[ent][TRK_INFL_B]   <= 1'b1;
                    else if (book_entry_onehot[ent])
                        state_q[ent][TRK_BOOKED_B] <= 1'b1;
                    else
                        state_q[ent][TRK_READY_B]  <= 1'b1;
                    deps_q[ent] <= deps_load[ent];
                end else if (valid_q[ent]) begin
                    deps_q[ent] <= deps_q[ent] & ~freed_now;
                    if (s2_entry_onehot[ent]) begin
                        if (s2_free_entry) begin
                            valid_q[ent] <= 1'b0;
                            valid_dup_q[ent] <= 1'b0;
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                        end else if (s2_to_ready) begin
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                            state_q[ent][TRK_READY_B] <= 1'b1;
                        end else if (s2_to_wait) begin
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                            if (push_ready_now) state_q[ent][TRK_READY_B] <= 1'b1;
                            else                state_q[ent][TRK_WAIT_B]  <= 1'b1;
                            wait_mshr_q[ent] <= push_mshr_mask;
                            wait_vc_q[ent]   <= push_vc_mask;
                            wait_nc_q[ent]   <= push_nc_bit;
                        end
                        if (s2_train_sent) trained_q[ent] <= 1'b1;
                    end else begin
                        if (state_q[ent][TRK_WAIT_B] &&
                            ((|(wait_mshr_q[ent] & mshr_freed)) || (|(wait_vc_q[ent] & vc_freed)) || (wait_nc_q[ent] && nc_freed))) begin
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                            state_q[ent][TRK_READY_B] <= 1'b1;
                        end
                        if (book_entry_onehot[ent] && state_q[ent][TRK_READY_B]) begin
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                            state_q[ent][TRK_BOOKED_B] <= 1'b1;
                        end
                        if (issue_entry_onehot[ent]) begin
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                            state_q[ent][TRK_INFL_B] <= 1'b1;
                        end
                    end
                end
            end
        end
    end
    assign entry_valid_q = valid_q;
    assign tracker_empty = ~|valid_q;
`ifndef SYNTHESIS
    integer chk_src;
    integer chk_entry;
    integer chk_hits;
    initial if (LSU_PORTS != 4)
        $display("L1D TRACKER ERROR bank %0d: LSU_PORTS = %0d, the ports _0 .. _3 need LSU_PORTS = 4", BANK_ID, LSU_PORTS);
    always @(posedge clk) begin
        if (!rst) begin
            if (drain_request && any_pins_arrival)
                $display("[%0t] L1D TRACKER ERROR bank %0d: pins arrival and drain in the same cycle (front rule A1 broken)", $time, BANK_ID);
            for (chk_entry = 0; chk_entry < NE; chk_entry = chk_entry + 1) begin
                chk_hits = 0;
                for (chk_src = 0; chk_src < NS; chk_src = chk_src + 1)
                    if (alloc[chk_src] && src_entry[chk_src][chk_entry]) chk_hits = chk_hits + 1;
                if (chk_hits > 1)
                    $display("[%0t] L1D TRACKER ERROR bank %0d: entry %0d allocated by %0d sources at once", $time, BANK_ID, chk_entry, chk_hits);
                if (alloc_en[chk_entry] && valid_q[chk_entry])
                    $display("[%0t] L1D TRACKER ERROR bank %0d: entry %0d allocated while in use", $time, BANK_ID, chk_entry);
            end
            for (chk_src = 0; chk_src < NS; chk_src = chk_src + 1)
                if (alloc[chk_src] && (src_entry[chk_src] == {NE{1'b0}}))
                    $display("[%0t] L1D TRACKER ERROR bank %0d: source %0d allocated without a free entry (request lost)", $time, BANK_ID, chk_src);
            if (s2_cpu_valid && !valid_q[s2_seat])
                $display("[%0t] L1D TRACKER ERROR bank %0d: S2 result for entry %0d, which is free", $time, BANK_ID, s2_seat);
            if (s2_cpu_valid && !state_q[s2_seat][TRK_INFL_B])
                $display("[%0t] L1D TRACKER ERROR bank %0d: S2 result for entry %0d, which is not in flight", $time, BANK_ID, s2_seat);
            if ((|book_entry_onehot) && (|(book_entry_onehot & issue_entry_onehot)))
                $display("[%0t] L1D TRACKER ERROR bank %0d: an entry is booked and issued in one cycle", $time, BANK_ID);
        end
    end
`endif
`ifdef L1D_DEBUG
    always @(posedge clk) begin
        if (!rst) begin
            for (chk_src = 0; chk_src < NS; chk_src = chk_src + 1)
                if (alloc[chk_src])
                    $display("[%0t] L1D TRACKER bank %0d: source %0d (port %0d) -> entry %b rank %0d%s deps=%b", $time, BANK_ID, chk_src,
                             src_packet[chk_src*REQ_W + REQ_PORT_LSB +: PORT_W], src_entry[chk_src], rank[chk_src],
                             (fresh_issue && rank[chk_src] == 0 && chk_src < LSU_PORTS) ? " FRESH" : "", deps_new[chk_src]);
            if (|freed_now)
                $display("[%0t] L1D TRACKER bank %0d: entry %0d freed (free next = %0d)", $time, BANK_ID, s2_seat, free_count_next);
        end
    end
`endif
endmodule
`default_nettype wire
`endif
