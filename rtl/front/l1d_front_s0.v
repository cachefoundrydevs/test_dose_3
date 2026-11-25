`include "l1d_defines.vh"
module l1d_front_s0 #(
    `L1D_PARAMS
) (
    clk,
    rst,
    port_request_valid_0,
    port_request_packet_0,
    port_request_pc_0,
    port_stop_0,
    port_request_valid_1,
    port_request_packet_1,
    port_request_pc_1,
    port_stop_1,
    port_request_valid_2,
    port_request_packet_2,
    port_request_pc_2,
    port_stop_2,
    port_request_valid_3,
    port_request_packet_3,
    port_request_pc_3,
    port_stop_3,
    port_request_line_3,
    bank_vec_slot_available,
    b0_lsu_response_fifo_free_0,
    b0_lsu_response_fifo_free_1,
    b0_lsu_response_fifo_free_2,
    b0_lsu_response_fifo_free_3,
    b0_lsu_response_fifo_free_next_0,
    b0_lsu_response_fifo_free_next_1,
    b0_lsu_response_fifo_free_next_2,
    b0_lsu_response_fifo_free_next_3,
    b0_snoop_request_valid,
    b0_snoop_request_line_addr,
    b0_snoop_request_invalidate,
    b0_snoop_request_ready,
    b0_snoop_response_fifo_free,
    b0_prefetch_request_valid,
    b0_prefetch_request_line_addr,
    b0_prefetch_request_unique,
    b0_prefetch_request_ready,
    b0_refill_handover_busy,
    b0_s1_valid,
    b0_s1_kind,
    b0_s1_req,
    b0_s2_valid,
    b0_s2_kind,
    b0_s2_req,
    b0_s2_hit,
    b0_s2_sid,
    b0_s2_line,
    b0_s2_seat,
    b0_s2_kill,
    b0_s2_answered,
    b0_act_valid,
    b0_act_code,
    b0_act_id,
    b0_snp_act_valid,
    b0_snp_act_code,
    b0_snp_act_id,
    b0_s2_train_sent,
    b0_s2_seat_trained,
    b0_s2_seat_pc,
    b0_s3_amo_v,
    b0_lock_valid,
    b0_lock_snp,
    b0_lock_line,
    b0_ev_amor,
    b0_tok_valid,
    b0_tok_id,
    b0_tok_line,
    b0_mt_valid,
    b0_mt_set,
    b0_mt_way,
    b0_mt_op,
    b0_m_resp_valid,
    b0_n_resp_valid,
    b0_mshr_freed,
    b0_vc_freed,
    b0_nc_freed,
    b0_any_gnt,
    b0_g_set,
    b0_gnt_refill,
    b0_gnt_maint,
    b0_r_valid,
    b0_r_kind,
    b0_r_seat,
    b0_r_req,
    b0_r_fast,
    b0_r_mid,
    b0_r_line,
    b0_r_mway,
    b0_r_mop,
    b0_r_sinv,
    b0_r_sid,
    b0_r_deps,
    b0_tracker_empty,
    b0_miss_ok,
    b0_vec_write_slot,
    b0_vec_write_valid,
    b1_lsu_response_fifo_free_0,
    b1_lsu_response_fifo_free_1,
    b1_lsu_response_fifo_free_2,
    b1_lsu_response_fifo_free_3,
    b1_lsu_response_fifo_free_next_0,
    b1_lsu_response_fifo_free_next_1,
    b1_lsu_response_fifo_free_next_2,
    b1_lsu_response_fifo_free_next_3,
    b1_snoop_request_valid,
    b1_snoop_request_line_addr,
    b1_snoop_request_invalidate,
    b1_snoop_request_ready,
    b1_snoop_response_fifo_free,
    b1_prefetch_request_valid,
    b1_prefetch_request_line_addr,
    b1_prefetch_request_unique,
    b1_prefetch_request_ready,
    b1_refill_handover_busy,
    b1_s1_valid,
    b1_s1_kind,
    b1_s1_req,
    b1_s2_valid,
    b1_s2_kind,
    b1_s2_req,
    b1_s2_hit,
    b1_s2_sid,
    b1_s2_line,
    b1_s2_seat,
    b1_s2_kill,
    b1_s2_answered,
    b1_act_valid,
    b1_act_code,
    b1_act_id,
    b1_snp_act_valid,
    b1_snp_act_code,
    b1_snp_act_id,
    b1_s2_train_sent,
    b1_s2_seat_trained,
    b1_s2_seat_pc,
    b1_s3_amo_v,
    b1_lock_valid,
    b1_lock_snp,
    b1_lock_line,
    b1_ev_amor,
    b1_tok_valid,
    b1_tok_id,
    b1_tok_line,
    b1_mt_valid,
    b1_mt_set,
    b1_mt_way,
    b1_mt_op,
    b1_m_resp_valid,
    b1_n_resp_valid,
    b1_mshr_freed,
    b1_vc_freed,
    b1_nc_freed,
    b1_any_gnt,
    b1_g_set,
    b1_gnt_refill,
    b1_gnt_maint,
    b1_r_valid,
    b1_r_kind,
    b1_r_seat,
    b1_r_req,
    b1_r_fast,
    b1_r_mid,
    b1_r_line,
    b1_r_mway,
    b1_r_mop,
    b1_r_sinv,
    b1_r_sid,
    b1_r_deps,
    b1_tracker_empty,
    b1_miss_ok,
    b1_vec_write_slot,
    b1_vec_write_valid,
    b2_lsu_response_fifo_free_0,
    b2_lsu_response_fifo_free_1,
    b2_lsu_response_fifo_free_2,
    b2_lsu_response_fifo_free_3,
    b2_lsu_response_fifo_free_next_0,
    b2_lsu_response_fifo_free_next_1,
    b2_lsu_response_fifo_free_next_2,
    b2_lsu_response_fifo_free_next_3,
    b2_snoop_request_valid,
    b2_snoop_request_line_addr,
    b2_snoop_request_invalidate,
    b2_snoop_request_ready,
    b2_snoop_response_fifo_free,
    b2_prefetch_request_valid,
    b2_prefetch_request_line_addr,
    b2_prefetch_request_unique,
    b2_prefetch_request_ready,
    b2_refill_handover_busy,
    b2_s1_valid,
    b2_s1_kind,
    b2_s1_req,
    b2_s2_valid,
    b2_s2_kind,
    b2_s2_req,
    b2_s2_hit,
    b2_s2_sid,
    b2_s2_line,
    b2_s2_seat,
    b2_s2_kill,
    b2_s2_answered,
    b2_act_valid,
    b2_act_code,
    b2_act_id,
    b2_snp_act_valid,
    b2_snp_act_code,
    b2_snp_act_id,
    b2_s2_train_sent,
    b2_s2_seat_trained,
    b2_s2_seat_pc,
    b2_s3_amo_v,
    b2_lock_valid,
    b2_lock_snp,
    b2_lock_line,
    b2_ev_amor,
    b2_tok_valid,
    b2_tok_id,
    b2_tok_line,
    b2_mt_valid,
    b2_mt_set,
    b2_mt_way,
    b2_mt_op,
    b2_m_resp_valid,
    b2_n_resp_valid,
    b2_mshr_freed,
    b2_vc_freed,
    b2_nc_freed,
    b2_any_gnt,
    b2_g_set,
    b2_gnt_refill,
    b2_gnt_maint,
    b2_r_valid,
    b2_r_kind,
    b2_r_seat,
    b2_r_req,
    b2_r_fast,
    b2_r_mid,
    b2_r_line,
    b2_r_mway,
    b2_r_mop,
    b2_r_sinv,
    b2_r_sid,
    b2_r_deps,
    b2_tracker_empty,
    b2_miss_ok,
    b2_vec_write_slot,
    b2_vec_write_valid,
    b3_lsu_response_fifo_free_0,
    b3_lsu_response_fifo_free_1,
    b3_lsu_response_fifo_free_2,
    b3_lsu_response_fifo_free_3,
    b3_lsu_response_fifo_free_next_0,
    b3_lsu_response_fifo_free_next_1,
    b3_lsu_response_fifo_free_next_2,
    b3_lsu_response_fifo_free_next_3,
    b3_snoop_request_valid,
    b3_snoop_request_line_addr,
    b3_snoop_request_invalidate,
    b3_snoop_request_ready,
    b3_snoop_response_fifo_free,
    b3_prefetch_request_valid,
    b3_prefetch_request_line_addr,
    b3_prefetch_request_unique,
    b3_prefetch_request_ready,
    b3_refill_handover_busy,
    b3_s1_valid,
    b3_s1_kind,
    b3_s1_req,
    b3_s2_valid,
    b3_s2_kind,
    b3_s2_req,
    b3_s2_hit,
    b3_s2_sid,
    b3_s2_line,
    b3_s2_seat,
    b3_s2_kill,
    b3_s2_answered,
    b3_act_valid,
    b3_act_code,
    b3_act_id,
    b3_snp_act_valid,
    b3_snp_act_code,
    b3_snp_act_id,
    b3_s2_train_sent,
    b3_s2_seat_trained,
    b3_s2_seat_pc,
    b3_s3_amo_v,
    b3_lock_valid,
    b3_lock_snp,
    b3_lock_line,
    b3_ev_amor,
    b3_tok_valid,
    b3_tok_id,
    b3_tok_line,
    b3_mt_valid,
    b3_mt_set,
    b3_mt_way,
    b3_mt_op,
    b3_m_resp_valid,
    b3_n_resp_valid,
    b3_mshr_freed,
    b3_vc_freed,
    b3_nc_freed,
    b3_any_gnt,
    b3_g_set,
    b3_gnt_refill,
    b3_gnt_maint,
    b3_r_valid,
    b3_r_kind,
    b3_r_seat,
    b3_r_req,
    b3_r_fast,
    b3_r_mid,
    b3_r_line,
    b3_r_mway,
    b3_r_mop,
    b3_r_sinv,
    b3_r_sid,
    b3_r_deps,
    b3_tracker_empty,
    b3_miss_ok,
    b3_vec_write_slot,
    b3_vec_write_valid
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"
    localparam NB = NUM_BANKS;
    input  wire clk;
    input  wire rst;
    input  wire port_request_valid_0;
    input  wire [REQ_W-1:0] port_request_packet_0;
    input  wire [PC_W-1:0] port_request_pc_0;
    output wire port_stop_0;
    input  wire port_request_valid_1;
    input  wire [REQ_W-1:0] port_request_packet_1;
    input  wire [PC_W-1:0] port_request_pc_1;
    output wire port_stop_1;
    input  wire port_request_valid_2;
    input  wire [REQ_W-1:0] port_request_packet_2;
    input  wire [PC_W-1:0] port_request_pc_2;
    output wire port_stop_2;
    input  wire port_request_valid_3;
    input  wire [REQ_W-1:0] port_request_packet_3;
    input  wire [PC_W-1:0] port_request_pc_3;
    output wire port_stop_3;
    input  wire [LINE_W-1:0] port_request_line_3;
    input  wire [NB-1:0] bank_vec_slot_available;
    input  wire [RCNT_W-1:0] b0_lsu_response_fifo_free_0;
    input  wire [RCNT_W-1:0] b0_lsu_response_fifo_free_1;
    input  wire [RCNT_W-1:0] b0_lsu_response_fifo_free_2;
    input  wire [RCNT_W-1:0] b0_lsu_response_fifo_free_3;
    input  wire [RCNT_W-1:0] b0_lsu_response_fifo_free_next_0;
    input  wire [RCNT_W-1:0] b0_lsu_response_fifo_free_next_1;
    input  wire [RCNT_W-1:0] b0_lsu_response_fifo_free_next_2;
    input  wire [RCNT_W-1:0] b0_lsu_response_fifo_free_next_3;
    input  wire  b0_snoop_request_valid;
    input  wire [LINE_ADDR_W-1:0] b0_snoop_request_line_addr;
    input  wire  b0_snoop_request_invalidate;
    output wire  b0_snoop_request_ready;
    input  wire [SNPR_CNT_W-1:0] b0_snoop_response_fifo_free;
    input  wire  b0_prefetch_request_valid;
    input  wire [LINE_ADDR_W-1:0] b0_prefetch_request_line_addr;
    input  wire  b0_prefetch_request_unique;
    output wire  b0_prefetch_request_ready;
    input  wire  b0_refill_handover_busy;
    input  wire  b0_s1_valid;
    input  wire [1:0] b0_s1_kind;
    input  wire [REQ_W-1:0] b0_s1_req;
    input  wire  b0_s2_valid;
    input  wire [1:0] b0_s2_kind;
    input  wire [REQ_W-1:0] b0_s2_req;
    input  wire  b0_s2_hit;
    input  wire [SNP_ID_W-1:0] b0_s2_sid;
    input  wire [LINE_ADDR_W-1:0] b0_s2_line;
    input  wire [SEAT_W-1:0] b0_s2_seat;
    input  wire  b0_s2_kill;
    input  wire  b0_s2_answered;
    input  wire  b0_act_valid;
    input  wire [2:0] b0_act_code;
    input  wire [WID_W-1:0] b0_act_id;
    input  wire  b0_snp_act_valid;
    input  wire [1:0] b0_snp_act_code;
    input  wire [WID_W-1:0] b0_snp_act_id;
    input  wire  b0_s2_train_sent;
    output wire  b0_s2_seat_trained;
    output wire [PC_W-1:0] b0_s2_seat_pc;
    input  wire  b0_s3_amo_v;
    input  wire  b0_lock_valid;
    input  wire  b0_lock_snp;
    input  wire [LINE_ADDR_W-1:0] b0_lock_line;
    input  wire  b0_ev_amor;
    input  wire  b0_tok_valid;
    input  wire [MSHR_ID_W-1:0] b0_tok_id;
    input  wire [LINE_ADDR_W-1:0] b0_tok_line;
    input  wire  b0_mt_valid;
    input  wire [INDEX_W-1:0] b0_mt_set;
    input  wire [WAY_W-1:0] b0_mt_way;
    input  wire  b0_mt_op;
    input  wire  b0_m_resp_valid;
    input  wire  b0_n_resp_valid;
    input  wire [MSHR_ENTRIES-1:0] b0_mshr_freed;
    input  wire [VC_SLOTS-1:0] b0_vc_freed;
    input  wire  b0_nc_freed;
    output wire  b0_any_gnt;
    output wire [INDEX_W-1:0] b0_g_set;
    output wire  b0_gnt_refill;
    output wire  b0_gnt_maint;
    output wire  b0_r_valid;
    output wire [1:0] b0_r_kind;
    output wire [SEAT_W-1:0] b0_r_seat;
    output wire [REQ_W-1:0] b0_r_req;
    output wire  b0_r_fast;
    output wire [MSHR_ID_W-1:0] b0_r_mid;
    output wire [LINE_ADDR_W-1:0] b0_r_line;
    output wire [WAY_W-1:0] b0_r_mway;
    output wire  b0_r_mop;
    output wire  b0_r_sinv;
    output wire [SNP_ID_W-1:0] b0_r_sid;
    output wire [TRK_ENTRIES-1:0] b0_r_deps;
    output wire  b0_tracker_empty;
    output wire  b0_miss_ok;
    input  wire [VEC_SLOT_W-1:0] b0_vec_write_slot;
    output wire  b0_vec_write_valid;
    input  wire [RCNT_W-1:0] b1_lsu_response_fifo_free_0;
    input  wire [RCNT_W-1:0] b1_lsu_response_fifo_free_1;
    input  wire [RCNT_W-1:0] b1_lsu_response_fifo_free_2;
    input  wire [RCNT_W-1:0] b1_lsu_response_fifo_free_3;
    input  wire [RCNT_W-1:0] b1_lsu_response_fifo_free_next_0;
    input  wire [RCNT_W-1:0] b1_lsu_response_fifo_free_next_1;
    input  wire [RCNT_W-1:0] b1_lsu_response_fifo_free_next_2;
    input  wire [RCNT_W-1:0] b1_lsu_response_fifo_free_next_3;
    input  wire  b1_snoop_request_valid;
    input  wire [LINE_ADDR_W-1:0] b1_snoop_request_line_addr;
    input  wire  b1_snoop_request_invalidate;
    output wire  b1_snoop_request_ready;
    input  wire [SNPR_CNT_W-1:0] b1_snoop_response_fifo_free;
    input  wire  b1_prefetch_request_valid;
    input  wire [LINE_ADDR_W-1:0] b1_prefetch_request_line_addr;
    input  wire  b1_prefetch_request_unique;
    output wire  b1_prefetch_request_ready;
    input  wire  b1_refill_handover_busy;
    input  wire  b1_s1_valid;
    input  wire [1:0] b1_s1_kind;
    input  wire [REQ_W-1:0] b1_s1_req;
    input  wire  b1_s2_valid;
    input  wire [1:0] b1_s2_kind;
    input  wire [REQ_W-1:0] b1_s2_req;
    input  wire  b1_s2_hit;
    input  wire [SNP_ID_W-1:0] b1_s2_sid;
    input  wire [LINE_ADDR_W-1:0] b1_s2_line;
    input  wire [SEAT_W-1:0] b1_s2_seat;
    input  wire  b1_s2_kill;
    input  wire  b1_s2_answered;
    input  wire  b1_act_valid;
    input  wire [2:0] b1_act_code;
    input  wire [WID_W-1:0] b1_act_id;
    input  wire  b1_snp_act_valid;
    input  wire [1:0] b1_snp_act_code;
    input  wire [WID_W-1:0] b1_snp_act_id;
    input  wire  b1_s2_train_sent;
    output wire  b1_s2_seat_trained;
    output wire [PC_W-1:0] b1_s2_seat_pc;
    input  wire  b1_s3_amo_v;
    input  wire  b1_lock_valid;
    input  wire  b1_lock_snp;
    input  wire [LINE_ADDR_W-1:0] b1_lock_line;
    input  wire  b1_ev_amor;
    input  wire  b1_tok_valid;
    input  wire [MSHR_ID_W-1:0] b1_tok_id;
    input  wire [LINE_ADDR_W-1:0] b1_tok_line;
    input  wire  b1_mt_valid;
    input  wire [INDEX_W-1:0] b1_mt_set;
    input  wire [WAY_W-1:0] b1_mt_way;
    input  wire  b1_mt_op;
    input  wire  b1_m_resp_valid;
    input  wire  b1_n_resp_valid;
    input  wire [MSHR_ENTRIES-1:0] b1_mshr_freed;
    input  wire [VC_SLOTS-1:0] b1_vc_freed;
    input  wire  b1_nc_freed;
    output wire  b1_any_gnt;
    output wire [INDEX_W-1:0] b1_g_set;
    output wire  b1_gnt_refill;
    output wire  b1_gnt_maint;
    output wire  b1_r_valid;
    output wire [1:0] b1_r_kind;
    output wire [SEAT_W-1:0] b1_r_seat;
    output wire [REQ_W-1:0] b1_r_req;
    output wire  b1_r_fast;
    output wire [MSHR_ID_W-1:0] b1_r_mid;
    output wire [LINE_ADDR_W-1:0] b1_r_line;
    output wire [WAY_W-1:0] b1_r_mway;
    output wire  b1_r_mop;
    output wire  b1_r_sinv;
    output wire [SNP_ID_W-1:0] b1_r_sid;
    output wire [TRK_ENTRIES-1:0] b1_r_deps;
    output wire  b1_tracker_empty;
    output wire  b1_miss_ok;
    input  wire [VEC_SLOT_W-1:0] b1_vec_write_slot;
    output wire  b1_vec_write_valid;
    input  wire [RCNT_W-1:0] b2_lsu_response_fifo_free_0;
    input  wire [RCNT_W-1:0] b2_lsu_response_fifo_free_1;
    input  wire [RCNT_W-1:0] b2_lsu_response_fifo_free_2;
    input  wire [RCNT_W-1:0] b2_lsu_response_fifo_free_3;
    input  wire [RCNT_W-1:0] b2_lsu_response_fifo_free_next_0;
    input  wire [RCNT_W-1:0] b2_lsu_response_fifo_free_next_1;
    input  wire [RCNT_W-1:0] b2_lsu_response_fifo_free_next_2;
    input  wire [RCNT_W-1:0] b2_lsu_response_fifo_free_next_3;
    input  wire  b2_snoop_request_valid;
    input  wire [LINE_ADDR_W-1:0] b2_snoop_request_line_addr;
    input  wire  b2_snoop_request_invalidate;
    output wire  b2_snoop_request_ready;
    input  wire [SNPR_CNT_W-1:0] b2_snoop_response_fifo_free;
    input  wire  b2_prefetch_request_valid;
    input  wire [LINE_ADDR_W-1:0] b2_prefetch_request_line_addr;
    input  wire  b2_prefetch_request_unique;
    output wire  b2_prefetch_request_ready;
    input  wire  b2_refill_handover_busy;
    input  wire  b2_s1_valid;
    input  wire [1:0] b2_s1_kind;
    input  wire [REQ_W-1:0] b2_s1_req;
    input  wire  b2_s2_valid;
    input  wire [1:0] b2_s2_kind;
    input  wire [REQ_W-1:0] b2_s2_req;
    input  wire  b2_s2_hit;
    input  wire [SNP_ID_W-1:0] b2_s2_sid;
    input  wire [LINE_ADDR_W-1:0] b2_s2_line;
    input  wire [SEAT_W-1:0] b2_s2_seat;
    input  wire  b2_s2_kill;
    input  wire  b2_s2_answered;
    input  wire  b2_act_valid;
    input  wire [2:0] b2_act_code;
    input  wire [WID_W-1:0] b2_act_id;
    input  wire  b2_snp_act_valid;
    input  wire [1:0] b2_snp_act_code;
    input  wire [WID_W-1:0] b2_snp_act_id;
    input  wire  b2_s2_train_sent;
    output wire  b2_s2_seat_trained;
    output wire [PC_W-1:0] b2_s2_seat_pc;
    input  wire  b2_s3_amo_v;
    input  wire  b2_lock_valid;
    input  wire  b2_lock_snp;
    input  wire [LINE_ADDR_W-1:0] b2_lock_line;
    input  wire  b2_ev_amor;
    input  wire  b2_tok_valid;
    input  wire [MSHR_ID_W-1:0] b2_tok_id;
    input  wire [LINE_ADDR_W-1:0] b2_tok_line;
    input  wire  b2_mt_valid;
    input  wire [INDEX_W-1:0] b2_mt_set;
    input  wire [WAY_W-1:0] b2_mt_way;
    input  wire  b2_mt_op;
    input  wire  b2_m_resp_valid;
    input  wire  b2_n_resp_valid;
    input  wire [MSHR_ENTRIES-1:0] b2_mshr_freed;
    input  wire [VC_SLOTS-1:0] b2_vc_freed;
    input  wire  b2_nc_freed;
    output wire  b2_any_gnt;
    output wire [INDEX_W-1:0] b2_g_set;
    output wire  b2_gnt_refill;
    output wire  b2_gnt_maint;
    output wire  b2_r_valid;
    output wire [1:0] b2_r_kind;
    output wire [SEAT_W-1:0] b2_r_seat;
    output wire [REQ_W-1:0] b2_r_req;
    output wire  b2_r_fast;
    output wire [MSHR_ID_W-1:0] b2_r_mid;
    output wire [LINE_ADDR_W-1:0] b2_r_line;
    output wire [WAY_W-1:0] b2_r_mway;
    output wire  b2_r_mop;
    output wire  b2_r_sinv;
    output wire [SNP_ID_W-1:0] b2_r_sid;
    output wire [TRK_ENTRIES-1:0] b2_r_deps;
    output wire  b2_tracker_empty;
    output wire  b2_miss_ok;
    input  wire [VEC_SLOT_W-1:0] b2_vec_write_slot;
    output wire  b2_vec_write_valid;
    input  wire [RCNT_W-1:0] b3_lsu_response_fifo_free_0;
    input  wire [RCNT_W-1:0] b3_lsu_response_fifo_free_1;
    input  wire [RCNT_W-1:0] b3_lsu_response_fifo_free_2;
    input  wire [RCNT_W-1:0] b3_lsu_response_fifo_free_3;
    input  wire [RCNT_W-1:0] b3_lsu_response_fifo_free_next_0;
    input  wire [RCNT_W-1:0] b3_lsu_response_fifo_free_next_1;
    input  wire [RCNT_W-1:0] b3_lsu_response_fifo_free_next_2;
    input  wire [RCNT_W-1:0] b3_lsu_response_fifo_free_next_3;
    input  wire  b3_snoop_request_valid;
    input  wire [LINE_ADDR_W-1:0] b3_snoop_request_line_addr;
    input  wire  b3_snoop_request_invalidate;
    output wire  b3_snoop_request_ready;
    input  wire [SNPR_CNT_W-1:0] b3_snoop_response_fifo_free;
    input  wire  b3_prefetch_request_valid;
    input  wire [LINE_ADDR_W-1:0] b3_prefetch_request_line_addr;
    input  wire  b3_prefetch_request_unique;
    output wire  b3_prefetch_request_ready;
    input  wire  b3_refill_handover_busy;
    input  wire  b3_s1_valid;
    input  wire [1:0] b3_s1_kind;
    input  wire [REQ_W-1:0] b3_s1_req;
    input  wire  b3_s2_valid;
    input  wire [1:0] b3_s2_kind;
    input  wire [REQ_W-1:0] b3_s2_req;
    input  wire  b3_s2_hit;
    input  wire [SNP_ID_W-1:0] b3_s2_sid;
    input  wire [LINE_ADDR_W-1:0] b3_s2_line;
    input  wire [SEAT_W-1:0] b3_s2_seat;
    input  wire  b3_s2_kill;
    input  wire  b3_s2_answered;
    input  wire  b3_act_valid;
    input  wire [2:0] b3_act_code;
    input  wire [WID_W-1:0] b3_act_id;
    input  wire  b3_snp_act_valid;
    input  wire [1:0] b3_snp_act_code;
    input  wire [WID_W-1:0] b3_snp_act_id;
    input  wire  b3_s2_train_sent;
    output wire  b3_s2_seat_trained;
    output wire [PC_W-1:0] b3_s2_seat_pc;
    input  wire  b3_s3_amo_v;
    input  wire  b3_lock_valid;
    input  wire  b3_lock_snp;
    input  wire [LINE_ADDR_W-1:0] b3_lock_line;
    input  wire  b3_ev_amor;
    input  wire  b3_tok_valid;
    input  wire [MSHR_ID_W-1:0] b3_tok_id;
    input  wire [LINE_ADDR_W-1:0] b3_tok_line;
    input  wire  b3_mt_valid;
    input  wire [INDEX_W-1:0] b3_mt_set;
    input  wire [WAY_W-1:0] b3_mt_way;
    input  wire  b3_mt_op;
    input  wire  b3_m_resp_valid;
    input  wire  b3_n_resp_valid;
    input  wire [MSHR_ENTRIES-1:0] b3_mshr_freed;
    input  wire [VC_SLOTS-1:0] b3_vc_freed;
    input  wire  b3_nc_freed;
    output wire  b3_any_gnt;
    output wire [INDEX_W-1:0] b3_g_set;
    output wire  b3_gnt_refill;
    output wire  b3_gnt_maint;
    output wire  b3_r_valid;
    output wire [1:0] b3_r_kind;
    output wire [SEAT_W-1:0] b3_r_seat;
    output wire [REQ_W-1:0] b3_r_req;
    output wire  b3_r_fast;
    output wire [MSHR_ID_W-1:0] b3_r_mid;
    output wire [LINE_ADDR_W-1:0] b3_r_line;
    output wire [WAY_W-1:0] b3_r_mway;
    output wire  b3_r_mop;
    output wire  b3_r_sinv;
    output wire [SNP_ID_W-1:0] b3_r_sid;
    output wire [TRK_ENTRIES-1:0] b3_r_deps;
    output wire  b3_tracker_empty;
    output wire  b3_miss_ok;
    input  wire [VEC_SLOT_W-1:0] b3_vec_write_slot;
    output wire  b3_vec_write_valid;
    wire [NB*TRK_CNT_W-1:0] bank_tracker_free_count;
    wire [NB-1:0] bank_arrival_request_0;
    wire [NB-1:0] bank_arrival_allocated_0;
    wire [NB-1:0] bank_arrival_request_1;
    wire [NB-1:0] bank_arrival_allocated_1;
    wire [NB-1:0] bank_arrival_request_2;
    wire [NB-1:0] bank_arrival_allocated_2;
    wire [NB-1:0] bank_arrival_request_3;
    wire [NB-1:0] bank_arrival_allocated_3;
    wire [NB-1:0] bank_drain_request;
    wire [NB*REQ_W-1:0] bank_drain_packet;
    wire [NB*PC_W-1:0] bank_drain_pc;
    wire [NB*LINE_W-1:0] bank_arrival_line_data;
    wire [NB-1:0] bank_drain_allocated;
    l1d_port_front #(`L1D_PARAMS_PASS) u_front (
        .clk(clk), .rst(rst),
        .port_request_valid_0(port_request_valid_0), .port_request_packet_0(port_request_packet_0), .port_request_pc_0(port_request_pc_0), .port_stop_0(port_stop_0),
        .port_request_valid_1(port_request_valid_1), .port_request_packet_1(port_request_packet_1), .port_request_pc_1(port_request_pc_1), .port_stop_1(port_stop_1),
        .port_request_valid_2(port_request_valid_2), .port_request_packet_2(port_request_packet_2), .port_request_pc_2(port_request_pc_2), .port_stop_2(port_stop_2),
        .port_request_valid_3(port_request_valid_3), .port_request_packet_3(port_request_packet_3), .port_request_pc_3(port_request_pc_3), .port_stop_3(port_stop_3),
        .port_request_line_3(port_request_line_3), .bank_tracker_free_count(bank_tracker_free_count), .bank_vec_slot_available(bank_vec_slot_available),
        .bank_arrival_request_0(bank_arrival_request_0), .bank_arrival_allocated_0(bank_arrival_allocated_0),
        .bank_arrival_request_1(bank_arrival_request_1), .bank_arrival_allocated_1(bank_arrival_allocated_1),
        .bank_arrival_request_2(bank_arrival_request_2), .bank_arrival_allocated_2(bank_arrival_allocated_2),
        .bank_arrival_request_3(bank_arrival_request_3), .bank_arrival_allocated_3(bank_arrival_allocated_3),
        .bank_drain_request(bank_drain_request), .bank_drain_packet(bank_drain_packet), .bank_drain_pc(bank_drain_pc),
        .bank_arrival_line_data(bank_arrival_line_data), .bank_drain_allocated(bank_drain_allocated)
    );
    l1d_bank_s0 #(`L1D_PARAMS_PASS, .BANK_ID(0)) u_s0_0 (
        .clk(clk), .rst(rst),
        .arrival_request_0(bank_arrival_request_0[0]), .arrival_packet_0(port_request_packet_0), .arrival_pc_0(port_request_pc_0), .arrival_allocated_0(bank_arrival_allocated_0[0]),
        .arrival_request_1(bank_arrival_request_1[0]), .arrival_packet_1(port_request_packet_1), .arrival_pc_1(port_request_pc_1), .arrival_allocated_1(bank_arrival_allocated_1[0]),
        .arrival_request_2(bank_arrival_request_2[0]), .arrival_packet_2(port_request_packet_2), .arrival_pc_2(port_request_pc_2), .arrival_allocated_2(bank_arrival_allocated_2[0]),
        .arrival_request_3(bank_arrival_request_3[0]), .arrival_packet_3(port_request_packet_3), .arrival_pc_3(port_request_pc_3), .arrival_allocated_3(bank_arrival_allocated_3[0]),
        .drain_request(bank_drain_request[0]), .drain_packet(bank_drain_packet[0*REQ_W +: REQ_W]), .drain_pc(bank_drain_pc[0*PC_W +: PC_W]),
        .drain_allocated(bank_drain_allocated[0]), .tracker_free_count_q(bank_tracker_free_count[0*TRK_CNT_W +: TRK_CNT_W]),
        .vec_slot_available(bank_vec_slot_available[0]),
        .lsu_response_fifo_free_0(b0_lsu_response_fifo_free_0),
        .lsu_response_fifo_free_1(b0_lsu_response_fifo_free_1),
        .lsu_response_fifo_free_2(b0_lsu_response_fifo_free_2),
        .lsu_response_fifo_free_3(b0_lsu_response_fifo_free_3),
        .lsu_response_fifo_free_next_0(b0_lsu_response_fifo_free_next_0),
        .lsu_response_fifo_free_next_1(b0_lsu_response_fifo_free_next_1),
        .lsu_response_fifo_free_next_2(b0_lsu_response_fifo_free_next_2),
        .lsu_response_fifo_free_next_3(b0_lsu_response_fifo_free_next_3),
        .snoop_request_valid(b0_snoop_request_valid),
        .snoop_request_line_addr(b0_snoop_request_line_addr),
        .snoop_request_invalidate(b0_snoop_request_invalidate),
        .snoop_request_ready(b0_snoop_request_ready),
        .snoop_response_fifo_free(b0_snoop_response_fifo_free),
        .prefetch_request_valid(b0_prefetch_request_valid),
        .prefetch_request_line_addr(b0_prefetch_request_line_addr),
        .prefetch_request_unique(b0_prefetch_request_unique),
        .prefetch_request_ready(b0_prefetch_request_ready),
        .refill_handover_busy(b0_refill_handover_busy),
        .s1_valid(b0_s1_valid),
        .s1_kind(b0_s1_kind),
        .s1_req(b0_s1_req),
        .s2_valid(b0_s2_valid),
        .s2_kind(b0_s2_kind),
        .s2_req(b0_s2_req),
        .s2_hit(b0_s2_hit),
        .s2_sid(b0_s2_sid),
        .s2_line(b0_s2_line),
        .s2_seat(b0_s2_seat),
        .s2_kill(b0_s2_kill),
        .s2_answered(b0_s2_answered),
        .act_valid(b0_act_valid),
        .act_code(b0_act_code),
        .act_id(b0_act_id),
        .snp_act_valid(b0_snp_act_valid),
        .snp_act_code(b0_snp_act_code),
        .snp_act_id(b0_snp_act_id),
        .s2_train_sent(b0_s2_train_sent),
        .s2_seat_trained(b0_s2_seat_trained),
        .s2_seat_pc(b0_s2_seat_pc),
        .s3_amo_v(b0_s3_amo_v),
        .lock_valid(b0_lock_valid),
        .lock_snp(b0_lock_snp),
        .lock_line(b0_lock_line),
        .ev_amor(b0_ev_amor),
        .tok_valid(b0_tok_valid),
        .tok_id(b0_tok_id),
        .tok_line(b0_tok_line),
        .mt_valid(b0_mt_valid),
        .mt_set(b0_mt_set),
        .mt_way(b0_mt_way),
        .mt_op(b0_mt_op),
        .m_resp_valid(b0_m_resp_valid),
        .n_resp_valid(b0_n_resp_valid),
        .mshr_freed(b0_mshr_freed),
        .vc_freed(b0_vc_freed),
        .nc_freed(b0_nc_freed),
        .any_gnt(b0_any_gnt),
        .g_set(b0_g_set),
        .gnt_refill(b0_gnt_refill),
        .gnt_maint(b0_gnt_maint),
        .r_valid(b0_r_valid),
        .r_kind(b0_r_kind),
        .r_seat(b0_r_seat),
        .r_req(b0_r_req),
        .r_fast(b0_r_fast),
        .r_mid(b0_r_mid),
        .r_line(b0_r_line),
        .r_mway(b0_r_mway),
        .r_mop(b0_r_mop),
        .r_sinv(b0_r_sinv),
        .r_sid(b0_r_sid),
        .r_deps(b0_r_deps),
        .tracker_empty(b0_tracker_empty),
        .miss_ok(b0_miss_ok),
        .vec_write_slot(b0_vec_write_slot),
        .vec_write_valid(b0_vec_write_valid)
    );
    l1d_bank_s0 #(`L1D_PARAMS_PASS, .BANK_ID(1)) u_s0_1 (
        .clk(clk), .rst(rst),
        .arrival_request_0(bank_arrival_request_0[1]), .arrival_packet_0(port_request_packet_0), .arrival_pc_0(port_request_pc_0), .arrival_allocated_0(bank_arrival_allocated_0[1]),
        .arrival_request_1(bank_arrival_request_1[1]), .arrival_packet_1(port_request_packet_1), .arrival_pc_1(port_request_pc_1), .arrival_allocated_1(bank_arrival_allocated_1[1]),
        .arrival_request_2(bank_arrival_request_2[1]), .arrival_packet_2(port_request_packet_2), .arrival_pc_2(port_request_pc_2), .arrival_allocated_2(bank_arrival_allocated_2[1]),
        .arrival_request_3(bank_arrival_request_3[1]), .arrival_packet_3(port_request_packet_3), .arrival_pc_3(port_request_pc_3), .arrival_allocated_3(bank_arrival_allocated_3[1]),
        .drain_request(bank_drain_request[1]), .drain_packet(bank_drain_packet[1*REQ_W +: REQ_W]), .drain_pc(bank_drain_pc[1*PC_W +: PC_W]),
        .drain_allocated(bank_drain_allocated[1]), .tracker_free_count_q(bank_tracker_free_count[1*TRK_CNT_W +: TRK_CNT_W]),
        .vec_slot_available(bank_vec_slot_available[1]),
        .lsu_response_fifo_free_0(b1_lsu_response_fifo_free_0),
        .lsu_response_fifo_free_1(b1_lsu_response_fifo_free_1),
        .lsu_response_fifo_free_2(b1_lsu_response_fifo_free_2),
        .lsu_response_fifo_free_3(b1_lsu_response_fifo_free_3),
        .lsu_response_fifo_free_next_0(b1_lsu_response_fifo_free_next_0),
        .lsu_response_fifo_free_next_1(b1_lsu_response_fifo_free_next_1),
        .lsu_response_fifo_free_next_2(b1_lsu_response_fifo_free_next_2),
        .lsu_response_fifo_free_next_3(b1_lsu_response_fifo_free_next_3),
        .snoop_request_valid(b1_snoop_request_valid),
        .snoop_request_line_addr(b1_snoop_request_line_addr),
        .snoop_request_invalidate(b1_snoop_request_invalidate),
        .snoop_request_ready(b1_snoop_request_ready),
        .snoop_response_fifo_free(b1_snoop_response_fifo_free),
        .prefetch_request_valid(b1_prefetch_request_valid),
        .prefetch_request_line_addr(b1_prefetch_request_line_addr),
        .prefetch_request_unique(b1_prefetch_request_unique),
        .prefetch_request_ready(b1_prefetch_request_ready),
        .refill_handover_busy(b1_refill_handover_busy),
        .s1_valid(b1_s1_valid),
        .s1_kind(b1_s1_kind),
        .s1_req(b1_s1_req),
        .s2_valid(b1_s2_valid),
        .s2_kind(b1_s2_kind),
        .s2_req(b1_s2_req),
        .s2_hit(b1_s2_hit),
        .s2_sid(b1_s2_sid),
        .s2_line(b1_s2_line),
        .s2_seat(b1_s2_seat),
        .s2_kill(b1_s2_kill),
        .s2_answered(b1_s2_answered),
        .act_valid(b1_act_valid),
        .act_code(b1_act_code),
        .act_id(b1_act_id),
        .snp_act_valid(b1_snp_act_valid),
        .snp_act_code(b1_snp_act_code),
        .snp_act_id(b1_snp_act_id),
        .s2_train_sent(b1_s2_train_sent),
        .s2_seat_trained(b1_s2_seat_trained),
        .s2_seat_pc(b1_s2_seat_pc),
        .s3_amo_v(b1_s3_amo_v),
        .lock_valid(b1_lock_valid),
        .lock_snp(b1_lock_snp),
        .lock_line(b1_lock_line),
        .ev_amor(b1_ev_amor),
        .tok_valid(b1_tok_valid),
        .tok_id(b1_tok_id),
        .tok_line(b1_tok_line),
        .mt_valid(b1_mt_valid),
        .mt_set(b1_mt_set),
        .mt_way(b1_mt_way),
        .mt_op(b1_mt_op),
        .m_resp_valid(b1_m_resp_valid),
        .n_resp_valid(b1_n_resp_valid),
        .mshr_freed(b1_mshr_freed),
        .vc_freed(b1_vc_freed),
        .nc_freed(b1_nc_freed),
        .any_gnt(b1_any_gnt),
        .g_set(b1_g_set),
        .gnt_refill(b1_gnt_refill),
        .gnt_maint(b1_gnt_maint),
        .r_valid(b1_r_valid),
        .r_kind(b1_r_kind),
        .r_seat(b1_r_seat),
        .r_req(b1_r_req),
        .r_fast(b1_r_fast),
        .r_mid(b1_r_mid),
        .r_line(b1_r_line),
        .r_mway(b1_r_mway),
        .r_mop(b1_r_mop),
        .r_sinv(b1_r_sinv),
        .r_sid(b1_r_sid),
        .r_deps(b1_r_deps),
        .tracker_empty(b1_tracker_empty),
        .miss_ok(b1_miss_ok),
        .vec_write_slot(b1_vec_write_slot),
        .vec_write_valid(b1_vec_write_valid)
    );
    l1d_bank_s0 #(`L1D_PARAMS_PASS, .BANK_ID(2)) u_s0_2 (
        .clk(clk), .rst(rst),
        .arrival_request_0(bank_arrival_request_0[2]), .arrival_packet_0(port_request_packet_0), .arrival_pc_0(port_request_pc_0), .arrival_allocated_0(bank_arrival_allocated_0[2]),
        .arrival_request_1(bank_arrival_request_1[2]), .arrival_packet_1(port_request_packet_1), .arrival_pc_1(port_request_pc_1), .arrival_allocated_1(bank_arrival_allocated_1[2]),
        .arrival_request_2(bank_arrival_request_2[2]), .arrival_packet_2(port_request_packet_2), .arrival_pc_2(port_request_pc_2), .arrival_allocated_2(bank_arrival_allocated_2[2]),
        .arrival_request_3(bank_arrival_request_3[2]), .arrival_packet_3(port_request_packet_3), .arrival_pc_3(port_request_pc_3), .arrival_allocated_3(bank_arrival_allocated_3[2]),
        .drain_request(bank_drain_request[2]), .drain_packet(bank_drain_packet[2*REQ_W +: REQ_W]), .drain_pc(bank_drain_pc[2*PC_W +: PC_W]),
        .drain_allocated(bank_drain_allocated[2]), .tracker_free_count_q(bank_tracker_free_count[2*TRK_CNT_W +: TRK_CNT_W]),
        .vec_slot_available(bank_vec_slot_available[2]),
        .lsu_response_fifo_free_0(b2_lsu_response_fifo_free_0),
        .lsu_response_fifo_free_1(b2_lsu_response_fifo_free_1),
        .lsu_response_fifo_free_2(b2_lsu_response_fifo_free_2),
        .lsu_response_fifo_free_3(b2_lsu_response_fifo_free_3),
        .lsu_response_fifo_free_next_0(b2_lsu_response_fifo_free_next_0),
        .lsu_response_fifo_free_next_1(b2_lsu_response_fifo_free_next_1),
        .lsu_response_fifo_free_next_2(b2_lsu_response_fifo_free_next_2),
        .lsu_response_fifo_free_next_3(b2_lsu_response_fifo_free_next_3),
        .snoop_request_valid(b2_snoop_request_valid),
        .snoop_request_line_addr(b2_snoop_request_line_addr),
        .snoop_request_invalidate(b2_snoop_request_invalidate),
        .snoop_request_ready(b2_snoop_request_ready),
        .snoop_response_fifo_free(b2_snoop_response_fifo_free),
        .prefetch_request_valid(b2_prefetch_request_valid),
        .prefetch_request_line_addr(b2_prefetch_request_line_addr),
        .prefetch_request_unique(b2_prefetch_request_unique),
        .prefetch_request_ready(b2_prefetch_request_ready),
        .refill_handover_busy(b2_refill_handover_busy),
        .s1_valid(b2_s1_valid),
        .s1_kind(b2_s1_kind),
        .s1_req(b2_s1_req),
        .s2_valid(b2_s2_valid),
        .s2_kind(b2_s2_kind),
        .s2_req(b2_s2_req),
        .s2_hit(b2_s2_hit),
        .s2_sid(b2_s2_sid),
        .s2_line(b2_s2_line),
        .s2_seat(b2_s2_seat),
        .s2_kill(b2_s2_kill),
        .s2_answered(b2_s2_answered),
        .act_valid(b2_act_valid),
        .act_code(b2_act_code),
        .act_id(b2_act_id),
        .snp_act_valid(b2_snp_act_valid),
        .snp_act_code(b2_snp_act_code),
        .snp_act_id(b2_snp_act_id),
        .s2_train_sent(b2_s2_train_sent),
        .s2_seat_trained(b2_s2_seat_trained),
        .s2_seat_pc(b2_s2_seat_pc),
        .s3_amo_v(b2_s3_amo_v),
        .lock_valid(b2_lock_valid),
        .lock_snp(b2_lock_snp),
        .lock_line(b2_lock_line),
        .ev_amor(b2_ev_amor),
        .tok_valid(b2_tok_valid),
        .tok_id(b2_tok_id),
        .tok_line(b2_tok_line),
        .mt_valid(b2_mt_valid),
        .mt_set(b2_mt_set),
        .mt_way(b2_mt_way),
        .mt_op(b2_mt_op),
        .m_resp_valid(b2_m_resp_valid),
        .n_resp_valid(b2_n_resp_valid),
        .mshr_freed(b2_mshr_freed),
        .vc_freed(b2_vc_freed),
        .nc_freed(b2_nc_freed),
        .any_gnt(b2_any_gnt),
        .g_set(b2_g_set),
        .gnt_refill(b2_gnt_refill),
        .gnt_maint(b2_gnt_maint),
        .r_valid(b2_r_valid),
        .r_kind(b2_r_kind),
        .r_seat(b2_r_seat),
        .r_req(b2_r_req),
        .r_fast(b2_r_fast),
        .r_mid(b2_r_mid),
        .r_line(b2_r_line),
        .r_mway(b2_r_mway),
        .r_mop(b2_r_mop),
        .r_sinv(b2_r_sinv),
        .r_sid(b2_r_sid),
        .r_deps(b2_r_deps),
        .tracker_empty(b2_tracker_empty),
        .miss_ok(b2_miss_ok),
        .vec_write_slot(b2_vec_write_slot),
        .vec_write_valid(b2_vec_write_valid)
    );
    l1d_bank_s0 #(`L1D_PARAMS_PASS, .BANK_ID(3)) u_s0_3 (
        .clk(clk), .rst(rst),
        .arrival_request_0(bank_arrival_request_0[3]), .arrival_packet_0(port_request_packet_0), .arrival_pc_0(port_request_pc_0), .arrival_allocated_0(bank_arrival_allocated_0[3]),
        .arrival_request_1(bank_arrival_request_1[3]), .arrival_packet_1(port_request_packet_1), .arrival_pc_1(port_request_pc_1), .arrival_allocated_1(bank_arrival_allocated_1[3]),
        .arrival_request_2(bank_arrival_request_2[3]), .arrival_packet_2(port_request_packet_2), .arrival_pc_2(port_request_pc_2), .arrival_allocated_2(bank_arrival_allocated_2[3]),
        .arrival_request_3(bank_arrival_request_3[3]), .arrival_packet_3(port_request_packet_3), .arrival_pc_3(port_request_pc_3), .arrival_allocated_3(bank_arrival_allocated_3[3]),
        .drain_request(bank_drain_request[3]), .drain_packet(bank_drain_packet[3*REQ_W +: REQ_W]), .drain_pc(bank_drain_pc[3*PC_W +: PC_W]),
        .drain_allocated(bank_drain_allocated[3]), .tracker_free_count_q(bank_tracker_free_count[3*TRK_CNT_W +: TRK_CNT_W]),
        .vec_slot_available(bank_vec_slot_available[3]),
        .lsu_response_fifo_free_0(b3_lsu_response_fifo_free_0),
        .lsu_response_fifo_free_1(b3_lsu_response_fifo_free_1),
        .lsu_response_fifo_free_2(b3_lsu_response_fifo_free_2),
        .lsu_response_fifo_free_3(b3_lsu_response_fifo_free_3),
        .lsu_response_fifo_free_next_0(b3_lsu_response_fifo_free_next_0),
        .lsu_response_fifo_free_next_1(b3_lsu_response_fifo_free_next_1),
        .lsu_response_fifo_free_next_2(b3_lsu_response_fifo_free_next_2),
        .lsu_response_fifo_free_next_3(b3_lsu_response_fifo_free_next_3),
        .snoop_request_valid(b3_snoop_request_valid),
        .snoop_request_line_addr(b3_snoop_request_line_addr),
        .snoop_request_invalidate(b3_snoop_request_invalidate),
        .snoop_request_ready(b3_snoop_request_ready),
        .snoop_response_fifo_free(b3_snoop_response_fifo_free),
        .prefetch_request_valid(b3_prefetch_request_valid),
        .prefetch_request_line_addr(b3_prefetch_request_line_addr),
        .prefetch_request_unique(b3_prefetch_request_unique),
        .prefetch_request_ready(b3_prefetch_request_ready),
        .refill_handover_busy(b3_refill_handover_busy),
        .s1_valid(b3_s1_valid),
        .s1_kind(b3_s1_kind),
        .s1_req(b3_s1_req),
        .s2_valid(b3_s2_valid),
        .s2_kind(b3_s2_kind),
        .s2_req(b3_s2_req),
        .s2_hit(b3_s2_hit),
        .s2_sid(b3_s2_sid),
        .s2_line(b3_s2_line),
        .s2_seat(b3_s2_seat),
        .s2_kill(b3_s2_kill),
        .s2_answered(b3_s2_answered),
        .act_valid(b3_act_valid),
        .act_code(b3_act_code),
        .act_id(b3_act_id),
        .snp_act_valid(b3_snp_act_valid),
        .snp_act_code(b3_snp_act_code),
        .snp_act_id(b3_snp_act_id),
        .s2_train_sent(b3_s2_train_sent),
        .s2_seat_trained(b3_s2_seat_trained),
        .s2_seat_pc(b3_s2_seat_pc),
        .s3_amo_v(b3_s3_amo_v),
        .lock_valid(b3_lock_valid),
        .lock_snp(b3_lock_snp),
        .lock_line(b3_lock_line),
        .ev_amor(b3_ev_amor),
        .tok_valid(b3_tok_valid),
        .tok_id(b3_tok_id),
        .tok_line(b3_tok_line),
        .mt_valid(b3_mt_valid),
        .mt_set(b3_mt_set),
        .mt_way(b3_mt_way),
        .mt_op(b3_mt_op),
        .m_resp_valid(b3_m_resp_valid),
        .n_resp_valid(b3_n_resp_valid),
        .mshr_freed(b3_mshr_freed),
        .vc_freed(b3_vc_freed),
        .nc_freed(b3_nc_freed),
        .any_gnt(b3_any_gnt),
        .g_set(b3_g_set),
        .gnt_refill(b3_gnt_refill),
        .gnt_maint(b3_gnt_maint),
        .r_valid(b3_r_valid),
        .r_kind(b3_r_kind),
        .r_seat(b3_r_seat),
        .r_req(b3_r_req),
        .r_fast(b3_r_fast),
        .r_mid(b3_r_mid),
        .r_line(b3_r_line),
        .r_mway(b3_r_mway),
        .r_mop(b3_r_mop),
        .r_sinv(b3_r_sinv),
        .r_sid(b3_r_sid),
        .r_deps(b3_r_deps),
        .tracker_empty(b3_tracker_empty),
        .miss_ok(b3_miss_ok),
        .vec_write_slot(b3_vec_write_slot),
        .vec_write_valid(b3_vec_write_valid)
    );
endmodule
