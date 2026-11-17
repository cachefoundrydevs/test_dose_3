`default_nettype none
`include "l1d_defines.vh"
module dcache_v5_v14_compat #(
    parameter D_CACHE_SIZE                    = 64 * 1024,
    parameter D_NUM_OF_WAYS                   = 4,
    parameter D_CACHE_LINE                    = 64,
    parameter D_SET_INDEX                     = D_CACHE_SIZE / (D_NUM_OF_WAYS * D_CACHE_LINE),
    parameter D_ADDR_WIDTH                    = 64,
    parameter D_CACHE_LINE_BITS               = D_CACHE_LINE * 8,
    parameter D_WRITE_DATA_WIDTH              = 64,
    parameter D_VICTIM_DEPTH                  = 8,
    parameter D_VICTIM_BITS                   = $clog2(D_VICTIM_DEPTH),
    parameter D_VC_DEPTH_FIFO                 = 8,
    parameter D_VC_DEPTH_FIFO_BITS            = $clog2(D_VC_DEPTH_FIFO),
    parameter D_MSHR_DEPTH                    = 16,
    parameter D_MSHR_BITS                     = $clog2(D_MSHR_DEPTH),
    parameter D_NON_CACHABLE_DEPTH            = 8,
    parameter D_NON_CACHABLE_BITS             = $clog2(D_NON_CACHABLE_DEPTH),
    parameter D_MSHR_DEPTH_FIFO               = 16,
    parameter D_AXI_BYTE                      = 16,
    parameter D_MM_BUS_SIZE_BITS              = D_AXI_BYTE * 8,
    parameter D_SHEDULAR_DEPTH                = 16,
    parameter D_SHEDULAR_BITS                 = $clog2(D_SHEDULAR_DEPTH),
    parameter D_NUM_OF_MASTER_ID              = 4,
    parameter D_NUM_OF_IDX                    = 8,
    parameter D_MASTER_ID_BITS                = $clog2(D_NUM_OF_MASTER_ID),
    parameter D_IDX_BITS                      = $clog2(D_NUM_OF_IDX),
    parameter D_MEM_SIZE_TYPE                 = 4,
    parameter D_MEM_SIZE_BITS                 = $clog2(D_MEM_SIZE_TYPE),
    parameter D_MSHR_TO_MM_DW_FIFO_DEPTH      = 16,
    parameter D_DATA_MSHR_TO_MM_DW_FIFO_DEPTH = 16,
    parameter D_OFFSET_BITS                   = $clog2(D_CACHE_LINE),
    parameter D_SET_BITS                      = $clog2(D_SET_INDEX),
    parameter D_WAY_BITS                      = $clog2(D_NUM_OF_WAYS),
    parameter D_TAG_BITS                      = D_ADDR_WIDTH - (D_SET_BITS + D_OFFSET_BITS),
    parameter D_CLEAN_WB_DEPTH                = 16,
    parameter D_CLEAN_WB_BITS                 = $clog2(D_CLEAN_WB_DEPTH),
    parameter D_NO_OF_TRANSTION               = D_CACHE_LINE_BITS / D_MM_BUS_SIZE_BITS,
    parameter D_TRANSTION_BITS                = $clog2(D_NO_OF_TRANSTION),
    parameter D_STROBE_BITS                   = D_AXI_BYTE,
    parameter D_FULL_STROBE_BITS              = D_NO_OF_TRANSTION * D_STROBE_BITS,
    parameter D_FINAL_WR_POINTER_AXI          = (D_MSHR_BITS + 1) + D_WAY_BITS + 1 + 1,
    parameter D_ARLEN                         = D_NO_OF_TRANSTION - 1,
    parameter D_ARSIZE                        = D_ARLEN,
    parameter D_ARBURST                       = 2,
    parameter D_AXI_READ_ADDR_BITS            = D_MSHR_BITS + 2,
    parameter D_AXI_WRITE_ADDR_BITS           = D_FINAL_WR_POINTER_AXI
) (
    input  wire                              clk,
    input  wire                              rst,
    input  wire                              flush,
    input  wire                              clean,
    input  wire                              valid_cpu,
    input  wire                              cachable,
    input  wire                              rd_wr_cpu,
    input  wire [D_MEM_SIZE_BITS-1:0]        mem_size_cpu,
    input  wire [D_MASTER_ID_BITS-1:0]       master_id_from_cpu,
    input  wire [D_IDX_BITS-1:0]             idx_from_cpu,
    input  wire [D_ADDR_WIDTH-1:0]           PC_from_cpu,
    input  wire [D_WRITE_DATA_WIDTH-1:0]     data_from_cpu,
    output wire                              axi_arvalid,
    output wire [D_ADDR_WIDTH-1:0]           axi_araddr,
    output wire [D_AXI_READ_ADDR_BITS-1:0]   axi_arid,
    output wire [D_ARLEN-1:0]                axi_arlen,
    output wire [2:0]                        axi_arsize,
    output wire [1:0]                        axi_arburst,
    output wire                              axi_arlock,
    output wire [3:0]                        axi_arcache,
    output wire [2:0]                        axi_arprot,
    output wire [3:0]                        axi_arqos,
    input  wire                              axi_arready,
    input  wire                              axi_rvalid,
    input  wire [D_MM_BUS_SIZE_BITS-1:0]     axi_rdata,
    input  wire [D_AXI_READ_ADDR_BITS-1:0]   axi_rid,
    input  wire [1:0]                        axi_rres,
    input  wire                              axi_rlast,
    output wire                              axi_rready,
    output wire                              axi_wr_arvalid,
    output wire [D_ADDR_WIDTH-1:0]           axi_wr_araddr,
    output wire [D_AXI_WRITE_ADDR_BITS-1:0]  axi_wr_arid,
    output wire [D_ARLEN-1:0]                axi_wr_arlen,
    output wire [2:0]                        axi_wr_arsize,
    output wire [1:0]                        axi_wr_arburst,
    output wire                              axi_wr_arlock,
    output wire [3:0]                        axi_wr_arcache,
    output wire [2:0]                        axi_wr_arprot,
    output wire [3:0]                        axi_wr_arqos,
    input  wire                              axi_wr_arready,
    output wire                              axi_wr_d_valid,
    output wire                              axi_wr_d_last,
    output wire [D_MM_BUS_SIZE_BITS-1:0]     axi_wr_d_data,
    output wire [D_AXI_WRITE_ADDR_BITS-1:0]  axi_wr_d_id,
    output wire [D_STROBE_BITS-1:0]          axi_wr_strobe,
    input  wire                              axi_wr_d_ready,
    input  wire                              axi_wr_res_valid,
    input  wire [1:0]                        axi_wr_res_error_info,
    input  wire [D_AXI_WRITE_ADDR_BITS-1:0]  axi_wr_res_addr_id,
    output wire                              axi_wr_res_ready,
    output wire                              stop,
    input  wire                              read_enable_from_lsu,
    output wire                              clean_complete,
    output wire                              flush_complete,
    output wire                              mem_err,
    output wire                              final_r_cache_hit_from_sheduler,
    output wire                              final_r_hit_rd_wr_from_sheduler,
    output wire [D_MEM_SIZE_BITS-1:0]        final_r_mem_size_from_sheduler,
    output wire [D_MASTER_ID_BITS-1:0]       final_r_master_id_from_sheduler,
    output wire [D_IDX_BITS-1:0]             final_r_idx_from_sheduler,
    output wire [D_WRITE_DATA_WIDTH-1:0]     final_r_data_from_sheduler
);
    localparam NUM_BANKS    = `L1D_NUM_BANKS;
    localparam SETS         = D_SET_INDEX / NUM_BANKS;
    localparam L2_ADDR_W    = 56;
    localparam L2_MID_W     = 2;
    localparam L2_TID_W     = 6;
    localparam L2_PKT_W     = 1 + L2_ADDR_W + 1 + L2_MID_W + L2_TID_W + 1 + D_CACHE_LINE + D_CACHE_LINE*8;
    localparam BRIDGE_SLOTS = 16;
    localparam READ_ID_PAD  = D_AXI_READ_ADDR_BITS  - L2_TID_W;
    localparam WRITE_ID_PAD = D_AXI_WRITE_ADDR_BITS - L2_TID_W;
`ifndef SYNTHESIS
    initial if (READ_ID_PAD < 0 || WRITE_ID_PAD < 0) begin
        $display("dcache_v5_v14_compat ERROR: AXI id (%0d bits) wider than D_AXI_READ_ADDR_BITS (%0d) or D_AXI_WRITE_ADDR_BITS (%0d); use D_MSHR_DEPTH >= 16",
                 L2_TID_W, D_AXI_READ_ADDR_BITS, D_AXI_WRITE_ADDR_BITS);
        $finish;
    end
`endif
    wire                    l2_request_valid;
    wire                    l2_request_ready;
    wire [L2_PKT_W-1:0]     l2_request_packet;
    wire                    l2_response_valid;
    wire                    l2_response_ready;
    wire [L2_PKT_W-1:0]     l2_response_packet;
    wire                    l2_response_error;
    wire [L2_TID_W-1:0]     axi_write_id;
    wire [L2_TID_W-1:0]     axi_read_id;
    wire [7:0]              axi_write_len;
    wire [7:0]              axi_read_len;
    l1d_top_single_port #(
        .ADDR_W           (D_ADDR_WIDTH),
        .LINE_BYTES       (D_CACHE_LINE),
        .NUM_BANKS        (NUM_BANKS),
        .SETS             (SETS),
        .WAYS             (D_NUM_OF_WAYS),
        .WRITE_DATA_WIDTH (D_WRITE_DATA_WIDTH),
        .MEM_SIZE_BITS    (D_MEM_SIZE_BITS),
        .MASTER_ID_BITS   (D_MASTER_ID_BITS),
        .IDX_BITS         (D_IDX_BITS),
        .AXI_DATA_W       (D_MM_BUS_SIZE_BITS),
        .AXI_ADDR_W       (D_ADDR_WIDTH),
        .L2_ADDR_W        (L2_ADDR_W),
        .L2_MID_W         (L2_MID_W),
        .L2_TID_W         (L2_TID_W)
    ) u_cache (
        .clk                       (clk),
        .rst                       (rst),
        .flush_request             (flush),
        .clean_request             (clean),
        .flush_complete            (flush_complete),
        .clean_complete            (clean_complete),
        .cpu_request_valid         (valid_cpu),
        .cpu_request_cacheable     (cachable),
        .cpu_request_is_write      (rd_wr_cpu),
        .cpu_request_size          (mem_size_cpu),
        .cpu_request_master_id     (master_id_from_cpu),
        .cpu_request_rob_index     (idx_from_cpu),
        .cpu_request_addr          (PC_from_cpu),
        .cpu_request_store_data    (data_from_cpu),
        .cpu_request_is_atomic     (1'b0),
        .cpu_request_atomic_is_amo (1'b0),
        .cpu_request_amo_op        (4'd0),
        .input_queue_full          (stop),
        .lsu_response_read_enable  (read_enable_from_lsu),
        .lsu_response_valid        (final_r_cache_hit_from_sheduler),
        .lsu_response_is_write_ack (final_r_hit_rd_wr_from_sheduler),
        .lsu_response_size         (final_r_mem_size_from_sheduler),
        .lsu_response_master_id    (final_r_master_id_from_sheduler),
        .lsu_response_rob_index    (final_r_idx_from_sheduler),
        .lsu_response_data         (final_r_data_from_sheduler),
        .snoop_request_valid       (1'b0),
        .snoop_request_ready       (),
        .snoop_request_addr        ({D_ADDR_WIDTH{1'b0}}),
        .snoop_request_invalidate  (1'b0),
        .snoop_response_valid      (),
        .snoop_response_ready      (1'b1),
        .snoop_response_addr       (),
        .snoop_response_present    (),
        .snoop_response_dirty      (),
        .snoop_response_data       (),
        .l2_error_sticky           (mem_err),
        .amo_lock_timeout_sticky   (),
        .ecc_correctable_pulse     (),
        .ecc_uncorrectable_sticky  (),
        .l2_req_valid              (l2_request_valid),
        .l2_req_ready              (l2_request_ready),
        .l2_req_pkt                (l2_request_packet),
        .l2_resp_valid             (l2_response_valid),
        .l2_resp_ready             (l2_response_ready),
        .l2_resp_pkt               (l2_response_packet),
        .l2_resp_err               (l2_response_error)
    );
    l2_pkt_axi_bridge #(
        .ADDR_W     (L2_ADDR_W),
        .MID_W      (L2_MID_W),
        .TID_W      (L2_TID_W),
        .DATA_W     (D_CACHE_LINE*8),
        .AXI_ADDR_W (D_ADDR_WIDTH),
        .AXI_DATA_W (D_MM_BUS_SIZE_BITS),
        .ID_HAS_MID (0),
        .AXI_ID_W   (L2_TID_W),
        .N_OUT      (BRIDGE_SLOTS)
    ) u_axi_bridge (
        .clk           (clk),
        .rst           (rst),
        .req_valid     (l2_request_valid),
        .req_pkt       (l2_request_packet),
        .req_ready     (l2_request_ready),
        .resp_valid    (l2_response_valid),
        .resp_pkt      (l2_response_packet),
        .resp_ready    (l2_response_ready),
        .resp_err      (l2_response_error),
        .m_axi_arid    (axi_read_id),
        .m_axi_araddr  (axi_araddr),
        .m_axi_arlen   (axi_read_len),
        .m_axi_arsize  (axi_arsize),
        .m_axi_arburst (axi_arburst),
        .m_axi_arcache (axi_arcache),
        .m_axi_arprot  (axi_arprot),
        .m_axi_aruser  (),
        .m_axi_arvalid (axi_arvalid),
        .m_axi_arready (axi_arready),
        .m_axi_rid     (axi_rid[L2_TID_W-1:0]),
        .m_axi_rdata   (axi_rdata),
        .m_axi_rresp   (axi_rres),
        .m_axi_rlast   (axi_rlast),
        .m_axi_ruser   (1'b0),
        .m_axi_rvalid  (axi_rvalid),
        .m_axi_rready  (axi_rready),
        .m_axi_awid    (axi_write_id),
        .m_axi_awaddr  (axi_wr_araddr),
        .m_axi_awlen   (axi_write_len),
        .m_axi_awsize  (axi_wr_arsize),
        .m_axi_awburst (axi_wr_arburst),
        .m_axi_awcache (axi_wr_arcache),
        .m_axi_awprot  (axi_wr_arprot),
        .m_axi_awvalid (axi_wr_arvalid),
        .m_axi_awready (axi_wr_arready),
        .m_axi_wdata   (axi_wr_d_data),
        .m_axi_wstrb   (axi_wr_strobe),
        .m_axi_wlast   (axi_wr_d_last),
        .m_axi_wvalid  (axi_wr_d_valid),
        .m_axi_wready  (axi_wr_d_ready),
        .m_axi_bid     (axi_wr_res_addr_id[L2_TID_W-1:0]),
        .m_axi_bresp   (axi_wr_res_error_info),
        .m_axi_bvalid  (axi_wr_res_valid),
        .m_axi_bready  (axi_wr_res_ready),
        .rd_pend_valid (),
        .rd_pend_addr  ()
    );
    generate
        if (READ_ID_PAD > 0) begin : g_read_id_pad
            assign axi_arid = {{READ_ID_PAD{1'b0}}, axi_read_id};
        end else begin : g_read_id_exact
            assign axi_arid = axi_read_id[D_AXI_READ_ADDR_BITS-1:0];
        end
        if (WRITE_ID_PAD > 0) begin : g_write_id_pad
            assign axi_wr_arid = {{WRITE_ID_PAD{1'b0}}, axi_write_id};
            assign axi_wr_d_id = {{WRITE_ID_PAD{1'b0}}, axi_write_id};
        end else begin : g_write_id_exact
            assign axi_wr_arid = axi_write_id[D_AXI_WRITE_ADDR_BITS-1:0];
            assign axi_wr_d_id = axi_write_id[D_AXI_WRITE_ADDR_BITS-1:0];
        end
    endgenerate
    assign axi_arlen      = axi_read_len[D_ARLEN-1:0];
    assign axi_wr_arlen   = axi_write_len[D_ARLEN-1:0];
    assign axi_arlock     = 1'b0;
    assign axi_arqos      = 4'd0;
    assign axi_wr_arlock  = 1'b0;
    assign axi_wr_arqos   = 4'd0;
endmodule
`default_nettype wire
