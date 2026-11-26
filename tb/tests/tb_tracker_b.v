`timescale 1ns/1ps
`include "l1d_defines.vh"
module tb_tracker_b;
    localparam SETS = `L1D_SETS, WAYS = `L1D_WAYS, LSU_BUF = `L1D_LSU_BUF_DEPTH,
               MSHR = `L1D_MSHR_ENTRIES, NCE = `L1D_MSHR_NC_ENTRIES, VCS = `L1D_VC_SLOTS,
               RFIFO = `L1D_RESP_FIFO_DEPTH;
`ifdef SEED
    localparam SEED0 = `SEED;
`else
    localparam SEED0 = 1;
`endif
`ifdef NOPS
    localparam N_RANDOM = `NOPS;
`else
    localparam N_RANDOM = 3000;
`endif
`include "tb_ports_common.vh"
    integer cov_skid_push = 0;
    integer cov_trk_full  = 0;
    integer cov_multi_alloc = 0;
    integer cv_b, cv_n;
    always @(posedge clk) if (!rst) begin
        if (|dut.u_dcache.u_front.skid_push) cov_skid_push = cov_skid_push + 1;
        for (cv_b = 0; cv_b < NB; cv_b = cv_b + 1) begin
            cv_n = dut.u_dcache.u_front.alloc_pred[cv_b*4 + 0] + dut.u_dcache.u_front.alloc_pred[cv_b*4 + 1] +
                   dut.u_dcache.u_front.alloc_pred[cv_b*4 + 2] + dut.u_dcache.u_front.alloc_pred[cv_b*4 + 3];
            if (cv_n >= 2) cov_multi_alloc = cov_multi_alloc + 1;
        end
        if (dut.u_dcache.g_bank[2].u_bank.u_s0.u_tracker.free_count_q == 0) cov_trk_full = cov_trk_full + 1;
    end
    integer e0, e1, e2, e3, lat0, lat1, lat2, n, pt, push_before;
    reg [ADDR_W-1:0] ra;
    initial begin
        init_tb;
        for (n = 0; n < 3; n = n + 1) begin
            send(0, K_LD, 1'b0, 2'd3, line_addr(0, n, 1), 0, OP_ADD, e0); tick; wait_entry(e0);
        end
        repeat (10) tick;
        $display("=== A: three ports, three sets of bank 1, one cycle ===");
        push_before = cov_skid_push;
        send(0, K_LD, 1'b0, 2'd3, line_addr(0, 0, 1) + 8, 0, OP_ADD, e0);
        send(1, K_LD, 1'b0, 2'd3, line_addr(0, 1, 1) + 8, 0, OP_ADD, e1);
        send(2, K_ST, 1'b0, 2'd3, line_addr(0, 2, 1) + 8, 64'h1111_2222_3333_4444, OP_ADD, e2);
        tick; wait_entry(e0); wait_entry(e1); wait_entry(e2);
        lat0 = got_t[e0] - out_t[e0]; lat1 = got_t[e1] - out_t[e1]; lat2 = got_t[e2] - out_t[e2];
        $display("  latencies: P0 %0d, P1 %0d, P2 (store) %0d", lat0, lat1, lat2);
        check(lat0 == 4 && lat1 == 5 && lat2 == 6, "A: three requests to one bank in one cycle answer at 4, 5, 6 (port order)");
        check(cov_skid_push == push_before, "A: all three got a tracker entry in the arrival cycle (no skid push)");
        repeat (10) tick;
`ifdef B_WARM
        send(0, K_LD, 1'b0, 2'd3, line_addr(1, 0, 1), 0, OP_ADD, e0); tick; wait_entry(e0);
        send(0, K_LD, 1'b0, 2'd3, line_addr(2, 0, 1), 0, OP_ADD, e0); tick; wait_entry(e0);
        repeat (10) tick;
`endif
        $display("=== B: three ports, ONE set of bank 1 (P0 load line X, P1 load line Y, P2 store line Z) ===");
        send(0, K_LD, 1'b0, 2'd3, line_addr(0, 0, 1) + 16, 0, OP_ADD, e0);
        send(1, K_LD, 1'b0, 2'd3, line_addr(1, 0, 1) + 16, 0, OP_ADD, e1);
        send(2, K_ST, 1'b0, 2'd3, line_addr(2, 0, 1) + 16, 64'h5555_6666_7777_8888, OP_ADD, e2);
        tick; wait_entry(e0); wait_entry(e1); wait_entry(e2);
        $display("  latencies: P0 %0d, P1 %0d, P2 (store) %0d", got_t[e0] - out_t[e0], got_t[e1] - out_t[e1], got_t[e2] - out_t[e2]);
        $display("  B detail: out_t e0 %0d e1 %0d e2 %0d | got_t e0 %0d e1 %0d e2 %0d | store answers %s", out_t[e0], out_t[e1], out_t[e2], got_t[e0], got_t[e1], got_t[e2],
                 (got_t[e2] > got_t[e0] && got_t[e2] > got_t[e1]) ? "AFTER both loads" : "NOT after both loads");
        check(got_t[e2] > got_t[e0] && got_t[e2] > got_t[e1], "B: the store answers after both older loads of its set (deps at allocation)");
        repeat (10) tick;
        finish_tb("tb_tracker_b");
    end
endmodule
