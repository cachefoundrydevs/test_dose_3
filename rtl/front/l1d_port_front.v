// ============================================================================
// l1d_port_front.v  -  front end of the four request ports (cache_v6): bank decode, one skid per port, the order ring per bank
// ============================================================================
// IN SIMPLE WORDS  (cache_v6, PLAN_v6.md section 4.5)
//   Every port may send one request per cycle to any of the four banks. Each bank has a request tracker with 8 entries
//   (l1d_req_tracker) that takes up to 4 arrivals in one cycle, so in the common case a request goes straight into its
//   bank's tracker (and, if the bank is free, into the pipeline) in its arrival cycle. Only when a bank has no room (or
//   something older already waits for it) does the request step aside into its port's SKID (PORT_QUEUE_DEPTH = 2 places,
//   shared by all banks), and the port is told to stop (port_stop_N, registered). The second skid place is for the one
//   request the LSU may still send in the cycle it sees the stop (BUG-7).
//   For every bank an l1d_bank_order remembers in which order the skidded requests arrived; one per cycle, the oldest of
//   them is DRAINED into the bank's tracker when the bank has room. While something waits for a bank, new pins requests for
//   that bank also wait (so the bank serves its requests in SEND order, CC-4 / CC-5).
//
//   THE ROOM RULE is computed HERE from flops and pins (the bank's free count, its ring, the vector data place) and AGAIN
//   in the bank's tracker from the same flops: both sides reach the same answer every cycle (a simulation check compares
//   them), and no grant has to come back from the bank to the front (rule T-1, the lean S0's longest path).
//
//      port N pins --+--> bank decode addr[7:6] --> arrival_request (ring empty) ---> bank b tracker (allocation there)
//                    |                              not taken: join ring b, push skid N
//                    +--> skid N (2 places) --> head --> drain into bank b when it is the ring head and b has room
//      skid N occupied after this cycle --> flop --> port_stop_N
//
// STAGE / CYCLE
//   Before / in S0 of the banks. Estimated depth from the pins to the skid / ring / stop flops: bank decode 1 + rank 2 +
//   room compare 2 + join / push 1 + skid level update 2 = about 8 levels (PLAN_v6 path G); nothing waits for a grant.
//
// INSTANTIATED BY
//   l1d_top, instance u_front, one copy. It builds the skids u_skid_0 .. u_skid_3 (plus u_skid_line_3 for the VPU line) and
//   NUM_BANKS copies of l1d_bank_order (g_bank[b].u_order, BANK_ID = b).
//
// PARAMETERS
//   `L1D_PARAMS            all cache parameters (l1d_defines.vh); used here: LSU_PORTS (must be 4), NUM_BANKS (4), REQ_W (148),
//                          PC_W (64), TRK_ALLOC_MAX (4), BANK_LSB, BANK_W (2), LINE_W (512), TRK_CNT_W (4)
//   PORT_QUEUE_DEPTH       default 2: skid places per port (at least 2: one for the late request, BUG-7)
//   LINE_PORT_ON           default 0: 1 = port 3 is the VPU port: its 512-bit line travels with its requests (C-20)
//   local constants        SKID_CNT_W  = 2 bits: skid fill count 0..2;  STOP_MARGIN = 2: stop when fewer than 2 places would be free
//                          SKID_W = REQ_W + PC_W = 212 bits: a skid entry (packet and PC together)
//
// PORTS (name, direction, width at the defaults, meaning)                 N = 0, 1, 2, 3 (the request port), b = bank
//   clk, rst                     in   1                 clock; synchronous reset, active high
//   port_request_valid_N         in   1                 port N sends a request this cycle
//   port_request_packet_N        in   148               its request packet
//   port_request_pc_N            in   64                its instruction PC
//   port_stop_N                  out  1 (registered)    port N must not send (its skid holds a request)
//   port_request_line_3          in   512               (C-20) the line that comes with the VPU request this cycle
//   bank_tracker_free_count      in   4*4 = 16          per bank: free tracker entries (flops of the banks)
//   bank_vec_slot_available      in   4                 per bank: a vector data place is free
//   bank_arrival_request_N       out  4                 per bank: port N's pins request is for bank b and b's ring is empty
//   bank_drain_request           out  4                 per bank: the oldest skidded request for b asks for an entry
//   bank_drain_packet            out  4*148             per bank: its packet
//   bank_drain_pc                out  4*64              per bank: its PC
//   bank_arrival_line_data       out  4*512             (C-20) per bank: the line of a vector store arriving at b (pins or skid head)
//   bank_arrival_allocated_N     in   4                 per bank: the tracker took port N's arrival (simulation check only)
//   bank_drain_allocated         in   4                 per bank: the tracker took the drained request (simulation check only)
//
// WORKED EXAMPLE
//   cycle t: ports 0, 1, 2 send to bank 0 (8 free entries, ring empty): ranks 0, 1, 2 < 8 -> all three are allocated (port 0
//            issues fresh, 1 and 2 are booked one after the other). No skid, no stop.
//   cycle t: bank 2 has 1 free entry, ports 0 and 1 send to it: port 0 (rank 0) is allocated, port 1 (rank 1) joins ring 2 and
//            goes into skid 1; port_stop_1 = 1 from t+1. When bank 2 frees an entry, head port 1 is drained and popped.
//
// ASSUMPTIONS (checked in simulation)
//   The LSU sends at most one request in the cycle it sees port_stop_N = 1 (BUG-7), then none while it stays 1.
//   The front's room rule equals the tracker's allocation (bank_arrival_allocated_N / bank_drain_allocated).
// ============================================================================
`ifndef L1D_PORT_FRONT_V
`define L1D_PORT_FRONT_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_port_front #(
    `L1D_PARAMS,                                           // all cache parameters (l1d_defines.vh); LSU_PORTS must be 4
    parameter PORT_QUEUE_DEPTH = `L1D_PORT_QUEUE_DEPTH,    // default 2: skid places per port
    parameter LINE_PORT_ON     = 0                         // default 0: 1 = port 3 brings a 512-bit line (VPU, C-20)
) (
    clk, rst,
    port_request_valid_0, port_request_valid_1, port_request_valid_2, port_request_valid_3,
    port_request_packet_0, port_request_packet_1, port_request_packet_2, port_request_packet_3,
    port_request_pc_0, port_request_pc_1, port_request_pc_2, port_request_pc_3,
    port_stop_0, port_stop_1, port_stop_2, port_stop_3,
    port_request_line_3,
    bank_tracker_free_count, bank_vec_slot_available,
    bank_arrival_request_0, bank_arrival_request_1, bank_arrival_request_2, bank_arrival_request_3,
    bank_drain_request, bank_drain_packet, bank_drain_pc, bank_arrival_line_data,
    bank_arrival_allocated_0, bank_arrival_allocated_1, bank_arrival_allocated_2, bank_arrival_allocated_3, bank_drain_allocated
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // ---- local constants (value at the defaults) ----
    localparam SKID_CNT_W  = $clog2(PORT_QUEUE_DEPTH + 1);      // skid fill count 0..2, 2 bits
    localparam STOP_MARGIN = 2;                                 // stop when fewer than 2 skid places would be free
    localparam SKID_POP_LAST = 1;                               // TIMING (iteration 4): 1 = the skid fill level takes the pop (a late signal) as its last select
    localparam SKID_W      = REQ_W + PC_W;                      // a skid entry: {pc, packet}, 148 + 64 = 212 bits
    localparam NB          = NUM_BANKS;                         // banks, 4
    localparam NP          = LSU_PORTS;                         // ports, 4
    localparam RANK_W      = 3;                                 // rank among this cycle's arrivals for one bank, 0..3, 3 bits
    localparam NA          = (TRK_ALLOC_MAX < NP) ? TRK_ALLOC_MAX : NP;   // most allocations per bank per cycle, 4

    // ---- ports ----
    input  wire                         clk;                    // clock: all registers change on its rising edge
    input  wire                         rst;                    // synchronous reset, active high
    input  wire                         port_request_valid_0;   // port 0 (load / LR): a request this cycle
    input  wire                         port_request_valid_1;   // port 1 (load / LR)
    input  wire                         port_request_valid_2;   // port 2 (store / SC / AMO)
    input  wire                         port_request_valid_3;   // port 3 (VPU)
    input  wire [REQ_W-1:0]             port_request_packet_0;  // port 0 request packet, REQ_W = 148 bits
    input  wire [REQ_W-1:0]             port_request_packet_1;  // port 1 packet, 148 bits
    input  wire [REQ_W-1:0]             port_request_packet_2;  // port 2 packet, 148 bits
    input  wire [REQ_W-1:0]             port_request_packet_3;  // port 3 packet, 148 bits
    input  wire [PC_W-1:0]              port_request_pc_0;      // port 0 instruction PC, PC_W = 64 bits
    input  wire [PC_W-1:0]              port_request_pc_1;      // port 1 PC, 64 bits
    input  wire [PC_W-1:0]              port_request_pc_2;      // port 2 PC, 64 bits
    input  wire [PC_W-1:0]              port_request_pc_3;      // port 3 PC, 64 bits
    output reg                          port_stop_0;            // port 0: do not send (registered)
    output reg                          port_stop_1;            // port 1: do not send (registered)
    output reg                          port_stop_2;            // port 2: do not send (registered)
    output reg                          port_stop_3;            // port 3: do not send (registered)
    input  wire [LINE_W-1:0]            port_request_line_3;    // C-20: the line of the VPU request, LINE_W = 512 bits
    input  wire [NB*TRK_CNT_W-1:0]      bank_tracker_free_count;  // per bank: free tracker entries, 4*4 = 16 bits
    input  wire [NB-1:0]                bank_vec_slot_available;  // per bank: a vector data place is free, 4 bits
    output wire [NB-1:0]                bank_arrival_request_0; // per bank: port 0's pins request is for it (ring empty), 4 bits
    output wire [NB-1:0]                bank_arrival_request_1; // ... port 1
    output wire [NB-1:0]                bank_arrival_request_2; // ... port 2
    output wire [NB-1:0]                bank_arrival_request_3; // ... port 3
    output wire [NB-1:0]                bank_drain_request;     // per bank: its oldest skidded request asks for an entry, 4 bits
    output wire [NB*REQ_W-1:0]          bank_drain_packet;      // per bank: that request's packet, 4*148 bits
    output wire [NB*PC_W-1:0]           bank_drain_pc;          // per bank: its PC, 4*64 bits
    output wire [NB*LINE_W-1:0]         bank_arrival_line_data; // C-20: per bank: the line of an arriving vector store, 4*512 bits
    input  wire [NB-1:0]                bank_arrival_allocated_0;  // per bank: the tracker took port 0's arrival (check only)
    input  wire [NB-1:0]                bank_arrival_allocated_1;  // ... port 1 (check only)
    input  wire [NB-1:0]                bank_arrival_allocated_2;  // ... port 2 (check only)
    input  wire [NB-1:0]                bank_arrival_allocated_3;  // ... port 3 (check only)
    input  wire [NB-1:0]                bank_drain_allocated;   // per bank: the tracker took the drained request (check only)

    // ======================================================================
    // THE SKIDS: one l1d_fifo per port holding {pc, packet} (first-word fall-through), plus the VPU line FIFO beside skid 3
    // ======================================================================
    wire [NP-1:0]       port_valid;                             // pins requests, 4 bits
    wire [NP*REQ_W-1:0] port_packet;                            // pins packets, 4*148 bits
    wire [NP*PC_W-1:0]  port_pc;                                // pins PCs, 4*64 bits
    wire [NP-1:0]       skid_push;                              // the pins request was not taken: keep it, 4 bits
    wire [NP-1:0]       skid_pop;                               // the skid head was drained into its bank, 4 bits
    wire [NP-1:0]       skid_empty;                             // the skid is empty, 4 bits
    wire [NP-1:0]       skid_full;                              // the skid is full (check only), 4 bits
    wire [NP*SKID_W-1:0] skid_head;                             // skid heads {pc, packet}, 4*212 bits
    wire [NP*SKID_CNT_W-1:0] skid_used;                         // skid places in use, 4*2 bits
    wire [LINE_W-1:0]   skid_head_line_3;                       // the line of the VPU skid head, 512 bits
    assign port_valid  = {port_request_valid_3, port_request_valid_2, port_request_valid_1, port_request_valid_0};
    assign port_packet = {port_request_packet_3, port_request_packet_2, port_request_packet_1, port_request_packet_0};
    assign port_pc     = {port_request_pc_3, port_request_pc_2, port_request_pc_1, port_request_pc_0};
    genvar sk;                                                  // loop variable: port whose skid is built
    generate
        for (sk = 0; sk < NP; sk = sk + 1) begin : g_skid
            wire [SKID_W-1:0] skid_in;                          // {pc, packet} of the pins request, 212 bits
            assign skid_in = {port_pc[sk*PC_W +: PC_W], port_packet[sk*REQ_W +: REQ_W]};
            l1d_fifo #(.WIDTH(SKID_W), .DEPTH(PORT_QUEUE_DEPTH), .CNT_W(SKID_CNT_W), .POP_LAST(SKID_POP_LAST)) u_skid (
                .clk (clk), .rst (rst), .push (skid_push[sk]), .push_data (skid_in), .pop (skid_pop[sk]),
                .front_data (skid_head[sk*SKID_W +: SKID_W]), .empty (skid_empty[sk]), .full (skid_full[sk]),
                .used_slots (skid_used[sk*SKID_CNT_W +: SKID_CNT_W]), .free_slots (), .free_slots_next ()
            );
        end
    endgenerate
    // C-20: the VPU line waits in a second FIFO that moves exactly with the port 3 skid
    generate
        if (LINE_PORT_ON != 0) begin : g_skid_line
            l1d_fifo #(.WIDTH(LINE_W), .DEPTH(PORT_QUEUE_DEPTH), .CNT_W(SKID_CNT_W)) u_skid_line_3 (
                .clk (clk), .rst (rst), .push (skid_push[NP-1]), .push_data (port_request_line_3), .pop (skid_pop[NP-1]),
                .front_data (skid_head_line_3), .empty (), .full (), .used_slots (), .free_slots (), .free_slots_next ()
            );
        end else begin : g_no_skid_line
            assign skid_head_line_3 = {LINE_W{1'b0}};
        end
    endgenerate

    // ---- the bank of every pins request and every skid head: the bank bits of the address (address[7:6]) ----
    wire [NP*BANK_W-1:0] pin_bank;                              // 4*2 bits
    wire [NP*BANK_W-1:0] head_bank;                             // 4*2 bits
    wire [NP-1:0]        pin_line_store;                        // the pins request is a vector store (needs a data place), 4 bits
    wire [NP-1:0]        head_line_store;                       // the skid head is a vector store, 4 bits
    genvar bk_p;                                                // loop variable: port
    generate
        for (bk_p = 0; bk_p < NP; bk_p = bk_p + 1) begin : g_bank_of
            assign pin_bank[bk_p*BANK_W +: BANK_W]  = port_packet[bk_p*REQ_W + REQ_ADDR_LSB + BANK_LSB +: BANK_W];
            assign head_bank[bk_p*BANK_W +: BANK_W] = skid_head[bk_p*SKID_W + REQ_ADDR_LSB + BANK_LSB +: BANK_W];
            assign pin_line_store[bk_p]  = port_packet[bk_p*REQ_W + REQ_LINE_BIT] & port_packet[bk_p*REQ_W + REQ_RDWR_BIT];
            assign head_line_store[bk_p] = skid_head[bk_p*SKID_W + REQ_LINE_BIT] & skid_head[bk_p*SKID_W + REQ_RDWR_BIT];
        end
    endgenerate

    // ======================================================================
    // PER BANK: the room rule (the same as the tracker's), the ring, the drain
    // ======================================================================
    wire [NB*NP-1:0] pin_for_bank;                              // [b*NP + p]: port p's pins request is for bank b, 16 bits
    wire [NB*NP-1:0] arrival_request;                           // ... and bank b's ring is empty, 16 bits
    wire [NB*NP-1:0] alloc_pred;                                // ... and the tracker takes it (the front's copy of the room rule), 16 bits
    wire [NB*NP-1:0] head_port;                                 // [b*NP + p]: port p's skid head is bank b's ring head, 16 bits
    wire [NB-1:0]    ring_empty_q;                              // per bank: nothing waits, 4 bits
    wire [NB-1:0]    drain_alloc_pred;                          // per bank: the drained request is taken this cycle, 4 bits
    genvar b;                                                   // loop variable: bank
    generate
        for (b = 0; b < NB; b = b + 1) begin : g_bank
            // ---- arrivals of this bank: which ports, their ranks, the room rule ----
            wire [TRK_CNT_W-1:0] free_count;                    // free tracker entries of this bank, 4 bits
            wire                 vec_ok;                        // a vector data place is free in this bank
            wire [NP-1:0]        eligible;                      // pins requests for this bank that may take an entry (place ok), 4 bits
            reg  [NP-1:0]        rank_onehot [0:NP-1];          // TIMING (iteration 1): rank of each port among the eligible ones as a one-hot count
                                                                //   (bit k = exactly k lower ports are eligible), 4 x 4 bits; replaces the 3-bit adder chain
            reg  [NP:1]          free_at_least;                 // TIMING (iteration 1, 5): bit k = this bank's ring is empty AND the tracker has at least k free entries, 4 bits
                                                                //   (replaces the compare "rank < free_count": rank < free_count  <=>  free_count >= rank + 1)
            reg  [NP-1:0]        alloc_here;                    // the tracker takes the port's request, 4 bits
            wire [NP-1:0]        join_here;                     // the port's request waits (joins the ring), 4 bits
            wire [NP-1:0]        head_port_onehot_b;            // port of the oldest waiting request of this bank, one-hot, 4 bits
            assign free_count = bank_tracker_free_count[b*TRK_CNT_W +: TRK_CNT_W];
            assign vec_ok     = bank_vec_slot_available[b];
            genvar p;                                           // loop variable: port
            for (p = 0; p < NP; p = p + 1) begin : g_port
                assign pin_for_bank[b*NP + p]    = port_valid[p] && (pin_bank[p*BANK_W +: BANK_W] == b);
                assign arrival_request[b*NP + p] = pin_for_bank[b*NP + p] && ring_empty_q[b];
                // TIMING (iteration 5): "eligible" no longer contains ring_empty_q; the ring-empty flop is folded into the
                // free_at_least flags below, so the flop (a high-fan-out signal) is off the pins -> rank -> allocate -> skid push path
                assign eligible[p]               = pin_for_bank[b*NP + p] && (!pin_line_store[p] || vec_ok);
                assign alloc_pred[b*NP + p]      = alloc_here[p];
                assign join_here[p]              = pin_for_bank[b*NP + p] && !alloc_here[p];
                assign head_port[b*NP + p]       = head_port_onehot_b[p];
            end
            // TIMING (iteration 1, same function as before): the tracker takes port p's request when it is eligible, fewer than
            // free_count lower ports are eligible (rank < free_count) and rank < NA. Instead of adding up the lower eligible
            // bits and comparing with free_count, the lower eligible ports are counted in ONE-HOT form (a shift per eligible
            // port, no adder) and AND-ed with the "at least k free" flags, which come straight from the free_count flops.
            // Example: ports 0 and 2 eligible, free_count = 2: rank_onehot[0] = 0001 (alloc: 2 >= 1), rank_onehot[2] = 0010
            // (one lower port; alloc: 2 >= 2), so both are taken; with free_count = 1 only port 0 is taken.
            integer rk_p;                                       // loop variable: port whose rank is built
            integer rk_k;                                       // loop variable: rank value (number of lower eligible ports)
            reg  [NP-1:0] rank_run;                             // one-hot count of the eligible ports below the current one, 4 bits
            always @* begin
                for (rk_k = 1; rk_k <= NP; rk_k = rk_k + 1)
                    free_at_least[rk_k] = ring_empty_q[b] && (free_count >= rk_k);   // iteration 5: includes "the ring is empty"
                rank_run = {{(NP-1){1'b0}}, 1'b1};              // below port 0 there is no port: count 0
                for (rk_p = 0; rk_p < NP; rk_p = rk_p + 1) begin
                    rank_onehot[rk_p] = rank_run;
                    alloc_here[rk_p]  = 1'b0;
                    for (rk_k = 0; rk_k < NP; rk_k = rk_k + 1)
                        if (rk_k < NA)
                            alloc_here[rk_p] = alloc_here[rk_p] | (rank_run[rk_k] & free_at_least[rk_k + 1]);
                    alloc_here[rk_p] = alloc_here[rk_p] & eligible[rk_p];
                    if (eligible[rk_p])
                        rank_run = {rank_run[NP-2:0], 1'b0};    // one more eligible port below the next port
                end
            end
            // ---- the ring of skidded requests ----
            l1d_bank_order #(`L1D_PARAMS_PASS, .BANK_ID(b), .PORT_QUEUE_DEPTH(PORT_QUEUE_DEPTH)) u_order (
                .clk              (clk),
                .rst              (rst),
                .join_0           (join_here[0]),
                .join_1           (join_here[1]),
                .join_2           (join_here[2]),
                .join_3           (join_here[3]),
                .pop              (drain_alloc_pred[b]),
                .head_port_onehot (head_port_onehot_b),
                .ring_empty_q     (ring_empty_q[b])
            );
            // ---- the drain: the ring head's skid head, when it is for this bank (it may still be behind an older request of the
            //      same port for another bank) and the bank has room (a vector store also needs a data place) ----
            reg  [SKID_W-1:0] drain_entry;                      // the skid head of the ring-head port, 212 bits
            reg               drain_head_ok;                    // that skid head exists and is for this bank
            reg               drain_line_store;                 // it is a vector store
            integer dr_p;                                       // loop variable: port
            always @* begin
                drain_entry      = {SKID_W{1'b0}};
                drain_head_ok    = 1'b0;
                drain_line_store = 1'b0;
                for (dr_p = 0; dr_p < NP; dr_p = dr_p + 1) begin
                    drain_entry      = drain_entry | ({SKID_W{head_port_onehot_b[dr_p]}} & skid_head[dr_p*SKID_W +: SKID_W]);
                    drain_head_ok    = drain_head_ok | (head_port_onehot_b[dr_p] && !skid_empty[dr_p] && (head_bank[dr_p*BANK_W +: BANK_W] == b));
                    drain_line_store = drain_line_store | (head_port_onehot_b[dr_p] & head_line_store[dr_p]);
                end
            end
            assign bank_drain_request[b] = !ring_empty_q[b] && drain_head_ok;
            assign drain_alloc_pred[b]   = bank_drain_request[b] && (free_count != {TRK_CNT_W{1'b0}}) && (!drain_line_store || vec_ok);
            assign bank_drain_packet[b*REQ_W +: REQ_W] = drain_entry[0 +: REQ_W];
            assign bank_drain_pc[b*PC_W +: PC_W]       = drain_entry[REQ_W +: PC_W];
            // only the VPU port (3) has a line: the pins line while the ring is empty, else the line of the VPU skid head
            assign bank_arrival_line_data[b*LINE_W +: LINE_W] = ring_empty_q[b] ? port_request_line_3 : skid_head_line_3;
        end
    endgenerate
    genvar out_b;                                               // loop variable: bank
    generate
        for (out_b = 0; out_b < NB; out_b = out_b + 1) begin : g_out
            assign bank_arrival_request_0[out_b] = arrival_request[out_b*NP + 0];
            assign bank_arrival_request_1[out_b] = arrival_request[out_b*NP + 1];
            assign bank_arrival_request_2[out_b] = arrival_request[out_b*NP + 2];
            assign bank_arrival_request_3[out_b] = arrival_request[out_b*NP + 3];
        end
    endgenerate

    // ======================================================================
    // PER PORT: push / pop of the skid (OR over the banks: a port's request goes to exactly one bank) and the stop
    // ======================================================================
    reg  [NP-1:0] port_allocated;                               // the pins request was taken by its bank, 4 bits
    reg  [NP-1:0] port_drained;                                 // the skid head was drained into its bank, 4 bits
    integer pp;                                                 // loop variable: port
    integer pb;                                                 // loop variable: bank
    always @* begin
        for (pp = 0; pp < NP; pp = pp + 1) begin
            port_allocated[pp] = 1'b0;
            port_drained[pp]   = 1'b0;
            for (pb = 0; pb < NB; pb = pb + 1) begin
                port_allocated[pp] = port_allocated[pp] | alloc_pred[pb*NP + pp];
                port_drained[pp]   = port_drained[pp]   | (drain_alloc_pred[pb] & head_port[pb*NP + pp]);
            end
        end
    end
    assign skid_push = port_valid & ~port_allocated;            // a pins request that was not taken waits in its skid
    assign skid_pop  = port_drained;                            // a drained skid head leaves

    // stop: fewer than STOP_MARGIN places would be free after this cycle (used + push - pop > DEPTH - STOP_MARGIN). Both push and
    // pop come from flops and pins (no late grant any more): one small add and a compare into the stop flop.
    // Example: skid empty, pins request not taken: 0 + 1 - 0 = 1, 1 + 2 > 2 -> stop. Taken: no stop.
    function stop_after;
        input [SKID_CNT_W-1:0] used_now;                        // skid places in use now, 2 bits
        input                  push_now;                        // a pins request enters the skid
        input                  pop_now;                         // the skid head leaves
        reg   [SKID_CNT_W:0]   used_next;                       // skid places in use now (zero-extended), 3 bits
        reg   [SKID_CNT_W:0]   used_if_push;                    // TIMING (iteration 3): used + 1 (only a push), 3 bits
        reg   [SKID_CNT_W:0]   used_if_pop;                     // TIMING (iteration 3): used - 1 (only a pop), 3 bits
        reg                    stop_if_hold;                    // TIMING (iteration 3): stop when push and pop are equal (used unchanged)
        reg                    stop_if_push;                    // TIMING (iteration 3): stop when only a push happens
        reg                    stop_if_pop;                     // TIMING (iteration 3): stop when only a pop happens
        begin
            // TIMING (iteration 3, same function): the late push / pop signals only SELECT between three answers that are
            // computed from the used flops alone (no adder behind push / pop). A push and a pop in the same cycle cancel.
            // Example: used = 1: hold -> 1 + 2 > 2 stop; push -> 2 + 2 > 2 stop; pop -> 0 + 2 > 2 no stop.
            used_next    = {1'b0, used_now};
            used_if_push = used_next + {{SKID_CNT_W{1'b0}}, 1'b1};
            used_if_pop  = used_next - {{SKID_CNT_W{1'b0}}, 1'b1};
            stop_if_hold = (used_next    + STOP_MARGIN > PORT_QUEUE_DEPTH);
            stop_if_push = (used_if_push + STOP_MARGIN > PORT_QUEUE_DEPTH);
            stop_if_pop  = (used_if_pop  + STOP_MARGIN > PORT_QUEUE_DEPTH);
            if (push_now == pop_now) stop_after = stop_if_hold;
            else if (push_now)       stop_after = stop_if_push;
            else                     stop_after = stop_if_pop;
        end
    endfunction
    always @(posedge clk) begin : stop_proc
        if (rst) begin
            port_stop_0 <= 1'b0;
            port_stop_1 <= 1'b0;
            port_stop_2 <= 1'b0;
            port_stop_3 <= 1'b0;
        end else begin
            port_stop_0 <= stop_after(skid_used[0*SKID_CNT_W +: SKID_CNT_W], skid_push[0], skid_pop[0]);
            port_stop_1 <= stop_after(skid_used[1*SKID_CNT_W +: SKID_CNT_W], skid_push[1], skid_pop[1]);
            port_stop_2 <= stop_after(skid_used[2*SKID_CNT_W +: SKID_CNT_W], skid_push[2], skid_pop[2]);
            port_stop_3 <= stop_after(skid_used[3*SKID_CNT_W +: SKID_CNT_W], skid_push[3], skid_pop[3]);
        end
    end // always stop_proc

    // ======================================================================
    // SIMULATION CHECKS (always on): exactly 4 ports; the stop rule (BUG-7); the front's room rule equals the trackers'
    // ======================================================================
`ifndef SYNTHESIS
    wire [NB*NP-1:0] bank_allocated_flat;                       // the trackers' answers, 16 bits
    genvar ck_b;                                                // loop variable: bank
    generate
        for (ck_b = 0; ck_b < NB; ck_b = ck_b + 1) begin : g_ck
            assign bank_allocated_flat[ck_b*NP +: NP] = {bank_arrival_allocated_3[ck_b], bank_arrival_allocated_2[ck_b],
                                                         bank_arrival_allocated_1[ck_b], bank_arrival_allocated_0[ck_b]};
        end
    endgenerate
    integer ck_p;                                               // loop variable: port
    initial if (LSU_PORTS != 4)
        $display("L1D FRONT ERROR: LSU_PORTS = %0d, the named ports _0 .. _3 need LSU_PORTS = 4", LSU_PORTS);
    always @(posedge clk) begin
        if (!rst) begin
            for (ck_p = 0; ck_p < NP; ck_p = ck_p + 1)
                if (skid_push[ck_p] && skid_full[ck_p])
                    $display("[%0t] L1D SKID ERROR port %0d: request pushed into its full skid (stop rule broken)", $time, ck_p);
            if (bank_allocated_flat != alloc_pred)
                $display("[%0t] L1D FRONT ERROR: room rule disagrees with the trackers (front %b, banks %b)", $time, alloc_pred, bank_allocated_flat);
            if (bank_drain_allocated != drain_alloc_pred)
                $display("[%0t] L1D FRONT ERROR: drain rule disagrees with the trackers (front %b, banks %b)", $time, drain_alloc_pred, bank_drain_allocated);
        end
    end
`endif
endmodule

`default_nettype wire
`endif
