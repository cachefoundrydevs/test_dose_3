// ============================================================================
// l1d_bank_order.v  -  the arrival order of the SKIDDED requests of one bank (cache_v6)
// ============================================================================
// IN SIMPLE WORDS
//   A request that its bank could not take at arrival (no free tracker entry, or something older already waits) waits in its
//   port's skid (l1d_port_front). This block remembers, for its bank, in which order those waiting requests arrived: a ring of
//   port numbers. Its head names the port whose skid head is the oldest waiting request for this bank; the front drains that
//   one request per cycle into the bank's tracker (l1d_req_tracker) when the bank has room, and pops the ring.
//   So a bank sees its requests in SEND order, whatever port they came from (CC-4 / CC-5), also across a full tracker.
//
//   cache_v6 change against the lean S0: the ring no longer offers packets and no longer sees a late grant. Its inputs are
//   "port N joins now" (from the front's room rule: flops and pins) and "the head is drained now" (flops), so the ring update
//   is ONE outcome (no 2-outcome late select any more).
//
//      join_N (port N's pins request for this bank is not taken: it waits) ---> ring write (lower port first in one cycle)
//      pop (the head is drained into the tracker) -----------------------------> ring read pointer moves
//      head_port_onehot / head_valid / ring_empty_q ---> l1d_port_front (drain select), l1d_bank_s0 (arrival rule)
//
// STAGE / CYCLE
//   Before / in S0. All outputs come from flops (the head port is an AND-OR of the ring by the one-hot read pointer: 3 levels).
//
// INSTANTIATED BY
//   l1d_port_front, instance g_bank[b].u_order, NUM_BANKS = 4 copies (BANK_ID = b).
//
// PARAMETERS
//   `L1D_PARAMS            all cache parameters (l1d_defines.vh); used here: LSU_PORTS (must be 4), PORT_W (2)
//   BANK_ID                default 0: number of this bank (messages only)
//   PORT_QUEUE_DEPTH       default 2: skid places per port (sets the ring size)
//   local constants        ORDER_NEED  = 4 * PORT_QUEUE_DEPTH = 8: most requests that can wait for one bank
//                          ORDER_DEPTH = 8 (ORDER_NEED rounded up to a power of 2): ring places
//                          ORDER_CNT_W = 4 bits: ring fill count 0..8
//
// PORTS (name, direction, width at the defaults, meaning)              N = 0, 1, 2, 3 (the request port)
//   clk, rst                       in   1      clock; synchronous reset, active high
//   join_N                         in   1      port N's pins request for this bank is not taken now: it joins the ring
//   pop                            in   1      the ring head is drained into the tracker this cycle
//   head_port_onehot               out  4      the port whose skid head is the oldest waiting request (one-hot; zero when empty)
//   ring_empty_q                   out  1      flop: nothing waits for this bank (pins requests may be allocated at once)
//
// WORKED EXAMPLE (tracker of this bank full in cycle 1)
//   cycle 1: ports 0 and 2 send A, B to this bank and are not taken: join_0 = join_2 = 1 -> ring = 0, 2 (A older than B).
//   cycle 2: port 1 sends C; the ring is not empty, so C is not taken either: ring = 0, 2, 1. An entry frees.
//   cycle 3: head = port 0: the front drains A (pop) -> ring = 2, 1.  cycle 4: B;  cycle 5: C. The bank sees A, B, C.
//
// ASSUMPTIONS (checked in simulation)
//   The ring never overflows: at most ORDER_NEED requests can wait for one bank (one skid place each).
// ============================================================================
`ifndef L1D_BANK_ORDER_V
`define L1D_BANK_ORDER_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_bank_order #(
    `L1D_PARAMS,                                           // all cache parameters (l1d_defines.vh); LSU_PORTS must be 4
    parameter BANK_ID          = 0,                        // default 0: number of this bank (messages only)
    parameter PORT_QUEUE_DEPTH = `L1D_PORT_QUEUE_DEPTH     // default 2: skid places per port
) (
    clk, rst,
    join_0, join_1, join_2, join_3,
    pop,
    head_port_onehot, ring_empty_q
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // ---- local constants (value at the defaults) ----
    localparam ORDER_NEED  = LSU_PORTS * PORT_QUEUE_DEPTH;      // most requests waiting for this bank, 4*2 = 8
    localparam ORDER_DEPTH = 1 << $clog2(ORDER_NEED);           // ring places (power of 2), 8
    localparam ORDER_CNT_W = $clog2(ORDER_DEPTH + 1);           // ring fill count, 4 bits
    localparam JOIN_CNT_W  = 3;                                 // requests joining the ring in one cycle, 0..4, 3 bits

    // ---- ports ----
    input  wire                 clk;                  // clock: all registers change on its rising edge
    input  wire                 rst;                  // synchronous reset, active high
    input  wire                 join_0;               // port 0 (load / LR): its pins request waits for this bank
    input  wire                 join_1;               // port 1 (load / LR)
    input  wire                 join_2;               // port 2 (store / SC / AMO)
    input  wire                 join_3;               // port 3 (VPU)
    input  wire                 pop;                  // the ring head is drained now
    output wire [LSU_PORTS-1:0] head_port_onehot;     // port of the oldest waiting request, one-hot, 4 bits
    output reg                  ring_empty_q;         // flop: nothing waits for this bank

    // ======================================================================
    // THE ORDER RING: order_port[k] = port number (0..3) of the k-th waiting request for this bank, in arrival order
    // ======================================================================
    reg  [PORT_W-1:0]      order_port [0:ORDER_DEPTH-1];        // ring of port numbers, 8 places of PORT_W = 2 bits
    reg  [ORDER_DEPTH-1:0] order_read_onehot;                   // one-hot: place of the oldest waiting request, 8 bits
    reg  [ORDER_DEPTH-1:0] order_write_onehot;                  // one-hot: next free place, 8 bits
    reg  [ORDER_CNT_W-1:0] order_count;                         // waiting requests of this bank, 0..8, 4 bits
    wire [LSU_PORTS-1:0]   join_now;                                // the ports that join the ring now, 4 bits
    assign join_now = {join_3, join_2, join_1, join_0};

    // move a one-hot pointer step_cnt places forward around the ring
    function [ORDER_DEPTH-1:0] rotate_left;
        input [ORDER_DEPTH-1:0] pointer_onehot;                 // the pointer, one-hot, 8 bits
        input integer           step_cnt;                       // places to move (0..4)
        integer                 bit_idx;                        // loop variable: bit of the pointer
        begin
            for (bit_idx = 0; bit_idx < ORDER_DEPTH; bit_idx = bit_idx + 1)
                rotate_left[(bit_idx + step_cnt) % ORDER_DEPTH] = pointer_onehot[bit_idx];
        end
    endfunction
    // the one-hot pointer moved forward by a small count (a mux over the constant rotations 0..4)
    function [ORDER_DEPTH-1:0] rotate_by_count;
        input [ORDER_DEPTH-1:0] pointer_onehot;                 // 8 bits
        input [JOIN_CNT_W-1:0]  step_cnt;                       // 0..4, 3 bits
        integer                 step_idx;                       // loop variable: constant rotation
        begin
            rotate_by_count = pointer_onehot;
            for (step_idx = 1; step_idx <= 4; step_idx = step_idx + 1)
                if (step_cnt == step_idx) rotate_by_count = rotate_left(pointer_onehot, step_idx);
        end
    endfunction

    // ---- the ring head: port number of the oldest waiting request (AND-OR read at the one-hot read pointer), decoded one-hot ----
    reg     [PORT_W-1:0] oldest_port;                           // PORT_W = 2 bits
    integer              read_place;                            // loop variable: ring place
    always @* begin
        oldest_port = {PORT_W{1'b0}};
        for (read_place = 0; read_place < ORDER_DEPTH; read_place = read_place + 1)
            oldest_port = oldest_port | ({PORT_W{order_read_onehot[read_place]}} & order_port[read_place]);
    end
    genvar hp_idx;                                              // loop variable: port
    generate
        for (hp_idx = 0; hp_idx < LSU_PORTS; hp_idx = hp_idx + 1) begin : g_head
            assign head_port_onehot[hp_idx] = !ring_empty_q && (oldest_port == hp_idx);
        end
    endgenerate

    // ======================================================================
    // WRITE THE RING: every joining port gets the next place, lower port first (rank = number of lower joining ports)
    // ======================================================================
    reg  [JOIN_CNT_W-1:0]  rank_1, rank_2, rank_3;             // joining lower ports of ports 1..3, 3 bits each (port 0: rank 0)
    reg  [ORDER_DEPTH-1:0] place_0, place_1, place_2, place_3;  // the place of each port, one-hot, 8 bits each
    reg  [ORDER_DEPTH-1:0] place_write_en;                      // places written now, 8 bits
    reg  [ORDER_DEPTH*PORT_W-1:0] place_write_port;             // the port number written to each place, 16 bits
    reg  [JOIN_CNT_W-1:0]  join_cnt;                            // ports joining now, 0..4, 3 bits
    integer                place_idx;                           // loop variable: ring place
    always @* begin
        rank_1   = {2'b00, join_now[0]};
        rank_2   = {2'b00, join_now[0]} + {2'b00, join_now[1]};
        rank_3   = {2'b00, join_now[0]} + {2'b00, join_now[1]} + {2'b00, join_now[2]};
        join_cnt = rank_3 + {2'b00, join_now[3]};
        place_0  = order_write_onehot;
        place_1  = rotate_by_count(order_write_onehot, rank_1);
        place_2  = rotate_by_count(order_write_onehot, rank_2);
        place_3  = rotate_by_count(order_write_onehot, rank_3);
        place_write_en = ({ORDER_DEPTH{join_now[0]}} & place_0) | ({ORDER_DEPTH{join_now[1]}} & place_1) |
                         ({ORDER_DEPTH{join_now[2]}} & place_2) | ({ORDER_DEPTH{join_now[3]}} & place_3);
        for (place_idx = 0; place_idx < ORDER_DEPTH; place_idx = place_idx + 1)
            place_write_port[place_idx*PORT_W +: PORT_W] = ({PORT_W{join_now[1] && place_1[place_idx]}} & 2'd1) |
                                                           ({PORT_W{join_now[2] && place_2[place_idx]}} & 2'd2) |
                                                           ({PORT_W{join_now[3] && place_3[place_idx]}} & 2'd3);
    end
    wire [ORDER_CNT_W-1:0] order_count_next;                    // 4 bits
    assign order_count_next = order_count + {{(ORDER_CNT_W-JOIN_CNT_W){1'b0}}, join_cnt} - {{(ORDER_CNT_W-1){1'b0}}, pop};
    integer write_place;                                        // loop variable: ring place
    always @(posedge clk) begin : ring_proc
        if (rst) begin
            order_read_onehot  <= {{(ORDER_DEPTH-1){1'b0}}, 1'b1};
            order_write_onehot <= {{(ORDER_DEPTH-1){1'b0}}, 1'b1};
            order_count        <= {ORDER_CNT_W{1'b0}};
            ring_empty_q       <= 1'b1;
        end else begin
            for (write_place = 0; write_place < ORDER_DEPTH; write_place = write_place + 1)
                if (place_write_en[write_place])
                    order_port[write_place] <= place_write_port[write_place*PORT_W +: PORT_W];
            order_write_onehot <= rotate_by_count(order_write_onehot, join_cnt);
            if (pop)
                order_read_onehot <= rotate_left(order_read_onehot, 1);
            order_count  <= order_count_next;
            ring_empty_q <= (order_count_next == {ORDER_CNT_W{1'b0}});
        end
    end // always ring_proc

    // ---- simulation checks: exactly 4 ports; the ring can never overflow; no pop of an empty ring ----
`ifndef SYNTHESIS
    initial if (LSU_PORTS != 4)
        $display("L1D ORDER ERROR bank %0d: LSU_PORTS = %0d, the ports _0 .. _3 need LSU_PORTS = 4", BANK_ID, LSU_PORTS);
    always @(posedge clk) begin
        if (!rst && (order_count + join_cnt > ORDER_DEPTH))
            $display("[%0t] L1D ORDER ERROR bank %0d: ring overflow (%0d waiting + %0d new)", $time, BANK_ID, order_count, join_cnt);
        if (!rst && pop && ring_empty_q)
            $display("[%0t] L1D ORDER ERROR bank %0d: pop of an empty ring", $time, BANK_ID);
    end
`endif
`ifdef L1D_DEBUG
    always @(posedge clk) begin
        if (!rst && (|join_now))
            $display("[%0t] L1D ORDER bank %0d: ports %b join the ring (%0d waiting after)", $time, BANK_ID, join_now, order_count_next);
        if (!rst && pop)
            $display("[%0t] L1D ORDER bank %0d: head port %0d drained", $time, BANK_ID, oldest_port);
    end
`endif
endmodule

`default_nettype wire
`endif
