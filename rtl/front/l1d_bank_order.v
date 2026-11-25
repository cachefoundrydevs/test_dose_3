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
// TIMING-OPTIMISED BODY (2026-11-23, iterations BO-1 .. BO-8, kept iteration 8; history in l1d_bank_order_timing.md):
// the logic below is the comment-stripped public copy proven equal to the old file (names, Yosys equivalence). KNOWN GAP: the new logic
// (count one-hot select words, join thresholds, rank-based order_port write) still needs the full comment set of CODING_RULES V-1.7 / V-6.13.
// NOTE: body below = optimised, proven-equal version (port_front 5 iterations, kept: iteration 5 = 299.7 ps; comments in the body are partly missing, see CHAT_LOG)
`ifndef L1D_BANK_ORDER_V
`define L1D_BANK_ORDER_V
`default_nettype none
`include "l1d_defines.vh"
module l1d_bank_order #(
    `L1D_PARAMS,
    parameter BANK_ID          = 0,
    parameter PORT_QUEUE_DEPTH = `L1D_PORT_QUEUE_DEPTH
) (
    clk, rst,
    join_0, join_1, join_2, join_3,
    join_prefix_at_least,
    pop, drain_ready_port,
    head_port_onehot, ring_empty_q
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"
    localparam ORDER_NEED  = LSU_PORTS * PORT_QUEUE_DEPTH;
    localparam ORDER_DEPTH = 1 << $clog2(ORDER_NEED);
    localparam ORDER_CNT_W = $clog2(ORDER_DEPTH + 1);
    localparam JOIN_CNT_W  = 3;
    input  wire                 clk;
    input  wire                 rst;
    input  wire                 join_0;
    input  wire                 join_1;
    input  wire                 join_2;
    input  wire                 join_3;
    // bit (k-1)*LSU_PORTS + (m-1): at least m of the ports 0 .. k-1 join this cycle (k, m = 1 .. LSU_PORTS; m > k reads 0).
    // Thresholds computed by l1d_port_front straight from the pins and the room rule, equal to the counts of join_0 .. join_3.
    input  wire [LSU_PORTS*LSU_PORTS-1:0] join_prefix_at_least; // LSU_PORTS*LSU_PORTS = 16 bits
    input  wire                 pop;              // = the head's drain_ready_port bit when the ring is not empty (checks only)
    input  wire [LSU_PORTS-1:0] drain_ready_port; // LSU_PORTS = 4 bits: port p's skid head could be drained now (l1d_port_front)
    output wire [LSU_PORTS-1:0] head_port_onehot;
    output reg                  ring_empty_q;
    reg  [PORT_W-1:0]      order_port [0:ORDER_DEPTH-1];
    reg  [ORDER_DEPTH-1:0] order_read_onehot;
    reg  [ORDER_DEPTH-1:0] order_write_onehot;
    reg  [ORDER_CNT_W-1:0] order_count;
    wire [LSU_PORTS-1:0]   join_now;
    assign join_now = {join_3, join_2, join_1, join_0};
    function [ORDER_DEPTH-1:0] rotate_left;
        input [ORDER_DEPTH-1:0] pointer_onehot;
        input integer           step_cnt;
        integer                 bit_idx;
        begin
            for (bit_idx = 0; bit_idx < ORDER_DEPTH; bit_idx = bit_idx + 1)
                rotate_left[(bit_idx + step_cnt) % ORDER_DEPTH] = pointer_onehot[bit_idx];
        end
    endfunction
    function [ORDER_DEPTH-1:0] rotate_by_count;
        input [ORDER_DEPTH-1:0] pointer_onehot;
        input [JOIN_CNT_W-1:0]  step_cnt;
        integer                 step_idx;
        begin
            rotate_by_count = pointer_onehot;
            for (step_idx = 1; step_idx <= 4; step_idx = step_idx + 1)
                if (step_cnt == step_idx) rotate_by_count = rotate_left(pointer_onehot, step_idx);
        end
    endfunction
    reg     [PORT_W-1:0] oldest_port;
    integer              read_place;
    always @* begin
        oldest_port = {PORT_W{1'b0}};
        for (read_place = 0; read_place < ORDER_DEPTH; read_place = read_place + 1)
            oldest_port = oldest_port | ({PORT_W{order_read_onehot[read_place]}} & order_port[read_place]);
    end
    // pop decided here from the head index bits and the per-port ready flags (exactly the front's drain_alloc_pred =
    // OR over p of head_port_onehot[p] AND drain_ready_port[p]): a 2^PORT_W:1 mux by oldest_port, no one-hot decode
    // and no head_port_onehot fan-out on the pop path
    wire [(1 << PORT_W)-1:0] ready_by_index; // 2^PORT_W = 4 bits: drain_ready_port padded to every oldest_port value
    wire                     pop_from_head;  // the ring head is drained this cycle
    assign ready_by_index = {{(1 << PORT_W){1'b0}}, drain_ready_port}; // zero-extended (the upper zeros are cut off)
    assign pop_from_head  = !ring_empty_q && ready_by_index[oldest_port];
    genvar hp_idx;
    generate
        for (hp_idx = 0; hp_idx < LSU_PORTS; hp_idx = hp_idx + 1) begin : g_head
            assign head_port_onehot[hp_idx] = !ring_empty_q && (oldest_port == hp_idx);
        end
    endgenerate
    reg  [JOIN_CNT_W-1:0]  rank_1, rank_2, rank_3;
    reg  [ORDER_DEPTH-1:0] place_0, place_1, place_2, place_3;
    reg  [ORDER_DEPTH-1:0] place_write_en;
    reg  [ORDER_DEPTH*PORT_W-1:0] place_write_port;
    reg  [JOIN_CNT_W-1:0]  join_cnt;
    integer                place_idx;
    reg  [LSU_PORTS:0]     join_cnt_onehot;
    reg  [LSU_PORTS:0]     rank_before_onehot [0:LSU_PORTS-1];
    reg  [LSU_PORTS:0]     join_run;
    reg  [ORDER_CNT_W-1:0] count_next_sel;
    reg                    ring_empty_next;
    reg  [ORDER_CNT_W-1:0] count_candidate;
    integer                join_port_idx;
    integer                join_count_idx;
    integer                pop_value_idx;
    localparam COUNT_VALUES = 1 << ORDER_CNT_W;
    // next count bits [ORDER_CNT_W-1:0] and empty flag [ORDER_CNT_W] for each join count j (slice j), from the count flops and rst only
    localparam NEXT_W       = ORDER_CNT_W + 1;   // NEXT_W = 5 bits: next count (4) and empty flag (1)
    reg  [(LSU_PORTS+1)*NEXT_W-1:0] next_if_no_pop;  // (LSU_PORTS+1)*NEXT_W = 25 bits: count + j, empty flag (rst folded in)
    reg  [(LSU_PORTS+1)*NEXT_W-1:0] next_if_pop;     // 25 bits: count + j - 1, empty flag (rst folded in)
    reg  [NEXT_W-1:0]      count_after_no_pop;       // NEXT_W = 5 bits: next count and empty flag if no pop (join count selected)
    reg  [NEXT_W-1:0]      count_after_pop;          // NEXT_W = 5 bits: next count and empty flag if pop (join count selected)
    reg                    count_matches;            // order_count equals count_value_idx
    integer                count_value_idx;
    integer                count_bit_idx;
    reg  [LSU_PORTS-1:0]          rank_used;
    reg  [LSU_PORTS*PORT_W-1:0]   rank_port;
    integer                rank_idx;
    integer                place_rank_idx;
    reg  [LSU_PORTS+1:0]   join_at_least;            // LSU_PORTS+2 = 6 bits: bit m: at least m ports join (bit 0 = 1, top bit = 0)
    reg  [LSU_PORTS:0]     joins_below_at_most;      // LSU_PORTS+1 = 5 bits: bit v: at most rank_idx of the ports 0 .. v-1 join (thermometer)
    integer                below_idx;
    function [ORDER_DEPTH-1:0] rotate_by_onehot;
        input [ORDER_DEPTH-1:0] pointer_onehot;
        input [LSU_PORTS:0]     step_onehot;
        integer                 step_idx;
        begin
            rotate_by_onehot = {ORDER_DEPTH{1'b0}};
            for (step_idx = 0; step_idx <= LSU_PORTS; step_idx = step_idx + 1)
                rotate_by_onehot = rotate_by_onehot | ({ORDER_DEPTH{step_onehot[step_idx]}} & rotate_left(pointer_onehot, step_idx));
        end
    endfunction
    always @* begin
        join_run  = {{LSU_PORTS{1'b0}}, 1'b1};
        for (join_port_idx = 0; join_port_idx < LSU_PORTS; join_port_idx = join_port_idx + 1) begin
            rank_before_onehot[join_port_idx] = join_run;
            if (join_now[join_port_idx])
                join_run = {join_run[LSU_PORTS-1:0], 1'b0};
        end
        // join count thresholds = the full-prefix thresholds from l1d_port_front
        join_at_least = {(LSU_PORTS+2){1'b0}};
        join_at_least[0] = 1'b1;
        for (join_count_idx = 1; join_count_idx <= LSU_PORTS; join_count_idx = join_count_idx + 1)
            join_at_least[join_count_idx] = join_prefix_at_least[(LSU_PORTS-1)*LSU_PORTS + join_count_idx - 1];
        // rank r is used when at least r+1 ports join; its port is the number v of leading ports that hold at most r joins
        // (thermometer joins_below_at_most, encoded by AND-NOT of neighbours; monotone thresholds, no exact count)
        joins_below_at_most = {(LSU_PORTS+1){1'b0}};
        rank_port = {(LSU_PORTS*PORT_W){1'b0}};
        for (rank_idx = 0; rank_idx < LSU_PORTS; rank_idx = rank_idx + 1) begin
            rank_used[rank_idx] = join_at_least[rank_idx + 1];
            joins_below_at_most[0] = 1'b1;
            for (below_idx = 1; below_idx <= LSU_PORTS; below_idx = below_idx + 1)
                joins_below_at_most[below_idx] = !join_prefix_at_least[(below_idx-1)*LSU_PORTS + rank_idx];
            for (below_idx = 0; below_idx < LSU_PORTS; below_idx = below_idx + 1)
                rank_port[rank_idx*PORT_W +: PORT_W] = rank_port[rank_idx*PORT_W +: PORT_W] |
                    ({PORT_W{joins_below_at_most[below_idx] & !joins_below_at_most[below_idx + 1]}} & below_idx);
        end
        for (join_count_idx = 0; join_count_idx <= LSU_PORTS; join_count_idx = join_count_idx + 1)
            join_cnt_onehot[join_count_idx] = join_at_least[join_count_idx] & !join_at_least[join_count_idx + 1];
        // count_next = order_count + join count - pop (ORDER_CNT_W bits, wraps like the adder), reset gives 0 / empty.
        // Every (join count, pop) answer is a table of the count flops (early); join count AND-OR, then pop as the last 2:1 select.
        next_if_no_pop = {((LSU_PORTS+1)*NEXT_W){1'b0}};
        next_if_pop    = {((LSU_PORTS+1)*NEXT_W){1'b0}};
        for (join_count_idx = 0; join_count_idx <= LSU_PORTS; join_count_idx = join_count_idx + 1) begin
            next_if_no_pop[join_count_idx*NEXT_W + ORDER_CNT_W] = rst;
            next_if_pop[join_count_idx*NEXT_W + ORDER_CNT_W]    = rst;
            for (count_value_idx = 0; count_value_idx < COUNT_VALUES; count_value_idx = count_value_idx + 1) begin
                count_matches = (order_count == count_value_idx);
                for (pop_value_idx = 0; pop_value_idx <= 1; pop_value_idx = pop_value_idx + 1) begin
                    count_candidate = count_value_idx + join_count_idx - pop_value_idx;
                    for (count_bit_idx = 0; count_bit_idx < ORDER_CNT_W; count_bit_idx = count_bit_idx + 1)
                        if (count_candidate[count_bit_idx]) begin
                            if (pop_value_idx != 0)
                                next_if_pop[join_count_idx*NEXT_W + count_bit_idx] =
                                    next_if_pop[join_count_idx*NEXT_W + count_bit_idx] | (count_matches & !rst);
                            else
                                next_if_no_pop[join_count_idx*NEXT_W + count_bit_idx] =
                                    next_if_no_pop[join_count_idx*NEXT_W + count_bit_idx] | (count_matches & !rst);
                        end
                    if (count_candidate == {ORDER_CNT_W{1'b0}}) begin
                        if (pop_value_idx != 0)
                            next_if_pop[join_count_idx*NEXT_W + ORDER_CNT_W] =
                                next_if_pop[join_count_idx*NEXT_W + ORDER_CNT_W] | count_matches;
                        else
                            next_if_no_pop[join_count_idx*NEXT_W + ORDER_CNT_W] =
                                next_if_no_pop[join_count_idx*NEXT_W + ORDER_CNT_W] | count_matches;
                    end
                end
            end
        end
        // join_cnt_onehot is exactly one-hot (thresholds), so the AND-OR is a select and rst inside the table holds;
        // the join count selects within each pop value, the pop (decided late in l1d_port_front) is the last 2:1 select
        count_after_no_pop = {NEXT_W{1'b0}};
        count_after_pop    = {NEXT_W{1'b0}};
        for (join_count_idx = 0; join_count_idx <= LSU_PORTS; join_count_idx = join_count_idx + 1) begin
            count_after_no_pop = count_after_no_pop |
                                 ({NEXT_W{join_cnt_onehot[join_count_idx]}} & next_if_no_pop[join_count_idx*NEXT_W +: NEXT_W]);
            count_after_pop    = count_after_pop |
                                 ({NEXT_W{join_cnt_onehot[join_count_idx]}} & next_if_pop[join_count_idx*NEXT_W +: NEXT_W]);
        end
        count_next_sel  = pop_from_head ? count_after_pop[ORDER_CNT_W-1:0] : count_after_no_pop[ORDER_CNT_W-1:0];
        ring_empty_next = pop_from_head ? count_after_pop[ORDER_CNT_W]     : count_after_no_pop[ORDER_CNT_W];
    end
    always @* begin
        rank_1   = {2'b00, join_now[0]};
        rank_2   = {2'b00, join_now[0]} + {2'b00, join_now[1]};
        rank_3   = {2'b00, join_now[0]} + {2'b00, join_now[1]} + {2'b00, join_now[2]};
        join_cnt = rank_3 + {2'b00, join_now[3]};
        place_0  = order_write_onehot;
        place_1  = rotate_by_onehot(order_write_onehot, rank_before_onehot[1]);
        place_2  = rotate_by_onehot(order_write_onehot, rank_before_onehot[2]);
        place_3  = rotate_by_onehot(order_write_onehot, rank_before_onehot[3]);
        place_write_en   = {ORDER_DEPTH{1'b0}};
        place_write_port = {(ORDER_DEPTH*PORT_W){1'b0}};
        for (place_idx = 0; place_idx < ORDER_DEPTH; place_idx = place_idx + 1)
            for (place_rank_idx = 0; place_rank_idx < LSU_PORTS; place_rank_idx = place_rank_idx + 1) begin
                place_write_en[place_idx] = place_write_en[place_idx] |
                    (order_write_onehot[(place_idx + ORDER_DEPTH - place_rank_idx) % ORDER_DEPTH] & rank_used[place_rank_idx]);
                place_write_port[place_idx*PORT_W +: PORT_W] = place_write_port[place_idx*PORT_W +: PORT_W] |
                    ({PORT_W{order_write_onehot[(place_idx + ORDER_DEPTH - place_rank_idx) % ORDER_DEPTH]}} &
                     rank_port[place_rank_idx*PORT_W +: PORT_W]);
            end
    end
    wire [ORDER_CNT_W-1:0] order_count_next;
    assign order_count_next = count_next_sel;
    integer write_place;
    always @(posedge clk) begin : ring_proc
        if (rst) begin
            order_read_onehot  <= {{(ORDER_DEPTH-1){1'b0}}, 1'b1};
            order_write_onehot <= {{(ORDER_DEPTH-1){1'b0}}, 1'b1};
        end else begin
            for (write_place = 0; write_place < ORDER_DEPTH; write_place = write_place + 1)
                if (place_write_en[write_place])
                    order_port[write_place] <= place_write_port[write_place*PORT_W +: PORT_W];
            order_write_onehot <= rotate_by_onehot(order_write_onehot, join_cnt_onehot);
            if (pop_from_head)
                order_read_onehot <= rotate_left(order_read_onehot, 1);
        end
        order_count  <= order_count_next;
        ring_empty_q <= ring_empty_next;
    end
`ifndef SYNTHESIS
    initial if (LSU_PORTS != 4)
        $display("L1D ORDER ERROR bank %0d: LSU_PORTS = %0d, the ports _0 .. _3 need LSU_PORTS = 4", BANK_ID, LSU_PORTS);
    always @(posedge clk) begin
        if (!rst && (order_count + join_cnt > ORDER_DEPTH))
            $display("[%0t] L1D ORDER ERROR bank %0d: ring overflow (%0d waiting + %0d new)", $time, BANK_ID, order_count, join_cnt);
        if (!rst && (pop != pop_from_head))
            $display("[%0t] L1D ORDER ERROR bank %0d: pop input %b differs from the head pop %b", $time, BANK_ID, pop, pop_from_head);
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
