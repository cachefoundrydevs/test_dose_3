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
    pop,
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
    input  wire                 pop;
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
    reg  [(LSU_PORTS+1)*NEXT_W-1:0] next_per_join;   // 25 bits: pop picks one of the two above; the join count one-hot selects last
    reg                    count_matches;            // order_count equals count_value_idx
    integer                count_value_idx;
    integer                count_bit_idx;
    reg  [LSU_PORTS-1:0]          rank_used;
    reg  [LSU_PORTS*PORT_W-1:0]   rank_port;
    integer                rank_idx;
    integer                place_rank_idx;
    localparam JOIN_PATTERNS = 1 << LSU_PORTS;
    reg  [LSU_PORTS+1:0]   join_at_least;
    integer                join_pattern_idx;
    reg                    lower_joins_exact;        // exactly rank_idx of the ports below join_port_idx join
    function integer join_pattern_ones;
        input integer pattern_value;
        input integer pattern_width;
        integer       pattern_bit;
        begin
            join_pattern_ones = 0;
            for (pattern_bit = 0; pattern_bit < pattern_width; pattern_bit = pattern_bit + 1)
                join_pattern_ones = join_pattern_ones + ((pattern_value >> pattern_bit) & 1);
        end
    endfunction
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
        // rank r is the port that joins with exactly r joining ports below it: a flat AND-OR of the lower join bits
        // (no serial join_run chain on the order_port write path)
        rank_port = {(LSU_PORTS*PORT_W){1'b0}};
        lower_joins_exact = 1'b0;
        for (join_port_idx = 0; join_port_idx < LSU_PORTS; join_port_idx = join_port_idx + 1)
            for (rank_idx = 0; rank_idx <= join_port_idx; rank_idx = rank_idx + 1) begin
                lower_joins_exact = 1'b0;
                for (join_pattern_idx = 0; join_pattern_idx < JOIN_PATTERNS; join_pattern_idx = join_pattern_idx + 1)
                    if ((join_pattern_idx >> join_port_idx) == 0 && join_pattern_ones(join_pattern_idx, LSU_PORTS) == rank_idx)
                        lower_joins_exact = lower_joins_exact |
                                            ((join_now & ((1 << join_port_idx) - 1)) == join_pattern_idx);
                rank_port[rank_idx*PORT_W +: PORT_W] = rank_port[rank_idx*PORT_W +: PORT_W] |
                    ({PORT_W{join_now[join_port_idx] & lower_joins_exact}} & join_port_idx);
            end
        join_at_least = {(LSU_PORTS+2){1'b0}};
        for (join_pattern_idx = 0; join_pattern_idx < JOIN_PATTERNS; join_pattern_idx = join_pattern_idx + 1)
            for (join_count_idx = 0; join_count_idx <= LSU_PORTS; join_count_idx = join_count_idx + 1)
                if (join_pattern_ones(join_pattern_idx, LSU_PORTS) == join_count_idx)
                    join_at_least[join_count_idx] = join_at_least[join_count_idx] |
                                                    ((join_now & join_pattern_idx) == join_pattern_idx);
        for (join_count_idx = 0; join_count_idx <= LSU_PORTS; join_count_idx = join_count_idx + 1)
            join_cnt_onehot[join_count_idx] = join_at_least[join_count_idx] & !join_at_least[join_count_idx + 1];
        // rank r is used when at least r+1 ports join
        for (rank_idx = 0; rank_idx < LSU_PORTS; rank_idx = rank_idx + 1)
            rank_used[rank_idx] = join_at_least[rank_idx + 1];
        // count_next = order_count + join count - pop (ORDER_CNT_W bits, wraps like the adder), reset gives 0 / empty.
        // Every (join count, pop) answer is a table of the count flops; pop is a 2:1 select, the join count one-hot is the last AND-OR.
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
        next_per_join   = pop ? next_if_pop : next_if_no_pop;
        // join_cnt_onehot is exactly one-hot (thresholds), so the AND-OR is a select and rst inside the table holds
        count_next_sel  = {ORDER_CNT_W{1'b0}};
        ring_empty_next = 1'b0;
        for (join_count_idx = 0; join_count_idx <= LSU_PORTS; join_count_idx = join_count_idx + 1) begin
            count_next_sel  = count_next_sel |
                              ({ORDER_CNT_W{join_cnt_onehot[join_count_idx]}} & next_per_join[join_count_idx*NEXT_W +: ORDER_CNT_W]);
            ring_empty_next = ring_empty_next | (join_cnt_onehot[join_count_idx] & next_per_join[join_count_idx*NEXT_W + ORDER_CNT_W]);
        end
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
            if (pop)
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
