`ifndef L1D_PORT_FRONT_V
`define L1D_PORT_FRONT_V
`default_nettype none
`include "l1d_defines.vh"
module l1d_port_front #(
    `L1D_PARAMS,
    parameter PORT_QUEUE_DEPTH = `L1D_PORT_QUEUE_DEPTH,
    parameter LINE_PORT_ON     = 0
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
    localparam SKID_CNT_W  = $clog2(PORT_QUEUE_DEPTH + 1);
    localparam STOP_MARGIN = 2;
    localparam SKID_POP_LAST = 1;
    localparam SKID_W      = REQ_W + PC_W;
    localparam NB          = NUM_BANKS;
    localparam NP          = LSU_PORTS;
    localparam RANK_W      = 3;
    localparam NA          = (TRK_ALLOC_MAX < NP) ? TRK_ALLOC_MAX : NP;
    input  wire                         clk;
    input  wire                         rst;
    input  wire                         port_request_valid_0;
    input  wire                         port_request_valid_1;
    input  wire                         port_request_valid_2;
    input  wire                         port_request_valid_3;
    input  wire [REQ_W-1:0]             port_request_packet_0;
    input  wire [REQ_W-1:0]             port_request_packet_1;
    input  wire [REQ_W-1:0]             port_request_packet_2;
    input  wire [REQ_W-1:0]             port_request_packet_3;
    input  wire [PC_W-1:0]              port_request_pc_0;
    input  wire [PC_W-1:0]              port_request_pc_1;
    input  wire [PC_W-1:0]              port_request_pc_2;
    input  wire [PC_W-1:0]              port_request_pc_3;
    output reg                          port_stop_0;
    output reg                          port_stop_1;
    output reg                          port_stop_2;
    output reg                          port_stop_3;
    input  wire [LINE_W-1:0]            port_request_line_3;
    input  wire [NB*TRK_CNT_W-1:0]      bank_tracker_free_count;
    input  wire [NB-1:0]                bank_vec_slot_available;
    output wire [NB-1:0]                bank_arrival_request_0;
    output wire [NB-1:0]                bank_arrival_request_1;
    output wire [NB-1:0]                bank_arrival_request_2;
    output wire [NB-1:0]                bank_arrival_request_3;
    output wire [NB-1:0]                bank_drain_request;
    output wire [NB*REQ_W-1:0]          bank_drain_packet;
    output wire [NB*PC_W-1:0]           bank_drain_pc;
    output wire [NB*LINE_W-1:0]         bank_arrival_line_data;
    input  wire [NB-1:0]                bank_arrival_allocated_0;
    input  wire [NB-1:0]                bank_arrival_allocated_1;
    input  wire [NB-1:0]                bank_arrival_allocated_2;
    input  wire [NB-1:0]                bank_arrival_allocated_3;
    input  wire [NB-1:0]                bank_drain_allocated;
    wire [NP-1:0]       port_valid;
    wire [NP*REQ_W-1:0] port_packet;
    wire [NP*PC_W-1:0]  port_pc;
    wire [NP-1:0]       skid_push;
    wire [NP-1:0]       skid_pop;
    wire [NP-1:0]       skid_empty;
    wire [NP-1:0]       skid_full;
    wire [NP*SKID_W-1:0] skid_head;
    wire [NP*SKID_CNT_W-1:0] skid_used;
    wire [LINE_W-1:0]   skid_head_line_3;
    assign port_valid  = {port_request_valid_3, port_request_valid_2, port_request_valid_1, port_request_valid_0};
    assign port_packet = {port_request_packet_3, port_request_packet_2, port_request_packet_1, port_request_packet_0};
    assign port_pc     = {port_request_pc_3, port_request_pc_2, port_request_pc_1, port_request_pc_0};
    genvar sk;
    generate
        for (sk = 0; sk < NP; sk = sk + 1) begin : g_skid
            wire [SKID_W-1:0] skid_in;
            assign skid_in = {port_pc[sk*PC_W +: PC_W], port_packet[sk*REQ_W +: REQ_W]};
            l1d_fifo #(.WIDTH(SKID_W), .DEPTH(PORT_QUEUE_DEPTH), .CNT_W(SKID_CNT_W), .POP_LAST(SKID_POP_LAST)) u_skid (
                .clk (clk), .rst (rst), .push (skid_push[sk]), .push_data (skid_in), .pop (skid_pop[sk]),
                .front_data (skid_head[sk*SKID_W +: SKID_W]), .empty (skid_empty[sk]), .full (skid_full[sk]),
                .used_slots (skid_used[sk*SKID_CNT_W +: SKID_CNT_W]), .free_slots (), .free_slots_next ()
            );
        end
    endgenerate
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
    wire [NP*BANK_W-1:0] pin_bank;
    wire [NP*BANK_W-1:0] head_bank;
    wire [NP-1:0]        pin_line_store;
    wire [NP-1:0]        head_line_store;
    genvar bk_p;
    generate
        for (bk_p = 0; bk_p < NP; bk_p = bk_p + 1) begin : g_bank_of
            assign pin_bank[bk_p*BANK_W +: BANK_W]  = port_packet[bk_p*REQ_W + REQ_ADDR_LSB + BANK_LSB +: BANK_W];
            assign head_bank[bk_p*BANK_W +: BANK_W] = skid_head[bk_p*SKID_W + REQ_ADDR_LSB + BANK_LSB +: BANK_W];
            assign pin_line_store[bk_p]  = port_packet[bk_p*REQ_W + REQ_LINE_BIT] & port_packet[bk_p*REQ_W + REQ_RDWR_BIT];
            assign head_line_store[bk_p] = skid_head[bk_p*SKID_W + REQ_LINE_BIT] & skid_head[bk_p*SKID_W + REQ_RDWR_BIT];
        end
    endgenerate
    wire [NB*NP-1:0] pin_for_bank;
    wire [NB*NP-1:0] arrival_request;
    wire [NB*NP-1:0] alloc_pred;
    wire [NB*NP-1:0] head_port;
    wire [NB-1:0]    ring_empty_q;
    wire [NB-1:0]    drain_alloc_pred;
    // drain decided per port before the ring-head select (head_port_onehot is one-hot or zero, so the head select is the last AND-OR)
    wire [NB*NP-1:0] drain_port_ready; // NB*NP = 16 bits: port p's skid head could be drained into bank b now (head select not applied)
    wire [NB*NP-1:0] drain_alloc_port; // NB*NP = 16 bits: bank b drains port p this cycle (ring head AND ready)
    // number of ones of a constant port pattern (elaboration only: builds the 'at least k lower ports' threshold terms)
    function integer ones_in_pattern;
        input integer pattern_value;
        integer       pattern_bit;
        begin
            ones_in_pattern = 0;
            for (pattern_bit = 0; pattern_bit < NP; pattern_bit = pattern_bit + 1)
                ones_in_pattern = ones_in_pattern + ((pattern_value >> pattern_bit) & 1);
        end
    endfunction
    genvar b;
    generate
        for (b = 0; b < NB; b = b + 1) begin : g_bank
            wire [TRK_CNT_W-1:0] free_count;
            wire                 vec_ok;
            wire [NP-1:0]        eligible;
            wire [NP-1:0]        eligible_if_empty; // NP = 4 bits: eligible without ring_empty_q (counted for the lower ports)
            wire                 free_nonzero;      // at least one tracker entry free
            reg  [NP:1]          free_at_least;
            reg  [NP-1:0]        alloc_here;
            wire [NP-1:0]        join_here;
            wire [NP-1:0]        head_port_onehot_b;
            assign free_count = bank_tracker_free_count[b*TRK_CNT_W +: TRK_CNT_W];
            assign vec_ok     = bank_vec_slot_available[b];
            assign free_nonzero = (free_count != {TRK_CNT_W{1'b0}});
            genvar p;
            for (p = 0; p < NP; p = p + 1) begin : g_port
                assign pin_for_bank[b*NP + p]    = port_valid[p] && (pin_bank[p*BANK_W +: BANK_W] == b);
                assign arrival_request[b*NP + p] = pin_for_bank[b*NP + p] && ring_empty_q[b];
                assign eligible[p]               = arrival_request[b*NP + p] && (!pin_line_store[p] || vec_ok);
                assign eligible_if_empty[p]      = pin_for_bank[b*NP + p] && (!pin_line_store[p] || vec_ok);
                // ready to drain: head present, for this bank, a free entry, and a vec slot for a line store (flops and early pins only)
                assign drain_port_ready[b*NP + p] = !skid_empty[p] && (head_bank[p*BANK_W +: BANK_W] == b) && free_nonzero &&
                                                    (!head_line_store[p] || vec_ok);
                assign alloc_pred[b*NP + p]      = alloc_here[p];
                assign join_here[p]              = pin_for_bank[b*NP + p] && !alloc_here[p];
                assign head_port[b*NP + p]       = head_port_onehot_b[p];
                assign drain_alloc_port[b*NP + p] = head_port[b*NP + p] && drain_port_ready[b*NP + p]; // ring head is the last AND
            end
            // Join thresholds for the ring, straight from the pins and the room rule (not from join_here):
            // in the ports 0 .. k-1, P pins are for this bank, L of them are eligible_if_empty and min(L, G) of those
            // get an entry (G = min(free_count, NA)), so with an empty ring joins = P - min(L, G), else joins = P.
            // joins >= m  <=>  OR over a of (P >= m + a) AND (L < a+1 OR G < a+1): monotone AND/OR, no exact count,
            // and ring_empty_q is the last select.
            reg  [NP*NP-1:0] join_prefix_at_least; // NP*NP = 16 bits: bit (k-1)*NP + (m-1): at least m of the ports 0 .. k-1 join
            reg              pins_at_least;        // at least jp_m + jp_a of the ports 0 .. jp_k-1 have a pin for this bank
            reg              pins_at_least_m;      // at least jp_m of the ports 0 .. jp_k-1 have a pin for this bank
            reg              eligible_at_least;    // at least jp_a + 1 of the ports 0 .. jp_k-1 are eligible_if_empty
            reg              joins_if_empty;       // at least jp_m of the ports 0 .. jp_k-1 join when the ring is empty
            integer          jp_k;
            integer          jp_m;
            integer          jp_a;
            integer          jp_pattern;
            integer rk_p;
            integer rk_k;
            integer rk_pattern;
            reg     lower_at_least; // at least rk_k of the lower ports 0 .. rk_p-1 are eligible_if_empty (monotone AND-OR)
            // port p is allocated when it is eligible, at least one entry is free, and for every k >= 1:
            // (at least k lower ports eligible) implies (at least k+1 entries free, and k < NA).
            // Threshold implications only (no exact-count decode, no XOR). eligible = eligible_if_empty AND ring_empty_q:
            // when the ring is not empty no port is eligible and alloc_here is 0 either way, so the lower ports are
            // counted without ring_empty_q; it enters only through eligible[p] in the last AND.
            always @* begin
                for (rk_k = 1; rk_k <= NP; rk_k = rk_k + 1)
                    free_at_least[rk_k] = (free_count >= rk_k);
                lower_at_least = 1'b0;
                for (rk_p = 0; rk_p < NP; rk_p = rk_p + 1) begin
                    alloc_here[rk_p] = eligible[rk_p] & free_at_least[1] & (NA > 0);
                    for (rk_k = 1; rk_k < NP; rk_k = rk_k + 1)
                        if (rk_k <= rk_p) begin
                            lower_at_least = 1'b0;
                            for (rk_pattern = 0; rk_pattern < (1 << NP); rk_pattern = rk_pattern + 1)
                                if ((rk_pattern >> rk_p) == 0 && ones_in_pattern(rk_pattern) == rk_k)
                                    lower_at_least = lower_at_least | ((eligible_if_empty & rk_pattern) == rk_pattern);
                            alloc_here[rk_p] = alloc_here[rk_p] & (!lower_at_least | (free_at_least[rk_k + 1] & (rk_k < NA)));
                        end
                end
                // join thresholds of every port prefix (see above)
                join_prefix_at_least = {(NP*NP){1'b0}};
                pins_at_least     = 1'b0;
                pins_at_least_m   = 1'b0;
                eligible_at_least = 1'b0;
                joins_if_empty    = 1'b0;
                for (jp_k = 1; jp_k <= NP; jp_k = jp_k + 1)
                    for (jp_m = 1; jp_m <= NP; jp_m = jp_m + 1)
                        if (jp_m <= jp_k) begin
                            pins_at_least_m = 1'b0;
                            for (jp_pattern = 0; jp_pattern < (1 << NP); jp_pattern = jp_pattern + 1)
                                if ((jp_pattern >> jp_k) == 0 && ones_in_pattern(jp_pattern) == jp_m)
                                    pins_at_least_m = pins_at_least_m |
                                        ((pin_for_bank[b*NP +: NP] & jp_pattern) == jp_pattern);
                            joins_if_empty = 1'b0;
                            for (jp_a = 0; jp_a <= jp_k - jp_m; jp_a = jp_a + 1) begin
                                pins_at_least     = 1'b0;
                                eligible_at_least = 1'b0;
                                for (jp_pattern = 0; jp_pattern < (1 << NP); jp_pattern = jp_pattern + 1)
                                    if ((jp_pattern >> jp_k) == 0) begin
                                        if (ones_in_pattern(jp_pattern) == jp_m + jp_a)
                                            pins_at_least = pins_at_least |
                                                ((pin_for_bank[b*NP +: NP] & jp_pattern) == jp_pattern);
                                        if (ones_in_pattern(jp_pattern) == jp_a + 1)
                                            eligible_at_least = eligible_at_least | ((eligible_if_empty & jp_pattern) == jp_pattern);
                                    end
                                joins_if_empty = joins_if_empty | (pins_at_least &
                                    (!eligible_at_least | !(free_at_least[jp_a + 1] & (jp_a + 1 <= NA))));
                            end
                            join_prefix_at_least[(jp_k-1)*NP + jp_m - 1] = ring_empty_q[b] ? joins_if_empty : pins_at_least_m;
                        end
            end
            l1d_bank_order #(`L1D_PARAMS_PASS, .BANK_ID(b), .PORT_QUEUE_DEPTH(PORT_QUEUE_DEPTH)) u_order (
                .clk              (clk),
                .rst              (rst),
                .join_0           (join_here[0]),
                .join_1           (join_here[1]),
                .join_2           (join_here[2]),
                .join_3           (join_here[3]),
                .join_prefix_at_least (join_prefix_at_least),
                .pop              (drain_alloc_pred[b]),
                .head_port_onehot (head_port_onehot_b),
                .ring_empty_q     (ring_empty_q[b])
            );
            reg  [SKID_W-1:0] drain_entry;
            reg               drain_head_ok;
            integer dr_p;
            always @* begin
                drain_entry      = {SKID_W{1'b0}};
                drain_head_ok    = 1'b0;
                for (dr_p = 0; dr_p < NP; dr_p = dr_p + 1) begin
                    drain_entry      = drain_entry | ({SKID_W{head_port_onehot_b[dr_p]}} & skid_head[dr_p*SKID_W +: SKID_W]);
                    drain_head_ok    = drain_head_ok | (head_port_onehot_b[dr_p] && !skid_empty[dr_p] && (head_bank[dr_p*BANK_W +: BANK_W] == b));
                end
            end
            assign bank_drain_request[b] = !ring_empty_q[b] && drain_head_ok;
            // = bank_drain_request && free entry && (!line store of the head || vec_ok): head_port_onehot_b is one-hot or zero
            // and already holds !ring_empty_q, so the per-port ready flags are ORed after the head select
            assign drain_alloc_pred[b]   = |drain_alloc_port[b*NP +: NP];
            assign bank_drain_packet[b*REQ_W +: REQ_W] = drain_entry[0 +: REQ_W];
            assign bank_drain_pc[b*PC_W +: PC_W]       = drain_entry[REQ_W +: PC_W];
            assign bank_arrival_line_data[b*LINE_W +: LINE_W] = ring_empty_q[b] ? port_request_line_3 : skid_head_line_3;
        end
    endgenerate
    genvar out_b;
    generate
        for (out_b = 0; out_b < NB; out_b = out_b + 1) begin : g_out
            assign bank_arrival_request_0[out_b] = arrival_request[out_b*NP + 0];
            assign bank_arrival_request_1[out_b] = arrival_request[out_b*NP + 1];
            assign bank_arrival_request_2[out_b] = arrival_request[out_b*NP + 2];
            assign bank_arrival_request_3[out_b] = arrival_request[out_b*NP + 3];
        end
    endgenerate
    reg  [NP-1:0] port_allocated;
    reg  [NP-1:0] port_drained;
    integer pp;
    integer pb;
    always @* begin
        for (pp = 0; pp < NP; pp = pp + 1) begin
            port_allocated[pp] = 1'b0;
            port_drained[pp]   = 1'b0;
            for (pb = 0; pb < NB; pb = pb + 1) begin
                port_allocated[pp] = port_allocated[pp] | alloc_pred[pb*NP + pp];
                port_drained[pp]   = port_drained[pp]   | drain_alloc_port[pb*NP + pp]; // = drain_alloc_pred & head_port (one-hot head)
            end
        end
    end
    assign skid_push = port_valid & ~port_allocated;
    assign skid_pop  = port_drained;
    function stop_after;
        input [SKID_CNT_W-1:0] used_now;
        input                  push_now;
        input                  pop_now;
        reg   [SKID_CNT_W:0]   used_next;
        reg   [SKID_CNT_W:0]   used_if_push;
        reg   [SKID_CNT_W:0]   used_if_pop;
        reg                    stop_if_hold;
        reg                    stop_if_push;
        reg                    stop_if_pop;
        begin
            used_next    = {1'b0, used_now};
            used_if_push = used_next + {{SKID_CNT_W{1'b0}}, 1'b1};
            used_if_pop  = used_next - {{SKID_CNT_W{1'b0}}, 1'b1};
            stop_if_hold = (used_next    + STOP_MARGIN > PORT_QUEUE_DEPTH);
            stop_if_push = (used_if_push + STOP_MARGIN > PORT_QUEUE_DEPTH);
            stop_if_pop  = (used_if_pop  + STOP_MARGIN > PORT_QUEUE_DEPTH);
            // pop is the last select (no push == pop compare): hold if both or neither
            if (pop_now) stop_after = push_now ? stop_if_hold : stop_if_pop;
            else         stop_after = push_now ? stop_if_push : stop_if_hold;
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
    end
`ifndef SYNTHESIS
    wire [NB*NP-1:0] bank_allocated_flat;
    genvar ck_b;
    generate
        for (ck_b = 0; ck_b < NB; ck_b = ck_b + 1) begin : g_ck
            assign bank_allocated_flat[ck_b*NP +: NP] = {bank_arrival_allocated_3[ck_b], bank_arrival_allocated_2[ck_b],
                                                         bank_arrival_allocated_1[ck_b], bank_arrival_allocated_0[ck_b]};
        end
    endgenerate
    integer ck_p;
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
