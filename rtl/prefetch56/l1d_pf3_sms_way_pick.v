`ifndef L1D_PF3_SMS_WAY_PICK_V
`define L1D_PF3_SMS_WAY_PICK_V
`default_nettype none
(* keep_hierarchy *)
module l1d_pf3_sms_way_pick #(
    parameter WAYS = 4
) (
    input  wire                    set_go,
    input  wire [WAYS-1:0]         tag_match,
    input  wire [WAYS-1:0]         way_valid,
    input  wire [$clog2(WAYS)-1:0] ring_way,
    output wire [WAYS-1:0]         way_write,
    output wire                    ring_adv
);
    reg  [WAYS-1:0] free_first;
    reg  [WAYS-1:0] ring_oh;
    reg             free_seen;
    wire            any_free;
    wire            any_match;
    wire [WAYS-1:0] victim;
    always @* begin : free_proc
        integer way_idx;
        free_first = {WAYS{1'b0}};
        free_seen  = 1'b0;
        for (way_idx = 0; way_idx < WAYS; way_idx = way_idx + 1) begin
            free_first[way_idx] = ~way_valid[way_idx] & ~free_seen;
            free_seen           = free_seen | ~way_valid[way_idx];
        end
        ring_oh = {WAYS{1'b0}};
        ring_oh[ring_way] = 1'b1;
    end
    assign any_free  = ~(&way_valid);
    assign any_match = |tag_match;
    assign victim    = any_free ? free_first : ring_oh;
    assign way_write = (tag_match | (victim & {WAYS{~any_match}}))
                       & {WAYS{set_go}};
    assign ring_adv  = set_go & ~any_match & ~any_free;
endmodule
`default_nettype wire
`endif
