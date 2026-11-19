`ifndef L1D_META_FLOPS_V
`define L1D_META_FLOPS_V
`default_nettype none
`include "l1d_defines.vh"
module l1d_meta_flops #(
    parameter SETS    = 64,
    parameter WAYS    = 4,
    parameter INDEX_W = $clog2(SETS),
    parameter WAY_W   = $clog2(WAYS),
    parameter NB      = WAYS - 1,
    parameter READ_ONEHOT  = 0,
    parameter WRITE_ONEHOT = 0
) (
    input  wire               clk,
    input  wire               rst,
    input  wire [INDEX_W-1:0] read_set,
    output wire [2*WAYS-1:0]  read_way_states_flat,
    output wire [WAYS-1:0]    read_way_valid,
    output wire [NB-1:0]      read_plru_bits,
    output wire [WAYS-1:0]    read_pf_bits,
    output wire [WAYS-1:0]    read_victim_hot,
    output wire               read_any_invalid,
    input  wire               state_write_enable,
    input  wire [INDEX_W-1:0] state_write_set,
    input  wire [WAY_W-1:0]   state_write_way,
    input  wire [1:0]         state_write_value,
    input  wire [WAYS-1:0]    state_write_way_enable,
    input  wire [2*WAYS-1:0]  state_write_way_value,
    input  wire               plru_write_enable,
    input  wire [INDEX_W-1:0] plru_write_set,
    input  wire [NB-1:0]      plru_write_value,
    input  wire               pf_write_enable,
    input  wire [INDEX_W-1:0] pf_write_set,
    input  wire [WAY_W-1:0]   pf_write_way,
    input  wire               pf_write_value,
    input  wire [WAYS-1:0]    pf_write_way_enable
);
    reg [2*WAYS-1:0] states [0:SETS-1];
    reg [NB-1:0]     plru   [0:SETS-1];
    reg [WAYS-1:0]   pf_bit [0:SETS-1];
    integer s;
    integer ww;
    wire [SETS-1:0] read_hot;
    wire [SETS-1:0] state_write_hot;
    wire [SETS-1:0] plru_write_hot;
    wire [SETS-1:0] pf_write_hot;
    genvar hs;
    generate
        for (hs = 0; hs < SETS; hs = hs + 1) begin : g_set_hot
            assign read_hot[hs]        = (read_set == hs);
            assign state_write_hot[hs] = (state_write_set == hs);
            assign plru_write_hot[hs]  = (plru_write_set == hs);
            assign pf_write_hot[hs]    = (pf_write_set == hs);
        end
    endgenerate
    function [WAYS-1:0] victim_hot_of;
        input [WAYS-1:0]  valid_bits;
        input [WAY_W-1:0] plru_way;
        integer fw;
        reg     found;
        begin
            victim_hot_of = {WAYS{1'b0}};
            found         = 1'b0;
            for (fw = 0; fw < WAYS; fw = fw + 1) begin
                if (!valid_bits[fw] && !found)
                    victim_hot_of[fw] = 1'b1;
                if (!valid_bits[fw])
                    found = 1'b1;
            end
            for (fw = 0; fw < WAYS; fw = fw + 1)
                if (!found && (plru_way == fw))
                    victim_hot_of[fw] = 1'b1;
        end
    endfunction
    always @(posedge clk) begin : meta_proc
        if (rst) begin
            for (s = 0; s < SETS; s = s + 1) begin
                states[s] <= {2*WAYS{1'b0}};
                plru[s]   <= {NB{1'b0}};
                pf_bit[s] <= {WAYS{1'b0}};
            end
        end else if (WRITE_ONEHOT != 0) begin
            for (s = 0; s < SETS; s = s + 1) begin
                for (ww = 0; ww < WAYS; ww = ww + 1) begin
                    if (state_write_hot[s] && state_write_way_enable[ww])
                        states[s][2*ww +: 2] <= state_write_way_value[2*ww +: 2];
                    if (pf_write_hot[s] && pf_write_way_enable[ww])
                        pf_bit[s][ww] <= pf_write_value;
                end
                if (plru_write_enable && plru_write_hot[s])
                    plru[s] <= plru_write_value;
            end
        end else begin
            if (state_write_enable)
                states[state_write_set][2*state_write_way +: 2] <= state_write_value;
            if (plru_write_enable)
                plru[plru_write_set] <= plru_write_value;
            if (pf_write_enable)
                pf_bit[pf_write_set][pf_write_way] <= pf_write_value;
        end
    end
    generate
        if (READ_ONEHOT != 0) begin : g_read_onehot
            wire [WAYS*SETS-1:0] set_victim_hot;
            wire [SETS-1:0]      set_any_invalid;
            genvar vs, iw;
            for (vs = 0; vs < SETS; vs = vs + 1) begin : g_set_victim
                wire [WAYS-1:0]  set_valid;
                wire [WAY_W-1:0] set_plru_way;
                wire [NB-1:0]    set_plru_unused;
                for (iw = 0; iw < WAYS; iw = iw + 1) begin : g_valid_way
                    assign set_valid[iw] = |states[vs][2*iw +: 2];
                end
                l1d_plru #(.WAYS(WAYS), .WAY_W(WAY_W), .NB(NB)) u_plru_victim (
                    .plru_bits      (plru[vs]),
                    .accessed_way   ({WAY_W{1'b0}}),
                    .victim_way     (set_plru_way),
                    .plru_bits_next (set_plru_unused)
                );
                assign set_victim_hot[vs*WAYS +: WAYS] = victim_hot_of(set_valid, set_plru_way);
                assign set_any_invalid[vs]             = !(&set_valid);
            end
            reg [2*WAYS-1:0] read_states_or;
            reg [WAYS-1:0]   read_valid_or;
            reg [NB-1:0]     read_plru_or;
            reg [WAYS-1:0]   read_pf_or;
            reg [WAYS-1:0]   read_victim_or;
            reg              read_any_invalid_or;
            integer rs;
            integer rw;
            always @* begin
                read_states_or = {2*WAYS{1'b0}};
                read_valid_or  = {WAYS{1'b0}};
                read_plru_or   = {NB{1'b0}};
                read_pf_or     = {WAYS{1'b0}};
                read_victim_or = {WAYS{1'b0}};
                read_any_invalid_or = 1'b0;
                for (rs = 0; rs < SETS; rs = rs + 1) begin
                    read_victim_or      = read_victim_or      | ({WAYS{read_hot[rs]}} & set_victim_hot[rs*WAYS +: WAYS]);
                    read_any_invalid_or = read_any_invalid_or | (read_hot[rs] & set_any_invalid[rs]);
                    read_states_or = read_states_or | ({2*WAYS{read_hot[rs]}} & states[rs]);
                    read_plru_or   = read_plru_or   | ({NB{read_hot[rs]}}     & plru[rs]);
                    read_pf_or     = read_pf_or     | ({WAYS{read_hot[rs]}}   & pf_bit[rs]);
                    for (rw = 0; rw < WAYS; rw = rw + 1)
                        read_valid_or[rw] = read_valid_or[rw] | (read_hot[rs] & (|states[rs][2*rw +: 2]));
                end
            end
            assign read_way_states_flat = read_states_or;
            assign read_way_valid       = read_valid_or;
            assign read_plru_bits       = read_plru_or;
            assign read_pf_bits         = read_pf_or;
            assign read_victim_hot      = read_victim_or;
            assign read_any_invalid     = read_any_invalid_or;
        end else begin : g_read_binary
            genvar vw;
            wire [WAY_W-1:0] read_plru_way;
            wire [NB-1:0]    read_plru_unused;
            assign read_way_states_flat = states[read_set];
            assign read_plru_bits       = plru[read_set];
            assign read_pf_bits         = pf_bit[read_set];
            l1d_plru #(.WAYS(WAYS), .WAY_W(WAY_W), .NB(NB)) u_plru_victim (
                .plru_bits      (read_plru_bits),
                .accessed_way   ({WAY_W{1'b0}}),
                .victim_way     (read_plru_way),
                .plru_bits_next (read_plru_unused)
            );
            assign read_victim_hot      = victim_hot_of(read_way_valid, read_plru_way);
            assign read_any_invalid     = !(&read_way_valid);
            for (vw = 0; vw < WAYS; vw = vw + 1) begin : g_valid
                assign read_way_valid[vw] = |read_way_states_flat[2*vw +: 2];
            end
        end
    endgenerate
`ifdef L1D_DEBUG
    always @(posedge clk) begin
        if (!rst && state_write_enable && states[state_write_set][2*state_write_way +: 2] != state_write_value)
            $display("[%0t] L1D META %m set=%0d way=%0d state %0d -> %0d",
                     $time, state_write_set, state_write_way, states[state_write_set][2*state_write_way +: 2], state_write_value);
        if (!rst && pf_write_enable && pf_bit[pf_write_set][pf_write_way] != pf_write_value)
            $display("[%0t] L1D META %m set=%0d way=%0d pf_bit -> %0d", $time, pf_write_set, pf_write_way, pf_write_value);
    end
`endif
endmodule
`default_nettype wire
`endif
