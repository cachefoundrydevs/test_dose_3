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
    parameter WRITE_ONEHOT = 0,
    parameter PLRU_ONEHOT  = 0,
    parameter WRITE_BEHIND = 0
) (
    input  wire               clk,
    input  wire               rst,
    input  wire [INDEX_W-1:0] read_set,
    output wire [2*WAYS-1:0]  read_way_states_flat,
    output wire [WAYS-1:0]    read_way_valid,
    output wire [NB-1:0]      read_plru_bits,
    output wire [WAYS-1:0]    read_pf_bits,
    output wire [WAYS-1:0]    read_inv_first,
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
    input  wire [WAYS-1:0]    plru_write_way_enable,
    input  wire [WAYS*NB-1:0] plru_write_way_value,
    input  wire               pf_write_enable,
    input  wire [INDEX_W-1:0] pf_write_set,
    input  wire [WAY_W-1:0]   pf_write_way,
    input  wire               pf_write_value,
    input  wire [WAYS-1:0]    pf_write_way_enable
);
    reg [2*WAYS-1:0] states [0:SETS-1];
    reg [NB-1:0]     plru   [0:SETS-1];
    reg [WAYS-1:0]   pf_bit [0:SETS-1];
    reg [2*WAYS-1:0]  states_held [0:SETS-1];
    reg [NB-1:0]      plru_held   [0:SETS-1];
    reg [WAYS-1:0]    pf_held     [0:SETS-1];
    reg [INDEX_W-1:0] pend_state_set;
    reg [WAYS-1:0]    pend_state_en;
    reg [2*WAYS-1:0]  pend_state_val;
    reg [INDEX_W-1:0] pend_plru_set;
    reg [WAYS-1:0]    pend_plru_en;
    reg [WAYS*NB-1:0] pend_plru_cand;
    reg [INDEX_W-1:0] pend_pf_set;
    reg [WAYS-1:0]    pend_pf_en;
    reg               pend_pf_val;
    reg [NB-1:0]      pend_plru_pick;
    integer           es;
    integer           ew;
    integer s;
    integer ww;
    wire [SETS-1:0] read_hot;
    wire [SETS-1:0] state_write_hot;
    wire [SETS-1:0] plru_write_hot;
    wire [SETS-1:0] pf_write_hot;
    wire [SETS-1:0] pend_state_hot;
    wire [SETS-1:0] pend_plru_hot;
    wire [SETS-1:0] pend_pf_hot;
    genvar hs;
    generate
        for (hs = 0; hs < SETS; hs = hs + 1) begin : g_set_hot
            assign read_hot[hs]        = (read_set == hs);
            assign state_write_hot[hs] = (state_write_set == hs);
            assign plru_write_hot[hs]  = (plru_write_set == hs);
            assign pf_write_hot[hs]    = (pf_write_set == hs);
            assign pend_state_hot[hs]  = (pend_state_set == hs);
            assign pend_plru_hot[hs]   = (pend_plru_set == hs);
            assign pend_pf_hot[hs]     = (pend_pf_set == hs);
        end
    endgenerate
    function [WAYS-1:0] inv_first_of;
        input [WAYS-1:0] valid_bits;
        integer fw;
        reg     found;
        begin
            inv_first_of = {WAYS{1'b0}};
            found        = 1'b0;
            for (fw = 0; fw < WAYS; fw = fw + 1) begin
                inv_first_of[fw] = !valid_bits[fw] && !found;
                found            = found || !valid_bits[fw];
            end
        end
    endfunction
    generate
    if (WRITE_BEHIND == 0) begin : g_write_direct
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
                if (PLRU_ONEHOT != 0) begin
                    for (ww = 0; ww < WAYS; ww = ww + 1)
                        if (plru_write_hot[s] && plru_write_way_enable[ww])
                            plru[s] <= plru_write_way_value[ww*NB +: NB];
                end else if (plru_write_enable && plru_write_hot[s])
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
    end else begin : g_write_behind
    always @* begin : pend_plru_proc
        pend_plru_pick = {NB{1'b0}};
        for (ew = 0; ew < WAYS; ew = ew + 1)
            if (pend_plru_en[ew])
                pend_plru_pick = pend_plru_cand[ew*NB +: NB];
    end
    always @* begin : effective_proc
        for (es = 0; es < SETS; es = es + 1) begin
            states[es] = states_held[es];
            plru[es]   = plru_held[es];
            pf_bit[es] = pf_held[es];
            for (ew = 0; ew < WAYS; ew = ew + 1) begin
                if (pend_state_hot[es] && pend_state_en[ew])
                    states[es][2*ew +: 2] = pend_state_val[2*ew +: 2];
                if (pend_pf_hot[es] && pend_pf_en[ew])
                    pf_bit[es][ew] = pend_pf_val;
            end
            if (pend_plru_hot[es] && (|pend_plru_en))
                plru[es] = pend_plru_pick;
        end
    end
    always @(posedge clk) begin : meta_behind_proc
        if (rst) begin
            for (s = 0; s < SETS; s = s + 1) begin
                states_held[s] <= {2*WAYS{1'b0}};
                plru_held[s]   <= {NB{1'b0}};
                pf_held[s]     <= {WAYS{1'b0}};
            end
            pend_state_en <= {WAYS{1'b0}};
            pend_plru_en  <= {WAYS{1'b0}};
            pend_pf_en    <= {WAYS{1'b0}};
        end else begin
            for (s = 0; s < SETS; s = s + 1) begin
                states_held[s] <= states[s];
                plru_held[s]   <= plru[s];
                pf_held[s]     <= pf_bit[s];
            end
            pend_state_set <= state_write_set;
            pend_state_en  <= state_write_way_enable;
            pend_state_val <= state_write_way_value;
            pend_plru_set  <= plru_write_set;
            pend_plru_en   <= plru_write_way_enable;
            pend_plru_cand <= plru_write_way_value;
            pend_pf_set    <= pf_write_set;
            pend_pf_en     <= pf_write_way_enable;
            pend_pf_val    <= pf_write_value;
        end
    end
    end
    endgenerate
    generate
        if (WRITE_BEHIND == 1) begin : g_read_behind
            reg [2*WAYS-1:0] held_states_or;
            reg [WAYS-1:0]   held_valid_or;
            reg [NB-1:0]     held_plru_or;
            reg [WAYS-1:0]   held_pf_or;
            integer hr;
            integer hv;
            always @* begin
                held_states_or = {2*WAYS{1'b0}};
                held_valid_or  = {WAYS{1'b0}};
                held_plru_or   = {NB{1'b0}};
                held_pf_or     = {WAYS{1'b0}};
                for (hr = 0; hr < SETS; hr = hr + 1) begin
                    held_states_or = held_states_or | ({2*WAYS{read_hot[hr]}} & states_held[hr]);
                    held_plru_or   = held_plru_or   | ({NB{read_hot[hr]}}     & plru_held[hr]);
                    held_pf_or     = held_pf_or     | ({WAYS{read_hot[hr]}}   & pf_held[hr]);
                    for (hv = 0; hv < WAYS; hv = hv + 1)
                        held_valid_or[hv] = held_valid_or[hv] | (read_hot[hr] & (|states_held[hr][2*hv +: 2]));
                end
            end
            wire pend_state_here;
            wire pend_plru_here;
            wire pend_pf_here;
            assign pend_state_here = (pend_state_set == read_set);
            assign pend_plru_here  = (pend_plru_set == read_set) && (|pend_plru_en);
            assign pend_pf_here    = (pend_pf_set == read_set);
            genvar fv;
            for (fv = 0; fv < WAYS; fv = fv + 1) begin : g_forward
                assign read_way_states_flat[2*fv +: 2] = (pend_state_here && pend_state_en[fv]) ? pend_state_val[2*fv +: 2] :
                                                                                                  held_states_or[2*fv +: 2];
                assign read_way_valid[fv]  = (pend_state_here && pend_state_en[fv]) ? (|pend_state_val[2*fv +: 2]) :
                                                                                      held_valid_or[fv];
                assign read_pf_bits[fv]    = (pend_pf_here && pend_pf_en[fv]) ? pend_pf_val : held_pf_or[fv];
            end
            assign read_plru_bits   = pend_plru_here ? pend_plru_pick : held_plru_or;
            assign read_inv_first   = inv_first_of(read_way_valid);
            assign read_any_invalid = !(&read_way_valid);
        end else if (READ_ONEHOT != 0) begin : g_read_onehot
            reg [2*WAYS-1:0] read_states_or;
            reg [WAYS-1:0]   read_valid_or;
            reg [NB-1:0]     read_plru_or;
            reg [WAYS-1:0]   read_pf_or;
            reg [WAYS-1:0]   read_inv_first_or;
            reg              read_any_invalid_or;
            reg [WAYS-1:0]   set_valid_bits;
            integer rs;
            integer rw;
            always @* begin
                read_states_or = {2*WAYS{1'b0}};
                read_valid_or  = {WAYS{1'b0}};
                read_plru_or   = {NB{1'b0}};
                read_pf_or     = {WAYS{1'b0}};
                read_inv_first_or   = {WAYS{1'b0}};
                read_any_invalid_or = 1'b0;
                set_valid_bits      = {WAYS{1'b0}};
                for (rs = 0; rs < SETS; rs = rs + 1) begin
                    for (rw = 0; rw < WAYS; rw = rw + 1)
                        set_valid_bits[rw] = |states[rs][2*rw +: 2];
                    read_inv_first_or   = read_inv_first_or   | ({WAYS{read_hot[rs]}} & inv_first_of(set_valid_bits));
                    read_any_invalid_or = read_any_invalid_or | (read_hot[rs] & !(&set_valid_bits));
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
            assign read_inv_first       = read_inv_first_or;
            assign read_any_invalid     = read_any_invalid_or;
        end else begin : g_read_binary
            genvar vw;
            assign read_way_states_flat = states[read_set];
            assign read_plru_bits       = plru[read_set];
            assign read_pf_bits         = pf_bit[read_set];
            for (vw = 0; vw < WAYS; vw = vw + 1) begin : g_valid
                assign read_way_valid[vw] = |read_way_states_flat[2*vw +: 2];
            end
            assign read_inv_first   = inv_first_of(read_way_valid);
            assign read_any_invalid = !(&read_way_valid);
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
