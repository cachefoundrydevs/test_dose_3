`ifndef L1D_ECC_FIX_V
`define L1D_ECC_FIX_V
`default_nettype none
module l1d_ecc_fix #(
    parameter DATA_W = 64,
    parameter HAM_W  = 7
) (
    input  wire [DATA_W-1:0] data,
    input  wire [HAM_W-1:0]  syndrome,
    input  wire              fix_enable,
    output wire [DATA_W-1:0] fixed
);
    function integer data_pos;
        input integer bit_num;
        integer pos;
        integer seen;
        integer is_pow2;
        integer k;
        begin
            data_pos = 0;
            seen     = 0;
            for (pos = 1; pos <= DATA_W + HAM_W; pos = pos + 1) begin
                is_pow2 = 0;
                for (k = 0; k <= HAM_W; k = k + 1)
                    if (pos == (1 << k)) is_pow2 = 1;
                if (!is_pow2) begin
                    if (seen == bit_num) data_pos = pos;
                    seen = seen + 1;
                end
            end
        end
    endfunction
    localparam LO_W = HAM_W / 2;
    wire [(1 << LO_W)-1:0]           lo_hot;
    wire [(1 << (HAM_W - LO_W))-1:0] hi_hot;
    genvar lv, hv;
    generate
        for (lv = 0; lv < (1 << LO_W); lv = lv + 1) begin : g_lo
            assign lo_hot[lv] = (syndrome[LO_W-1:0] == lv);
        end
        for (hv = 0; hv < (1 << (HAM_W - LO_W)); hv = hv + 1) begin : g_hi
            assign hi_hot[hv] = (syndrome[HAM_W-1:LO_W] == hv);
        end
    endgenerate
    wire [DATA_W-1:0] flip;
    genvar d;
    generate
        for (d = 0; d < DATA_W; d = d + 1) begin : g_flip
            localparam [HAM_W-1:0] POS = data_pos(d);
            assign flip[d] = fix_enable & lo_hot[POS[LO_W-1:0]] & hi_hot[POS[HAM_W-1:LO_W]];
        end
    endgenerate
    assign fixed = data ^ flip;
endmodule
`default_nettype wire
`endif
