`ifndef L1D_ECC_SYN_V
`define L1D_ECC_SYN_V
`default_nettype none
module l1d_ecc_syn #(
    parameter DATA_W = 64,
    parameter HAM_W  = 7
) (
    input  wire [DATA_W-1:0] data,
    input  wire [HAM_W:0]    check,
    output wire [HAM_W-1:0]  syndrome,
    output wire              parity_odd,
    output wire              correctable,
    output wire              uncorrectable
);
    wire [HAM_W:0] recomputed;
    l1d_ecc_enc #(.DATA_W(DATA_W), .HAM_W(HAM_W)) u_enc (
        .data  (data),
        .check (recomputed)
    );
    wire syndrome_nonzero;
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
    function [DATA_W-1:0] ham_mask;
        input integer ham_bit;
        integer d;
        begin
            for (d = 0; d < DATA_W; d = d + 1)
                ham_mask[d] = ((data_pos(d) >> ham_bit) & 1) ? 1'b1 : 1'b0;
        end
    endfunction
    genvar sb;
    generate
        for (sb = 0; sb < HAM_W; sb = sb + 1) begin : g_synd
            localparam [DATA_W-1:0] SMASK = ham_mask(sb);
            assign syndrome[sb] = ^{(data & SMASK), check[sb]};
        end
    endgenerate
    assign syndrome_nonzero = |syndrome;
    assign parity_odd    = ^{data, check};
    assign correctable   = parity_odd;
    assign uncorrectable = syndrome_nonzero & ~parity_odd;
endmodule
`default_nettype wire
`endif
