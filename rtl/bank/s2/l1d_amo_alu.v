// ============================================================================
// l1d_amo_alu.v  -  the AMO unit (Stage 2 of the bank): new = f(old, operand)
// ============================================================================
// cache_v2_bank_pipe (2026-11-03, user decision: an AMO is ONE request computed inside the cache: the line is read at the
// grant and available in S1, this unit computes the new value in S2, the bank writes the merged line in S3).
//
// IN SIMPLE WORDS
//   Given the value that was in memory (old) and the value the CPU sent (operand), produce the value to write back.
//   32-bit AMOs (size 2) use the low 32 bits only (MIN / MAX compare them as signed 32-bit numbers); 64-bit AMOs
//   (size 3) use all 64. The bits above a 32-bit result are don't-care: only 4 bytes are written.
//
// OPERATIONS (amo_op, l1d_encodings.vh)
//   AMO_SWAP operand   AMO_ADD old+operand   AMO_XOR / AMO_AND / AMO_OR bitwise
//   AMO_MIN / AMO_MAX signed    AMO_MINU / AMO_MAXU unsigned       any other code: SWAP
//
// PORTS
//   amo_op        which operation (4 bits)
//   access_size   0..3 = 1/2/4/8 bytes (AMOs use 2 or 3; 0 / 1 behave as 32-bit)
//   old_value     the old value, aligned to bit 0 (l1d_resp_gen output: zero-extended)
//   operand       the CPU's operand (store data, low bits)
//   new_value     the value to write (aligned to bit 0)
// ============================================================================

`default_nettype none

module l1d_amo_alu #(
    parameter W             = 64,                         // W = 64 bits
    parameter MEM_SIZE_BITS = 2                           // MEM_SIZE_BITS = 2 bits
) (
    input  wire [3:0]               amo_op,
    input  wire [MEM_SIZE_BITS-1:0] access_size,
    input  wire [W-1:0]             old_value,
    input  wire [W-1:0]             operand,
    output reg  [W-1:0]             new_value
);
    `include "l1d_encodings.vh"

    localparam HW = W / 2;                                // HW = 32 bits
    wire is64 = (access_size == 2'd3);

    // compare in the width of the access: a 32-bit access sign-extends (signed) or zero-extends (unsigned) its low half
    wire [W-1:0] a_s = is64 ? old_value : {{HW{old_value[HW-1]}}, old_value[HW-1:0]};
    wire [W-1:0] b_s = is64 ? operand   : {{HW{operand[HW-1]}},   operand[HW-1:0]};
    wire [W-1:0] a_u = is64 ? old_value : {{HW{1'b0}}, old_value[HW-1:0]};
    wire [W-1:0] b_u = is64 ? operand   : {{HW{1'b0}}, operand[HW-1:0]};
    wire         lt_s = $signed(a_s) < $signed(b_s);       // old < operand, signed
    wire         lt_u = a_u < b_u;                         // old < operand, unsigned

    always @* begin
        case (amo_op)
            AMO_ADD:  new_value = old_value + operand;
            AMO_XOR:  new_value = old_value ^ operand;
            AMO_AND:  new_value = old_value & operand;
            AMO_OR:   new_value = old_value | operand;
            AMO_MIN:  new_value = lt_s ? old_value : operand;
            AMO_MAX:  new_value = lt_s ? operand : old_value;
            AMO_MINU: new_value = lt_u ? old_value : operand;
            AMO_MAXU: new_value = lt_u ? operand : old_value;
            default:  new_value = operand;                 // AMO_SWAP
        endcase
    end
endmodule

`default_nettype wire
