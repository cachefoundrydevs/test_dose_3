// ==========================================================================
// l1d_pf_sms_fifo.v - small lossy FIFO for the SMS front end
// ==========================================================================
// IN SIMPLE WORDS : A short queue (a few pigeon holes). If it is full a
//   new record is thrown away: the SMS prefetcher only learns from the
//   records, so a lost record costs a little accuracy, never correctness
//   and never back-pressure on the cache (README sections 6 and 9).
// STAGE / CYCLE   : push is registered; the head is read through a 4:1
//   mux straight from the flops (combinational output, V-3.11, 2 levels).
// INSTANTIATED BY : l1d_pf_sms_front, u_train_fifo (62 bits) and
//   u_evict_fifo (50 bits): 2 copies of this one module.
//
// PARAMETERS:
//   name   default  unit     meaning
//   WIDTH  62       bits     width of one record
//   DEPTH  4        records  FIFO depth (must be a power of two)
//
// PORTS (default widths):
//   clk, rst     in  1       clock, synchronous reset (high)
//   push_valid   in  1       a record to store this cycle
//   push_data    in  62      the record
//   pop_en       in  1       take the head record this cycle
//   head_valid   out 1       a record is waiting (combinational)
//   head_data    out 62      the oldest record (combinational)
//
// WORKED EXAMPLE : DEPTH 4, five pushes without a pop: the first four
//   are stored, the fifth is dropped; pops return records 1 to 4.
//
// ASSUMPTIONS (V-9.2): A1 DEPTH is a power of two (pointers wrap by
//   overflow). A2 a push into a full FIFO is dropped even if a pop
//   happens in the same cycle (simple, lossy by design).
// ==========================================================================
`ifndef L1D_PF_SMS_FIFO_V
`define L1D_PF_SMS_FIFO_V
`default_nettype none

module l1d_pf_sms_fifo #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter WIDTH = 62, // default 62 bits: one record
    parameter DEPTH = 4   // default 4 records (power of two)
) (
    // ---- clock and reset ----
    input  wire             clk,         // clock, rising edge
    input  wire             rst,         // synchronous reset, high
    // ---- push side ----
    input  wire             push_valid,  // 1 = store a record
    input  wire [WIDTH-1:0] push_data,   // the record
    // ---- pop side ----
    input  wire             pop_en,      // 1 = take the head
    output wire             head_valid,  // 1 = head exists, combinational
    output wire [WIDTH-1:0] head_data    // oldest record, combinational
);

    // ---- local constants (value at the default) ----
    localparam PTR_W = $clog2(DEPTH);            // 2 bits pointer
    localparam CNT_W = $clog2(DEPTH + 1);        // 3 bits count 0..4
    localparam [CNT_W-1:0] DEPTH_V = DEPTH;      // 4, as a count value

    reg  [DEPTH*WIDTH-1:0] mem_q;     // 4 x 62: record storage
    reg  [PTR_W-1:0]       wr_ptr_q;  // 2: next slot to write
    reg  [PTR_W-1:0]       rd_ptr_q;  // 2: oldest slot
    reg  [CNT_W-1:0]       count_q;   // 3: records held
    wire                   full;      // 1: no room
    wire                   push_ok;   // 1: push is stored
    wire                   pop_ok;    // 1: pop takes a record

    assign full       = (count_q == DEPTH_V);
    assign push_ok    = push_valid & ~full;
    assign head_valid = (count_q != {CNT_W{1'b0}});
    assign pop_ok     = pop_en & head_valid;
    assign head_data  = mem_q[rd_ptr_q*WIDTH +: WIDTH];

    always @(posedge clk) begin : fifo_proc
        if (rst) begin
            wr_ptr_q <= {PTR_W{1'b0}};
            rd_ptr_q <= {PTR_W{1'b0}};
            count_q  <= {CNT_W{1'b0}};
        end
        else begin
            if (push_ok) begin
                mem_q[wr_ptr_q*WIDTH +: WIDTH] <= push_data;
                wr_ptr_q <= wr_ptr_q + 1'b1;
            end
            if (pop_ok) begin
                rd_ptr_q <= rd_ptr_q + 1'b1;
            end
            if (push_ok && !pop_ok) begin
                count_q <= count_q + 1'b1;
            end
            else if (!push_ok && pop_ok) begin
                count_q <= count_q - 1'b1;
            end // else if (pop only)
        end // else (run)
    end // always fifo_proc

endmodule

`default_nettype wire
`endif
