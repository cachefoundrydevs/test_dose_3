// ==========================================================================
// l1d_pf3_fifo.v - small FIFO with an early data write and a late push
// ==========================================================================
// IN SIMPLE WORDS : a short queue of records (the SMS training / eviction
//   FIFO of the 3-cycle prefetch system). The next free slot (just past
//   the tail) is written with stage_data in every cycle the queue is not
//   full, whatever the record is; the record only becomes part of the
//   queue when push_valid says so (known late: "the core did not take it
//   directly"). A written but not pushed record is simply overwritten in
//   the next cycle, because the slot is past the tail and nobody reads it.
//   So no decision at all drives the data flops: their write enable is
//   the pointer decode and "not full", both from flops (rule T-6).
//   Same queue behaviour as l1d_pf_sms_fifo (which the 7-stage reference
//   still uses).
//
// STAGE / CYCLE   : the queue is written at the end of S1 and read
//   (head) combinationally in S1.
// INSTANTIATED BY : l1d_pf3_sms_core (u_train_fifo, u_evict_fifo).
//
// PARAMETERS:
//   name   default unit     meaning
//   WIDTH  79      bits     one record
//   DEPTH  4       records  queue depth (power of two)
//
// PORTS:
//   clk, rst     in  1      clock, synchronous reset (high)
//   stage_data   in  WIDTH  the record (written into the free slot)
//   push_valid   in  1      this cycle's stage_data joins the queue
//   pop_en       in  1      take the head
//   head_valid   out 1      a record is in the queue
//   head_data    out WIDTH  the oldest record
//
// WORKED EXAMPLE: empty queue; cycle 1 push_valid = 0 (no record, or the
//   core took it directly): slot 0 written, queue still empty. Cycle 2
//   push_valid = 1: slot 0 written again and kept, head_valid = 1 from
//   cycle 3; cycle 3 writes slot 1.
//
// LOGIC LEVELS (T-9): data write enable 2 (not full, slot decode, both
//   from flops); pointers and count 3; head read 4:1 select 2.
// ASSUMPTIONS (V-9.2): A1 DEPTH is a power of two. A push into a full
//   queue is dropped (as l1d_pf_sms_fifo).
// ==========================================================================
`ifndef L1D_PF3_FIFO_V
`define L1D_PF3_FIFO_V
`default_nettype none

module l1d_pf3_fifo #(
    parameter WIDTH = 79, // default 79 bits: one record
    parameter DEPTH = 4   // default 4 records (power of two)
) (
    input  wire             clk,         // clock
    input  wire             rst,         // sync reset, high
    input  wire [WIDTH-1:0] stage_data,  // WIDTH = 79 bits: the record
    input  wire             push_valid,  // the staged record is kept
    input  wire             pop_en,      // take the head
    output wire             head_valid,  // a record is in the queue
    output wire [WIDTH-1:0] head_data    // WIDTH = 79 bits: oldest record
);

    // ---- local constants (value at the default) ----
    localparam PTR_W = $clog2(DEPTH);            // 2 bits pointer
    localparam CNT_W = $clog2(DEPTH + 1);        // 3 bits count 0..4
    localparam [CNT_W-1:0] DEPTH_V = DEPTH;      // 4, as a count value

    reg  [DEPTH*WIDTH-1:0] mem_q;     // 4 x 79: record storage
    reg  [PTR_W-1:0]       wr_ptr_q;  // 2: next slot to write
    reg  [PTR_W-1:0]       rd_ptr_q;  // 2: oldest slot
    reg  [CNT_W-1:0]       count_q;   // 3: records held
    wire                   full;      // no room
    wire                   push_ok;   // the push is kept
    wire                   pop_ok;    // the pop takes a record

    assign full       = (count_q == DEPTH_V);
    assign push_ok    = push_valid & ~full;
    assign head_valid = (count_q != {CNT_W{1'b0}});
    assign pop_ok     = pop_en & head_valid;
    assign head_data  = mem_q[rd_ptr_q*WIDTH +: WIDTH];

    // ---- data: the free slot is written every cycle, kept only if pushed
    //      (the slot past the tail is not part of the queue) ----
    always @(posedge clk) begin : data_proc
        if (!full) begin
            mem_q[wr_ptr_q*WIDTH +: WIDTH] <= stage_data;
        end
    end // always data_proc

    // ---- pointers and count ----
    always @(posedge clk) begin : ptr_proc
        if (rst) begin
            wr_ptr_q <= {PTR_W{1'b0}};
            rd_ptr_q <= {PTR_W{1'b0}};
            count_q  <= {CNT_W{1'b0}};
        end
        else begin
            if (push_ok) begin
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
    end // always ptr_proc

endmodule

`default_nettype wire
`endif // L1D_PF3_FIFO_V
