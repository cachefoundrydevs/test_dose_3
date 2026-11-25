// ============================================================================
// l1d_fifo.v  -  a small first-in first-out queue
// ============================================================================
// IN SIMPLE WORDS
//   A queue like the line at a shop. push = a new entry joins at the back,
//   pop = the entry at the front leaves. "First word fall through" means the front
//   entry is ALREADY visible on front_data while the FIFO is not empty - there is no
//   extra clock cycle to read it, so a consumer can look and pop in the same cycle.
//
// WHERE IT IS USED (17 copies in cache_v2)
//   - u_inq in l1d_top               the CPU request input queue (one place per ROB index)
//   - u_fifo in l1d_resp_merge       one response FIFO per bank (4 copies)
//   - u_fifo in l1d_snoop_ctrl       one snoop-answer FIFO per bank (4 copies)
//   - u_pf in l1d_fifo_stage         the prefetch FIFO of each bank (4 copies, input tied off in v2)
//   - u_q in l1d_mshr_nc             the queue of non-cacheable accesses of each bank (4 copies)
//
// PORTS (new name, old name, meaning). clk and rst are the usual clock and
// synchronous active-high reset (they clear the pointers and the counter).
//   push_data                            (was din)                      data written into the FIFO when push = 1
//   front_data                           (was dout)                     oldest entry (first word fall through, valid while empty = 0)
//   used_slots                           (was count)                    number of entries in use
//   free_slots                           (was free)                     number of free places (a producer can reserve space with it)
//   push                                                                write push_data into the FIFO this cycle (ignored while full)
//   pop                                                                 remove the oldest entry this cycle (ignored while empty)
//   empty                                                               no entry is stored
//   full                                                                every place is used
//
// HOW IT WORKS
//   mem[]      the storage (an array of DEPTH words, written only on push)
//   wr_ptr     where the next pushed word is written          (moves forward on every push)
//   rd_ptr     the front word (the oldest)                    (moves forward on every pop)
//   cnt        how many words are stored (up on push, down on pop, unchanged if both)
//   empty / full / used_slots / free_slots are simply read from cnt.
//   A producer can use free_slots to RESERVE space ahead of time (this is how the banks keep
//   their response credit, see l1d_bank).
//
// WHAT KIND OF STATE MACHINE IS IT?  (question from an earlier session)
//   A Moore machine: its state is (wr_ptr, rd_ptr, cnt) and every output (empty, full, used_slots,
//   free_slots, front_data) depends on the state only, never directly on push/pop of the same cycle.
//   It has no named states (IDLE, RUN ...), only a counter.
//
// LIMITS
//   - A push while full, or a pop while empty, is ignored (the callers are built so it never happens).
//     A push while full is ignored even if a pop happens in the same cycle.
//   - The overflow / underflow checks at the bottom are $display messages for simulation, not assertions.
//   - The memory has no reset (only the control state has): an entry is valid only because cnt says so.
//
// VERILOG WORDS USED IN THIS FILE
//   reg [W-1:0] mem [0:D-1]   an array of D words of W bits (a small RAM)
//   {N{1'b0}}                 N copies of a 0 bit = the number zero, N bits wide (works for any N)
//   always @(posedge clk)     logic that changes on the rising clock edge: this is what makes flip-flops
//   <=                        "non-blocking" assignment: every register takes its new value at the SAME edge
//   assign a = b;             a permanent wire: a always follows b
//   function ... endfunction  a small piece of combinational logic written once and used like a formula
//   case ({a, b})             {a, b} joins two bits into a 2-bit number; case picks the branch for its value
// ============================================================================

// TIMING OPT (cache_v2_name_changed_timing_opt, OPT_LOG.md F-1..F-3): same behaviour in every cycle, built so that no compare and no
// subtractor sits behind the counter. Measured on the old file (ASAP7, 250 ps): the only failing endpoints were cnt[2:0] (-9.7 ps):
// cnt flop -> half-adder chain of the +1 / -1 and the compares "cnt != DEPTH" / "cnt != 0" -> cnt.
//   F-1  empty_r, full_r   flops that always equal (cnt == 0), (cnt == DEPTH). do_push / do_pop read them (no compare); the new flags are
//                          chosen from two compares of cnt itself (cnt == 1, cnt == DEPTH-1) by the accepted push / pop.
//   F-2  free_r            a down counter that always equals DEPTH - cnt (no subtractor on free_slots).
//   F-3  wr_oh, rd_oh      one-hot write and read pointers (rotate by one slot): the front entry is an AND-OR, a push enables one word.
// Ports, rules (push while full / pop while empty ignored, push and pop together keep the count) and all outputs are unchanged.
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

// TIMING OPT (RM-1, session 42): POP_LAST = 1 builds the next fill level as two candidates from the flops and the push only (with a pop,
//   without a pop), kept apart with (* keep *), and the late pop only selects between them in the last AND-OR. For a user whose pop comes
//   late (resp_merge: FIFO not empty -> arbiter grant -> pop) this keeps the pop out of the level arithmetic, which synthesis otherwise maps
//   to a weak half-adder cell. Same level in every case. Default 0: the present form (every other user, e.g. snoop_ctrl, is unchanged).
module l1d_fifo #(
    parameter WIDTH = 8,                         // bits per entry
    parameter DEPTH = 4,                         // entries, at least 2
    parameter CNT_W = $clog2(DEPTH + 1),         // width of count / free
    parameter POP_LAST = 0                       // 1 = level update with the pop as the last select (RM-1)
) (
    input  wire             clk,                          // clock: all registers change on its rising edge
    input  wire             rst,                          // synchronous reset, active high (control state goes to its idle value)
    input  wire             push,                         // write push_data into the FIFO this cycle (ignored while full)
    input  wire [WIDTH-1:0] push_data,                    // data written into the FIFO when push = 1
    input  wire             pop,                          // remove the oldest entry this cycle (ignored while empty)
    output wire [WIDTH-1:0] front_data,                   // oldest entry (first word fall through, valid while empty = 0)
    output wire             empty,                        // no entry is stored
    output wire             full,                         // every place is used
    output wire [CNT_W-1:0] used_slots,                   // number of entries in use
    output wire [CNT_W-1:0] free_slots,                   // number of free places (a producer can reserve space with it)
    output wire [CNT_W-1:0] free_slots_next               // cache_v6: free places AFTER this cycle's push / pop (the value free_slots takes at the edge)
);

    // DEPTH_C = DEPTH at the width of the counter
    localparam [CNT_W-1:0] DEPTH_C  = DEPTH;     // DEPTH at the counter width

    // ---- storage and state ----
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [DEPTH-1:0] wr_oh;                                // one-hot write pointer: bit i = 1 means the next push fills slot i, DEPTH = 4 bits
    reg [DEPTH-1:0] rd_oh;                                // one-hot read pointer: bit i = 1 means slot i is the front
    // TIMING OPT (F-4, session 41): the fill level is a ONE-HOT register lvl (bit k = 1 means k entries are stored), not a binary counter
    // with a +1 / -1 adder. A push shifts it up, a pop shifts it down: the late push / pop (in resp_merge and snoop_ctrl the pop comes from
    // the arbiter grant) only drive a 3-way mux, no carry chain. empty / full are single bits of it; the counts are encoded from it.
    reg [DEPTH:0]   lvl;                                  // one-hot fill level, DEPTH + 1 = 5 bits
    wire            empty_r = lvl[0];                     // (count == 0)
    wire            full_r  = lvl[DEPTH];                 // (count == DEPTH)
    reg [CNT_W-1:0] cnt;                                  // number of entries stored, encoded from lvl, CNT_W = 3 bits
    reg [CNT_W-1:0] free_r;                               // DEPTH - count, encoded from lvl, CNT_W = 3 bits
    integer i;
    integer j;
    integer k;
    always @* begin
        cnt    = {CNT_W{1'b0}};
        free_r = {CNT_W{1'b0}};
        for (k = 0; k <= DEPTH; k = k + 1)
            if (lvl[k]) begin
                cnt    = cnt    | k[CNT_W-1:0];
                free_r = free_r | (DEPTH_C - k[CNT_W-1:0]);
            end
    end

    // do_push / do_pop are the push and pop that are really accepted: a push needs a free place, a pop needs an entry.
    wire do_push = push && !full_r;
    wire do_pop  = pop  && !empty_r;

    // RM-1: the next level for POP_LAST = 1 (the push / pop table of the case below, rearranged: without a pop: up on a push, else hold;
    // with a pop: hold on a push, else down)
    wire [DEPTH:0] lvl_pl;                                // DEPTH + 1 = 5 bits
    generate
        if (POP_LAST != 0) begin : g_pop_last
            (* keep *) wire [DEPTH:0] lvl_np = do_push ? (lvl << 1) : lvl;     // no pop this cycle, DEPTH + 1 = 5 bits
            (* keep *) wire [DEPTH:0] lvl_p  = do_push ? lvl : (lvl >> 1);     // a pop this cycle, DEPTH + 1 = 5 bits
            assign lvl_pl = ({(DEPTH+1){do_pop}} & lvl_p) | ({(DEPTH+1){!do_pop}} & lvl_np);
        end else begin : g_pop_std
            assign lvl_pl = lvl;                          // unused
        end
    endgenerate


    // ---- the clocked part: pointers, counters and flags. Everything here changes only at the rising clock edge. ----
    always @(posedge clk) begin
        if (rst) begin
            wr_oh   <= {{(DEPTH-1){1'b0}}, 1'b1};
            rd_oh   <= {{(DEPTH-1){1'b0}}, 1'b1};
            lvl     <= {{DEPTH{1'b0}}, 1'b1};
        end else begin
            if (do_push) begin
                for (i = 0; i < DEPTH; i = i + 1)
                    if (wr_oh[i]) mem[i] <= push_data;
                wr_oh <= (wr_oh << 1) | (wr_oh >> (DEPTH - 1));      // rotate by one slot, wraps at DEPTH (any DEPTH)
            end
            if (do_pop)
                rd_oh <= (rd_oh << 1) | (rd_oh >> (DEPTH - 1));
            // fill level: push alone -> one up, pop alone -> one down, both or none -> unchanged
            // (a form with the pop as the last mux select was measured too: faster alone, but slower inside l1d_snoop_ctrl; OPT_LOG F-4)
            if (POP_LAST != 0)
                lvl <= lvl_pl;                            // RM-1
            else
                case ({do_push, do_pop})
                    2'b10:   lvl <= lvl << 1;
                    2'b01:   lvl <= lvl >> 1;
                    default: ;
                endcase
        end
    end

    // ---- outputs: all read straight from the state ----
    // The front entry is the AND-OR of the words with the one-hot read pointer. The words are first copied to a flat vector so that
    // the always @* is sensitive to a plain signal (an always @* over an array of words is awkward in Icarus Verilog).
    wire [DEPTH*WIDTH-1:0] mem_flat;
    genvar g;
    generate
        for (g = 0; g < DEPTH; g = g + 1) begin : g_flat
            assign mem_flat[g*WIDTH +: WIDTH] = mem[g];
        end
    endgenerate
    reg [WIDTH-1:0] front;
    always @* begin
        front = {WIDTH{1'b0}};
        for (j = 0; j < DEPTH; j = j + 1)
            front = front | (mem_flat[j*WIDTH +: WIDTH] & {WIDTH{rd_oh[j]}});
    end
    assign front_data  = front;
    assign empty       = empty_r;
    assign full        = full_r;
    assign used_slots  = cnt;
    assign free_slots  = free_r;

    // ---- cache_v6: the next fill level and the free count encoded from it (for a credit computed one cycle ahead, l1d_bank_s0).
    //      Same table as the clocked part: push alone up, pop alone down, both or none unchanged. ----
    reg [DEPTH:0]   lvl_next;                             // one-hot fill level after this cycle, DEPTH + 1 = 5 bits
    reg [CNT_W-1:0] free_next_r;                          // free places after this cycle, CNT_W = 3 bits
    integer         n;                                    // loop variable: fill level encoded
    always @* begin
        case ({do_push, do_pop})
            2'b10:   lvl_next = lvl << 1;
            2'b01:   lvl_next = lvl >> 1;
            default: lvl_next = lvl;
        endcase
        free_next_r = {CNT_W{1'b0}};
        for (n = 0; n <= DEPTH; n = n + 1)
            if (lvl_next[n]) free_next_r = free_next_r | (DEPTH_C - n[CNT_W-1:0]);
    end
    assign free_slots_next = free_next_r;

// ---- simulation-only checks (compiled only when L1D_DEBUG is defined) ----
`ifdef L1D_DEBUG
    // Debug check (simulation only): a push into a full FIFO or a pop from an
    // empty one is a bug in the caller (the entry would be lost / invented).
    always @(posedge clk) begin
        if (!rst) begin
            if (push && cnt == DEPTH_C)
                $display("[%0t] L1D FIFO %m ERROR push while full (depth %0d): entry lost", $time, DEPTH);
            if (pop && cnt == {CNT_W{1'b0}})
                $display("[%0t] L1D FIFO %m ERROR pop while empty", $time);
            if (push === 1'bx || pop === 1'bx)
                $display("[%0t] L1D FIFO %m WARNING push/pop is X", $time);
        end
    end
`endif

endmodule

`default_nettype wire
