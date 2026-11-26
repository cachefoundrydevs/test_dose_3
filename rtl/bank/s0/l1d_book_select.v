// ============================================================================
// l1d_book_select.v  -  the BOOKING of one bank: who enters the pipeline NEXT cycle? (decided one cycle early, from flops)
// ============================================================================
// IN SIMPLE WORDS  (cache_v6, PLAN_v6.md section 4.1; replaces l1d_prio_select of the lean S0)
//   The lean S0 chose the operation of cycle c IN cycle c, in one long chain (credits, same-set block, priority, grant, SRAM
//   address). cache_v6 "books" the operation of cycle c in cycle c-1: everything that is known from flops (snoops, refill and
//   flush/clean tokens, waiting tracker entries, the prefetch hint) is picked a cycle early, and the result is a few flops
//   (book_valid_q, book_kind_q). In cycle c the address lane only asks "booked, or the request arriving now?".
//   Only a request that ARRIVES in cycle c and finds nothing booked goes straight in (the fresh issue, l1d_req_tracker).
//
//   Priority (unchanged, rule R3):  snoop > refill token > flush/clean token > oldest READY entry > arrival of this cycle
//   (that was not issued fresh) > prefetch.
//
// CONDITIONS (as l1d_prio_select, now about the NEXT cycle)
//   - CREDITS: a snoop needs a free place in the snoop-answer FIFO; a CPU entry needs a response credit for its port
//     (entry_credit_ok / arrival_credit_ok, computed in l1d_bank_s0 from the next free counts); a vector store needs a data
//     place (it had one when it was allocated, so it is always 1 here).
//   - SKIP after a refill: the refill writes line and tag in its S2, three cycles after its issue: nothing is booked and nothing
//     issues fresh for REFILL_SKIP cycles after a refill issue (the bank passes REFILL_SKIP + 1 = 3).
//   - AMO BUBBLE: an AMO issued this cycle writes in S3: nothing is booked and nothing issues fresh next cycle.
//   - STORE IN FLIGHT (store_in_flight_next): a store-class CPU op will be in R, S1 or S2 (or an AMO writes in S3) next cycle: a
//     snoop, refill or flush/clean token (line readers) waits; while one waits no CPU op is booked or issued fresh, so the stores
//     drain in at most 3 cycles (bounded wait, rule S-1).
//
// STARVATION BOUND (rule S-1)
//   A refill token loses to snoops at most SNP_STARVE (8) times in a row; then it wins over the next snoop. Entries are booked
//   oldest first (age matrix), so every READY entry is booked after finitely many older ones; arrivals are booked only when no
//   READY entry exists.
//
// STAGE / CYCLE
//   Works in cycle c-1 for cycle c. Inputs: flops of the tracker, the snoop buffer, the MSHR, l1d_maint and the prefetch
//   register, plus two late signals of this cycle from l1d_bank_s0 (amo_issued_now, store_in_flight_next: they include the
//   fresh issue of this cycle, about 6 levels before this block). Outputs: flops. Estimated depth of the booking chain from
//   the tracker flops: candidate flags 2 + priority 3 + (the age pick and the set AND-OR are in the tracker, 5) = 9 to 10
//   levels into the book flops (PLAN_v6 path F; not the hit path).
//
// INSTANTIATED BY
//   l1d_bank_s0, instance u_book (one per bank).
//
// PARAMETERS
//   REFILL_SKIP   default 3: cycles without an issue after a refill issue (the bank passes REFILL_SKIP + 1 of l1d_defines)
//   SNP_STARVE    default 8: snoop bookings a waiting refill token may lose in a row
//   BOOK_KIND_W   5: the one-hot kind (BOOK_*_B of l1d_encodings.vh)
//
// PORTS (name, direction, width, meaning)
//   clk, rst                 in   1   clock; synchronous reset (nothing booked, counters 0)
//   snoop_offer_valid        in   1   the snoop buffer offers a snoop (flops)
//   snoop_credit_ok          in   1   the snoop answer FIFO has a free place (flops)
//   refill_token_valid       in   1   the MSHR offers a refill token (flops)
//   maint_token_valid        in   1   l1d_maint offers a flush/clean token (flops)
//   entry_ready_valid        in   1   the tracker has a READY entry without dependencies (flops)
//   entry_credit_ok          in   1   that entry's port has a response credit next cycle (flops)
//   arrival_ready_valid      in   1   an arrival of this cycle (not the fresh one) has no dependencies (late: pins + flops)
//   arrival_credit_ok        in   1   its port has a response credit next cycle
//   prefetch_valid           in   1   the prefetch register holds a hint (flop)
//   amo_issued_now           in   1   an AMO enters R next cycle (booked AMO issued now, or a fresh AMO: late)
//   store_in_flight_next     in   1   a store-class op will be in R / S1 / S2, or an AMO in S3, next cycle (late)
//   amo_write_next           in   1   an AMO is in S2 (flops): no CPU booking for its S3 write cycle (session 59 fix)
//   book_valid_q             out  1   flop: something is booked for the next cycle
//   book_kind_q              out  5   flop: what (one-hot, BOOK_*_B)
//   fresh_allowed_q          out  1   flop: next cycle nothing is booked and the door is open (an arrival may issue fresh)
//   book_now_*               out  1   the booking decision of THIS cycle (combinational; l1d_bank_s0 captures the fields)
// ============================================================================
`ifndef L1D_BOOK_SELECT_V
`define L1D_BOOK_SELECT_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_book_select #(
    parameter REFILL_SKIP = 3,                            // default 3: cycles without an issue after a refill issue
    parameter SNP_STARVE  = 8,                            // default 8: snoop bookings a waiting refill token may lose in a row
    parameter BOOK_KIND_W = 5                             // default 5: width of the one-hot kind (BOOK_*_B)
) (
    // ---- clock and reset ----
    input  wire                   clk,                    // clock: all registers change on its rising edge
    input  wire                   rst,                    // synchronous reset, active high
    // ---- candidates (flops of their sources) ----
    input  wire                   snoop_offer_valid,      // the snoop buffer offers a snoop
    input  wire                   snoop_credit_ok,        // the snoop answer FIFO has a free place
    input  wire                   refill_token_valid,     // the MSHR offers a refill token
    input  wire                   maint_token_valid,      // l1d_maint offers a flush/clean token
    input  wire                   entry_ready_valid,      // a READY tracker entry without dependencies exists
    input  wire                   entry_credit_ok,        // its port has a response credit next cycle
    input  wire                   arrival_ready_valid,    // an arrival of this cycle (not the fresh one) may be booked (late)
    input  wire                   arrival_credit_ok,      // its port has a response credit next cycle
    input  wire                   prefetch_valid,         // the prefetch register holds a hint
    // ---- gates about the next cycle ----
    input  wire                   amo_issued_now,         // an AMO is issued this cycle (it is in R next cycle: write bubble)
    input  wire                   store_in_flight_next,   // a store-class op will be in R / S1 / S2 next cycle, or an AMO in S3
    input  wire                   amo_write_next,         // an AMO is in S2 (flops): it may write its line in S3 next cycle
    // ---- the booking (flops) and this cycle's decision (combinational, for the field capture in l1d_bank_s0) ----
    output reg                    book_valid_q,           // flop: something is booked for the next cycle
    output reg  [BOOK_KIND_W-1:0] book_kind_q,            // flop: its kind, one-hot (BOOK_*_B of l1d_encodings.vh)
    output reg                    fresh_allowed_q,        // flop: next cycle an arrival may issue fresh (nothing booked, door open)
    output wire                   book_now_snoop,         // this cycle books the snoop
    output wire                   book_now_refill,        // ... the refill token
    output wire                   book_now_maint,         // ... the flush/clean token
    output wire                   book_now_entry,         // ... the oldest READY entry
    output wire                   book_now_arrival,       // ... the arrival of this cycle
    output wire                   book_now_prefetch       // ... the prefetch hint
);
    // ---- local constants (value at the defaults) ----
    localparam SKIP_W   = (REFILL_SKIP > 0) ? $clog2(REFILL_SKIP + 1) : 1;   // skip counter, 2 bits at REFILL_SKIP = 3
    localparam STARVE_W = $clog2(SNP_STARVE + 1);                              // starvation counter, 4 bits at SNP_STARVE = 8
    localparam [SKIP_W-1:0]   SKIP_C   = REFILL_SKIP;                          // REFILL_SKIP at the counter width
    localparam [STARVE_W-1:0] STARVE_C = SNP_STARVE;                           // SNP_STARVE at the counter width
    localparam BOOK_SNOOP_B  = 0;                         // bit numbers of book_kind_q (same as l1d_encodings.vh)
    localparam BOOK_REFILL_B = 1;
    localparam BOOK_MAINT_B  = 2;
    localparam BOOK_ENTRY_B  = 3;
    localparam BOOK_PF_B     = 4;

    // ---- state: the skip counter (cycles still closed after a refill issue) and the starvation counter ----
    reg  [SKIP_W-1:0]   skip_q;                           // closed cycles left, 2 bits
    reg                 skip_open_q;                      // flop: skip_q == 0 (rule T-8: the terminal compare is a flop)
    reg  [STARVE_W-1:0] starve_cnt_q;                     // snoop bookings while a refill token waited, 4 bits
    reg                 starve_q;                         // flop: starve_cnt_q == SNP_STARVE (the refill wins the next round)

    // ---- the conditions for the NEXT cycle ----
    wire door_open_next;                                  // no skip cycle left and no AMO write bubble
    wire snoop_request;                                   // a snoop wants to go (credit, not starving a refill)
    wire snoop_ok;                                        // ... and may go (no store in flight)
    wire snoop_waits;                                     // ... but a store is in flight: it waits, and so does CPU work
    wire refill_ok;                                       // the refill token may go
    wire maint_ok;                                        // the flush/clean token may go
    wire cpu_go;                                          // CPU work may go: door open, no line reader offered or waiting
    assign door_open_next = skip_open_q && !amo_issued_now;
    assign snoop_request  = snoop_offer_valid && snoop_credit_ok && !(starve_q && refill_token_valid);
    assign snoop_ok       = snoop_request && !store_in_flight_next;
    assign snoop_waits    = snoop_request && store_in_flight_next;
    assign refill_ok      = refill_token_valid && !snoop_ok && !store_in_flight_next;
    assign maint_ok       = maint_token_valid && !snoop_ok && !refill_token_valid && !store_in_flight_next;
    // amo_write_next (fix 2026-11-10, session 59): an AMO leaves the tracker in S2 but writes its line in S3. A request of
    // its set that arrived in the AMO's S2 cycle got no dependency on it (the entry was being freed), and a booked request
    // is not checked in stage R, so no CPU request may be booked for the AMO's S3 cycle (one bubble per AMO; fresh issues
    // in that cycle are still covered by the R check amo_hit).
    assign cpu_go         = door_open_next && !snoop_ok && !snoop_waits && !refill_token_valid && !maint_token_valid &&
                            !amo_write_next;

    // ---- the decision: fixed priority snoop > refill > maint > entry > arrival > prefetch (a priority chain on purpose) ----
    assign book_now_snoop    = door_open_next && snoop_ok;
    assign book_now_refill   = door_open_next && refill_ok;
    assign book_now_maint    = door_open_next && maint_ok;
    assign book_now_entry    = cpu_go && entry_ready_valid && entry_credit_ok;
    assign book_now_arrival  = cpu_go && !book_now_entry && arrival_ready_valid && arrival_credit_ok;
    assign book_now_prefetch = cpu_go && !book_now_entry && !book_now_arrival && prefetch_valid;
    wire book_now_any;                                    // something is booked this cycle
    assign book_now_any = book_now_snoop || book_now_refill || book_now_maint || book_now_entry || book_now_arrival || book_now_prefetch;

    // ---- the flops ----
    always @(posedge clk) begin : book_proc
        if (rst) begin
            book_valid_q    <= 1'b0;
            book_kind_q     <= {BOOK_KIND_W{1'b0}};
            fresh_allowed_q <= 1'b1;
        end else begin
            book_valid_q                <= book_now_any;
            book_kind_q[BOOK_SNOOP_B]   <= book_now_snoop;
            book_kind_q[BOOK_REFILL_B]  <= book_now_refill;
            book_kind_q[BOOK_MAINT_B]   <= book_now_maint;
            book_kind_q[BOOK_ENTRY_B]   <= book_now_entry || book_now_arrival;
            book_kind_q[BOOK_PF_B]      <= book_now_prefetch;
            // an arrival may issue fresh next cycle only when nothing is booked and CPU work is allowed (door open, no line
            // reader offered or waiting)
            fresh_allowed_q             <= cpu_go && !book_now_any;
        end
    end // always book_proc

    // ---- skip counter: a refill booking loads it (the refill issues next cycle, then REFILL_SKIP closed cycles follow) ----
    always @(posedge clk) begin : skip_proc
        if (rst) begin
            skip_q      <= {SKIP_W{1'b0}};
            skip_open_q <= 1'b1;
        end else if (book_now_refill) begin
            skip_q      <= SKIP_C;
            skip_open_q <= (REFILL_SKIP == 0);
        end else if (!skip_open_q) begin
            skip_q      <= skip_q - 1'b1;
            skip_open_q <= (skip_q == {{(SKIP_W-1){1'b0}}, 1'b1});
        end
    end // always skip_proc

    // ---- starvation counter: snoop bookings while a refill token waits; cleared when the refill is booked or no token waits ----
    always @(posedge clk) begin : starve_proc
        if (rst || book_now_refill || !refill_token_valid) begin
            starve_cnt_q <= {STARVE_W{1'b0}};
            starve_q     <= 1'b0;
        end else if (book_now_snoop && !starve_q) begin
            starve_cnt_q <= starve_cnt_q + 1'b1;
            starve_q     <= (starve_cnt_q == STARVE_C - 1'b1);
        end
    end // always starve_proc

endmodule

`default_nettype wire
`endif // L1D_BOOK_SELECT_V
