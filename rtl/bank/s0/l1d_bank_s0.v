// ============================================================================
// l1d_bank_s0.v  -  bank stage G (S0) of cache_v6: tracker, booking, address lane, decision lane, G -> R registers
// ============================================================================
// IN SIMPLE WORDS  (cache_v6, PLAN_v6.md section 4; replaces l1d_bank_fifo_s0 of the lean S0)
//   This is the front door of one bank. Three short lanes work side by side in every cycle c:
//     ADDRESS LANE   g_set = the set of the operation BOOKED for this cycle (a flop), or else the set of the lowest port that
//                    sends a request to this bank now. It goes to the tag and data SRAMs through one 2:1 select (rule T-3).
//     DECISION LANE  "does the booked operation issue, or does the arriving request issue fresh?" from flops and pins
//                    (about 6 levels, l1d_req_tracker). The result is r_valid, the tracker state and the grants to the sources.
//     DATA LANE      the r_* registers load the booked operation's fields (AND-OR of the tracker entries by a one-hot flop) or
//                    the arriving request's packet; r_valid says whether they mean anything.
//   The BOOKING for cycle c was done in cycle c-1 (l1d_book_select): snoop > refill token > flush/clean token > oldest READY
//   tracker entry > an arrival of c-1 that could not issue fresh > prefetch, all from flops.
//   The REQUEST TRACKER (l1d_req_tracker) gives every CPU request of this bank an entry at arrival (up to 4 in one cycle), with
//   its same-set dependencies as bits (rule T-2); a fresh request issues on trust and stage R checks its bits.
//
//      arrivals (pins) --+--> l1d_req_tracker --> fresh_issue, arrival candidate --+
//                        |                        oldest READY entry -------------+--> l1d_book_select --> book_*_q (flops)
//      snoop buffer, refill token, flush/clean token, prefetch register ----------+
//      book_*_q | lowest arrival ---> g_set (SRAM address), r_* registers;   r_valid <= booked issue | fresh_issue
//
// STAGE / CYCLE
//   Cycle c0 of a hit (stage G). The address lane and the decision lane are combinational in c0; the booking flops are
//   written at the end of c0-1; the r_* registers at the end of c0. Outputs to the sources: the grant of a booked snoop,
//   refill token, flush/clean token or prefetch is "its booking issues now" (flops, plus a cancel compare if the source's
//   offer changed since the booking: then nothing issues this cycle and the source is booked again).
//
// INSTANTIATED BY
//   l1d_bank, instance u_s0 (one per bank; l1d_top builds NUM_BANKS = 4 banks with BANK_ID = 0..3).
//
// PARAMETERS
//   `L1D_PARAMS   all cache parameters (l1d_defines.vh); used here: LSU_PORTS (4), TRK_ENTRIES (8), REFILL_SKIP (2),
//                 SNP_STARVE (8), PC_W (64) and the widths of l1d_fields.vh
//   BANK_ID       default 0: number of this bank (its bank bits in a line address)
//
// PORTS (name, direction, width at the defaults, meaning)       N = 0..3: the request port
//   clk, rst                           in   1      clock; synchronous reset, active high
//   arrival_request_N / _packet_N / _pc_N  in 1/148/64  port N's pins request is for this bank (ring empty) and its fields
//   drain_request / _packet / _pc      in   1/148/64  the ring head of this bank (a skidded request) and its fields
//   arrival_allocated_N, drain_allocated  out 1    the request got a tracker entry (the front pushes the others into the skids)
//   tracker_free_count_q               out  4      flop: free tracker entries (the front's room rule)
//   lsu_response_fifo_free_N           in   3      free places of port N's response FIFO for this bank (the credit of answers)
//   lsu_response_fifo_free_next_N      in   3      ... after this cycle (the credit of the next cycle's issue)
//   snoop_request_valid/_line_addr/_invalidate  in 1/58/1  a snoop from l1d_snoop_ctrl;  snoop_request_ready out 1
//   snoop_response_fifo_free           in   2      free places of this bank's snoop answer FIFO
//   prefetch_request_valid/_line_addr/_unique  in 1/58/1  a prefetch hint; prefetch_request_ready out 1 (0 while the hint is booked)
//   refill_handover_busy               in   1      the L2 port is handing a line to this bank: start no snoop
//   s1_valid/_kind/_req                in   1/2/148  the operation in S1
//   s2_valid/_kind/_req/_hit/_sid/_line/_seat/_kill  in  the operation in S2 (its entry, and whether it was cancelled in R)
//   s2_answered                        in   1      the CPU request in S2 answers now (s3_load)
//   act_valid/_code/_id                in   1/3/3  Stage 2 result of the CPU request in S2
//   snp_act_valid/_code/_id            in   1/2/3  Stage 2 result of the snoop in S2
//   s2_train_sent, s2_seat_trained, s2_seat_pc   in/out  training record bookkeeping of the entry in S2
//   s3_amo_v                           in   1      an AMO writes its line in S3 this cycle
//   lock_valid, lock_snp, lock_line, ev_amor  in  the AMO lock (never set in this version)
//   tok_valid/_id/_line                in   1/3/58 the MSHR refill token
//   mt_valid/_set/_way/_op             in   1/6/2/1 the flush/clean token
//   m_resp_valid, n_resp_valid         in   1/1    an MSHR / NC answer waits to be pushed
//   mshr_freed, vc_freed, nc_freed     in   8/8/1  wake-up pulses
//   any_gnt                            out  1      the SRAMs read set g_set this cycle (booked, or some arrival: read enable)
//   g_set                              out  6      the set they read
//   gnt_refill, gnt_maint              out  1/1    the refill token / flush-clean token issues now
//   r_valid ... r_sid, r_seat, r_fast  out  regs   the Stage R registers (r_seat = tracker entry, r_fast = fresh issue)
//   tracker_empty                      out  1      no tracker entry in use (bank idle check)
//   miss_ok                            out  1      an MSHR / NC answer may be pushed now (credit, used in l1d_bank_s3)
//   own_response_push_port             in   4      crossbank step 1: this bank pushes an answer for port p this cycle (from l1d_bank, in-bank flops only)
//   vec_slot_available, vec_write_slot in  1/1    a vector data place is free, and the lowest free one
//   vec_write_valid                    out  1      a vector store is allocated now: keep its line in place vec_write_slot
//
// WORKED EXAMPLE (idle bank, a load arrives on port 0 in c0)
//   c0-1: nothing to book: book_valid_q = 0, fresh_allowed_q = 1.
//   c0:   arrival_request_0 = 1 -> lowest port 0 -> g_set = its set (address lane, 6 levels); the tracker allocates entry k;
//         fresh_issue = 1 (fresh_allowed_q, credit_q_0) -> r_valid <= 1, r_fast <= 1, r_seat <= k, entry k INFL.
//   c1:   stage R: deps of entry k = 0, no write to its set at the edge -> not killed. S1: hit. S2: answer; entry k freed.
//
// ESTIMATED LOGIC LEVELS (PLAN_v6 4.6; not measured): address lane 6 from the pins / 2 from the book flops; decision lane 6;
//   data lane 4 from the entry flops; credits into flops 5; booking into flops 9 to 10.
// ============================================================================
`ifndef L1D_BANK_S0_V
`define L1D_BANK_S0_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_bank_s0 #(
    `L1D_PARAMS,                                       // all cache parameters (l1d_defines.vh)
    parameter BANK_ID = 0                              // default 0: number of this bank (its bank bits in a line address)
) (
    clk, rst,
    arrival_request_0, arrival_request_1, arrival_request_2, arrival_request_3,
    arrival_packet_0, arrival_packet_1, arrival_packet_2, arrival_packet_3,
    arrival_pc_0, arrival_pc_1, arrival_pc_2, arrival_pc_3,
    drain_request, drain_packet, drain_pc,
    arrival_allocated_0, arrival_allocated_1, arrival_allocated_2, arrival_allocated_3, drain_allocated,
    tracker_free_count_q,
    lsu_response_fifo_free_0, lsu_response_fifo_free_1, lsu_response_fifo_free_2, lsu_response_fifo_free_3,
    lsu_response_fifo_free_next_0, lsu_response_fifo_free_next_1, lsu_response_fifo_free_next_2, lsu_response_fifo_free_next_3,
    snoop_request_valid, snoop_request_line_addr, snoop_request_invalidate, snoop_request_ready, snoop_response_fifo_free,
    prefetch_request_valid, prefetch_request_line_addr, prefetch_request_unique, prefetch_request_ready,
    refill_handover_busy,
    s1_valid, s1_kind, s1_req,
    s2_valid, s2_kind, s2_req, s2_hit, s2_sid, s2_line, s2_seat, s2_kill, s2_answered,
    act_valid, act_code, act_id, snp_act_valid, snp_act_code, snp_act_id,
    s2_train_sent, s2_seat_trained, s2_seat_pc,
    s3_amo_v,
    lock_valid, lock_snp, lock_line, ev_amor,
    tok_valid, tok_id, tok_line, mt_valid, mt_set, mt_way, mt_op,
    m_resp_valid, n_resp_valid,
    mshr_freed, vc_freed, nc_freed,
    any_gnt, g_set, gnt_refill, gnt_maint,
    r_valid, r_kind, r_seat, r_req, r_fast, r_mid, r_line, r_mway, r_mop, r_sinv, r_sid, r_deps,
    tracker_empty, miss_ok, own_response_push_port,
    vec_slot_available, vec_write_slot, vec_write_valid
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // ---- local constants (value at the defaults) ----
    localparam [BANK_W-1:0]   BANK_C = BANK_ID;                  // this bank's number in a line address, BANK_W = 2 bits
    localparam [OFFSET_W-1:0] OFF0   = {OFFSET_W{1'b0}};         // zero line offset (debug messages print byte addresses), 6 bits
    localparam SET_IN_REQ  = REQ_ADDR_LSB + OFFSET_W + LA_SET_LSB;   // lowest bit of the set number in a request packet
    localparam LINE_IN_REQ = REQ_ADDR_LSB + OFFSET_W;                // lowest bit of the line address in a request packet
    localparam NE          = TRK_ENTRIES;                        // tracker entries, 8
    localparam [RCNT_W:0]  CREDIT_ONE   = 1;                     // CB-1: free > 0, the first threshold of port_miss_ok, 4 bits
    localparam [RCNT_W:0]  CREDIT_TWO   = 2;                     // the "+1 for the issue, +1 for a push" of the credit rule, 4 bits
    localparam [RCNT_W:0]  CREDIT_THREE = 3;                     // ... with an assumed issue of the same port, 4 bits
    localparam [RCNT_W:0]  CREDIT_FOUR  = 4;                     // crossbank step 1: two + (R of the port or an own push now) + an issue, 4 bits
    localparam [RCNT_W:0]  CREDIT_FIVE  = 5;                     // crossbank step 1: two + R + own push + issue (full stream needs RESP_FIFO_DEPTH >= 5), 4 bits

    // ======================================================================
    // Ports (declared after the includes so that the derived widths of l1d_fields.vh can be used)
    // ======================================================================
    input  wire                        clk;                      // clock: all registers change on its rising edge
    input  wire                        rst;                      // synchronous reset, active high
    // ---- arrivals (from l1d_port_front through l1d_bank) ----
    input  wire                        arrival_request_0;        // port 0's pins request is for this bank (ring empty)
    input  wire                        arrival_request_1;        // port 1
    input  wire                        arrival_request_2;        // port 2
    input  wire                        arrival_request_3;        // port 3
    input  wire [REQ_W-1:0]            arrival_packet_0;         // port 0 packet, REQ_W = 148 bits
    input  wire [REQ_W-1:0]            arrival_packet_1;         // port 1 packet, 148 bits
    input  wire [REQ_W-1:0]            arrival_packet_2;         // port 2 packet, 148 bits
    input  wire [REQ_W-1:0]            arrival_packet_3;         // port 3 packet, 148 bits
    input  wire [PC_W-1:0]             arrival_pc_0;             // port 0 PC, PC_W = 64 bits
    input  wire [PC_W-1:0]             arrival_pc_1;             // port 1 PC, 64 bits
    input  wire [PC_W-1:0]             arrival_pc_2;             // port 2 PC, 64 bits
    input  wire [PC_W-1:0]             arrival_pc_3;             // port 3 PC, 64 bits
    input  wire                        drain_request;            // the ring head of this bank asks for an entry (ring not empty)
    input  wire [REQ_W-1:0]            drain_packet;             // its packet, 148 bits
    input  wire [PC_W-1:0]             drain_pc;                 // its PC, 64 bits
    output wire                        arrival_allocated_0;      // port 0's arrival got an entry
    output wire                        arrival_allocated_1;      // port 1
    output wire                        arrival_allocated_2;      // port 2
    output wire                        arrival_allocated_3;      // port 3
    output wire                        drain_allocated;          // the drained request got an entry
    output wire [TRK_CNT_W-1:0]        tracker_free_count_q;     // flop: free tracker entries, 4 bits
    // ---- response credits ----
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_0; // free places of port 0's response FIFO for this bank, RCNT_W = 3 bits
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_1; // ... port 1
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_2; // ... port 2
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_3; // ... port 3
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_next_0;  // free places of port 0's FIFO after this cycle, 3 bits
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_next_1;  // ... port 1
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_next_2;  // ... port 2
    input  wire [RCNT_W-1:0]           lsu_response_fifo_free_next_3;  // ... port 3
    // ---- snoops (l1d_snoop_ctrl) ----
    input  wire                        snoop_request_valid;      // a snoop for this bank
    input  wire [LINE_ADDR_W-1:0]      snoop_request_line_addr;  // its line address, LINE_ADDR_W = 58 bits
    input  wire                        snoop_request_invalidate; // 1 = invalidate, 0 = read
    output wire                        snoop_request_ready;      // the snoop buffer has a free entry
    input  wire [SNPR_CNT_W-1:0]       snoop_response_fifo_free; // free places of the snoop answer FIFO, SNPR_CNT_W = 2 bits
    // ---- prefetch hints (l1d_prefetch_unit through l1d_top) ----
    input  wire                        prefetch_request_valid;   // a prefetch hint for this bank
    input  wire [LINE_ADDR_W-1:0]      prefetch_request_line_addr;  // its line address, 58 bits
    input  wire                        prefetch_request_unique;  // it wants the line unique (store-line prefetch)
    output wire                        prefetch_request_ready;   // a new hint may replace the waiting one (0 while it is booked)
    input  wire                        refill_handover_busy;     // the L2 port hands a line to this bank: start no snoop
    // ---- the operations in the later stages ----
    input  wire                        s1_valid;                 // stage S1 holds an operation
    input  wire [1:0]                  s1_kind;                  // its kind (K_LSU, K_REFILL, K_MAINT, K_SNOOP)
    input  wire [REQ_W-1:0]            s1_req;                   // its request packet, 148 bits
    input  wire                        s2_valid;                 // stage S2 holds an operation
    input  wire [1:0]                  s2_kind;                  // its kind
    input  wire [REQ_W-1:0]            s2_req;                   // its request packet, 148 bits
    input  wire                        s2_hit;                   // the CPU request in S2 was a served hit in S1
    input  wire [SNP_ID_W-1:0]         s2_sid;                   // snoop buffer entry of the snoop in S2, SNP_ID_W = 1 bit
    input  wire [LINE_ADDR_W-1:0]      s2_line;                  // line address of the operation in S2, 58 bits
    input  wire [SEAT_W-1:0]           s2_seat;                  // tracker entry of the CPU request in S2, SEAT_W = 3 bits
    input  wire                        s2_kill;                  // it was cancelled in R (ghost)
    input  wire                        s2_answered;              // it answers now (s3_load of l1d_bank_s2)
    // ---- Stage 2 results (l1d_bank_s2) ----
    input  wire                        act_valid;                // Stage 2 has a result for the CPU request in S2
    input  wire [2:0]                  act_code;                 // ACT_* (l1d_encodings.vh)
    input  wire [WID_W-1:0]            act_id;                   // MSHR entry or victim slot it waits for, WID_W = 3 bits
    input  wire                        snp_act_valid;            // Stage 2 has a result for the snoop in S2
    input  wire [1:0]                  snp_act_code;             // SACT_DONE / SACT_WMSHR / SACT_WVC
    input  wire [WID_W-1:0]            snp_act_id;               // MSHR entry or victim slot the snoop waits for, 3 bits
    input  wire                        s2_train_sent;            // S2 sends the training record of the entry in S2 now
    output wire                        s2_seat_trained;          // that entry already sent its record
    output wire [PC_W-1:0]             s2_seat_pc;               // that entry's PC, 64 bits
    input  wire                        s3_amo_v;                 // an AMO hit writes its line in S3 this cycle
    // ---- AMO lock (l1d_reservation_unit; never set in this version) ----
    input  wire                        lock_valid;               // a line of this bank is AMO-locked
    input  wire                        lock_snp;                 // a snoop to the locked line must wait
    input  wire [LINE_ADDR_W-1:0]      lock_line;                // the locked line, 58 bits
    input  wire                        ev_amor;                  // the lock is set at the next edge on s2_line
    // ---- tokens of the miss units ----
    input  wire                        tok_valid;                // the MSHR offers a refill token
    input  wire [MSHR_ID_W-1:0]        tok_id;                   // its MSHR entry, MSHR_ID_W = 3 bits
    input  wire [LINE_ADDR_W-1:0]      tok_line;                 // its line address, 58 bits
    input  wire                        mt_valid;                 // l1d_maint offers a flush/clean token
    input  wire [INDEX_W-1:0]          mt_set;                   // its set, INDEX_W = 6 bits
    input  wire [WAY_W-1:0]            mt_way;                   // its way, WAY_W = 2 bits
    input  wire                        mt_op;                    // 0 = clean, 1 = flush
    input  wire                        m_resp_valid;             // an MSHR answer waits to be pushed
    input  wire                        n_resp_valid;             // a non-cacheable answer waits to be pushed
    input  wire [MSHR_ENTRIES-1:0]     mshr_freed;               // one pulse per MSHR entry that became free, 8 bits
    input  wire [VC_SLOTS-1:0]         vc_freed;                 // one pulse per victim-cache slot that became free, 8 bits
    input  wire                        nc_freed;                 // a non-cacheable queue place became free
    // ---- the grant (combinational, this cycle) ----
    output wire                        any_gnt;                  // the SRAMs read set g_set (booked, or some arrival)
    output wire [INDEX_W-1:0]          g_set;                    // the set they read, 6 bits
    output wire                        gnt_refill;               // the refill token issues now (to the MSHR)
    output wire                        gnt_maint;                // the flush/clean token issues now (to l1d_maint)
    // ---- Stage R registers (G -> R) ----
    output reg                         r_valid;                  // Stage R holds an operation
    output reg  [1:0]                  r_kind;                   // its kind (K_LSU, K_REFILL, K_MAINT, K_SNOOP)
    output reg  [SEAT_W-1:0]           r_seat;                   // its tracker entry (CPU request), 3 bits
    output reg  [REQ_W-1:0]            r_req;                    // its request packet, 148 bits
    output reg                         r_fast;                   // it issued fresh (stage R checks its dependencies)
    output reg  [MSHR_ID_W-1:0]        r_mid;                    // MSHR entry of a refill token, 3 bits
    output reg  [LINE_ADDR_W-1:0]      r_line;                   // its line address, 58 bits
    output reg  [WAY_W-1:0]            r_mway;                   // way of a flush/clean token, 2 bits
    output reg                         r_mop;                    // operation of a flush/clean token
    output reg                         r_sinv;                   // snoop: 1 = invalidate
    output reg  [SNP_ID_W-1:0]         r_sid;                    // snoop buffer entry, 1 bit
    output wire [TRK_ENTRIES-1:0]      r_deps;                   // dependency bits of the entry in R (tracker flops, for the R check), 8 bits
    // ---- status ----
    output wire                        tracker_empty;            // no tracker entry in use
    output wire                        miss_ok;                  // an MSHR / NC answer may be pushed now
    // CROSSBANK STEP 1 (2026-11-25, see docs/CROSSBANK_CHANGES_v6.md): this bank pushes an answer for port p into its response FIFO THIS cycle.
    // Comes from l1d_bank (decode of lsu_response_valid and the port field of lsu_response_packet, flops of S2 / MSHR / NC in this bank only).
    // With it the credit reads only free_slots (a flop) and never free_slots_next (a function of all four banks' pushes and the LSU read pin).
    input  wire [LSU_PORTS-1:0]        own_response_push_port;   // bit p = 1: an answer for port p is pushed this cycle, LSU_PORTS = 4 bits
    // ---- vector store data places (l1d_vec_store_buf) ----
    input  wire                        vec_slot_available;       // a vector data place is free
    input  wire [VEC_SLOT_W-1:0]       vec_write_slot;           // the lowest free place, VEC_SLOT_W = 1 bit
    output wire                        vec_write_valid;          // a vector store is allocated now: keep its line in that place

    // ======================================================================
    // What is in the pipeline (flops of this and the later stages)
    // ======================================================================
    reg                r_cpu_q;                                  // flop: a tracked CPU request (not a prefetch) is in R
    reg                r_pf_q;                                   // flop: a prefetch is in R
    wire               s1_lsu;                                   // a CPU request (or prefetch) is in S1
    wire               s2_lsu;                                   // ... in S2
    wire               s2_cpu;                                   // a tracked CPU request (not a prefetch) is in S2
    wire [PORT_W-1:0]  r_port;                                   // port of the op in R, PORT_W = 2 bits
    wire [PORT_W-1:0]  s1_port;                                  // port of the op in S1, 2 bits
    wire               r_wr;                                     // the op in R writes
    wire               s1_wr;                                    // the op in S1 writes
    wire               s1_pf;                                    // the op in S1 is a prefetch
    assign s1_lsu  = s1_valid && (s1_kind == K_LSU);
    assign s2_lsu  = s2_valid && (s2_kind == K_LSU);
    assign s2_cpu  = s2_lsu && !s2_req[REQ_PF_BIT];
    assign s1_pf   = s1_req[REQ_PF_BIT];
    assign r_port  = r_req[REQ_PORT_LSB +: PORT_W];
    assign s1_port = s1_req[REQ_PORT_LSB +: PORT_W];
    assign r_wr    = r_req[REQ_RDWR_BIT];
    assign s1_wr   = s1_req[REQ_RDWR_BIT];

    // ======================================================================
    // Arrivals: the lowest requesting port (address lane, 2 levels from the pins) and its fields
    // ======================================================================
    wire [LSU_PORTS-1:0] arrival_request;                        // per port, 4 bits
    wire [LSU_PORTS-1:0] lowest_req;                             // the lowest requesting port, one-hot, 4 bits
    wire                 any_pins_arrival;                       // some port requests
    assign arrival_request = {arrival_request_3, arrival_request_2, arrival_request_1, arrival_request_0};
    assign lowest_req[0]   = arrival_request[0];
    assign lowest_req[1]   = arrival_request[1] & ~arrival_request[0];
    assign lowest_req[2]   = arrival_request[2] & ~arrival_request[1] & ~arrival_request[0];
    assign lowest_req[3]   = arrival_request[3] & ~arrival_request[2] & ~arrival_request[1] & ~arrival_request[0];
    assign any_pins_arrival = |arrival_request;
    reg  [REQ_W-1:0]   fresh_packet;                             // the lowest requesting port's packet, 148 bits
    wire [LSU_PORTS*REQ_W-1:0] arrival_packet_flat;              // the 4 pins packets side by side, LSU_PORTS*REQ_W = 592 bits
    assign arrival_packet_flat = {arrival_packet_3, arrival_packet_2, arrival_packet_1, arrival_packet_0};
    integer fp_port;                                             // loop variable: port
    always @* begin
        fresh_packet = {REQ_W{1'b0}};
        for (fp_port = 0; fp_port < LSU_PORTS; fp_port = fp_port + 1)
            fresh_packet = fresh_packet | ({REQ_W{lowest_req[fp_port]}} & arrival_packet_flat[fp_port*REQ_W +: REQ_W]);
        // a fresh VECTOR STORE carries the number of the data place its line is written into this cycle (fix 2026-11-10,
        // session 59): the tracker puts it into the entry's packet, but the fresh issue takes the pins packet, so without
        // this the store read and released place 0 whatever place it had (lines lost, "release of free place" errors).
        // Only the VEC_SLOT_W data bits change, from the buffer's flops (vec_write_slot); not on the decision path.
        if (fresh_packet[REQ_LINE_BIT] && fresh_packet[REQ_RDWR_BIT])
            fresh_packet[REQ_DATA_LSB +: VEC_SLOT_W] = vec_write_slot;
    end
    wire [INDEX_W-1:0]     fresh_set;                            // its set, 6 bits (the address lane reads only these bits of the AND-OR)
    wire [LINE_ADDR_W-1:0] fresh_line;                           // its line address, 58 bits
    wire                   fresh_is_write;                       // it writes
    wire                   fresh_is_amo;                         // it is a one-request AMO
    assign fresh_set      = fresh_packet[SET_IN_REQ +: INDEX_W];
    assign fresh_line     = fresh_packet[LINE_IN_REQ +: LINE_ADDR_W];
    assign fresh_is_write = fresh_packet[REQ_RDWR_BIT];
    assign fresh_is_amo   = fresh_packet[REQ_AMO_BIT] & fresh_packet[REQ_SEQ_BIT];

    // ======================================================================
    // The request tracker
    // ======================================================================
    wire                   fresh_allowed_q;                      // flop (l1d_book_select): an arrival may issue fresh this cycle
    reg  [LSU_PORTS-1:0]   credit_q;                             // flop per port: a response credit for an issue this cycle, 4 bits
    wire                   fresh_issue;                          // the lowest arrival issues now
    wire [LSU_PORTS-1:0]   fresh_port_onehot;                    // its port, 4 bits
    wire [NE-1:0]          fresh_entry_onehot;                   // its tracker entry, 8 bits
    wire                   tracker_any_pins;                     // (unused copy of any_pins_arrival)
    wire                   oldest_ready_valid;                   // a READY entry without dependencies exists
    wire [NE-1:0]          oldest_ready_onehot;                  // the oldest of them, 8 bits
    wire [INDEX_W-1:0]     oldest_ready_set;                     // its set, 6 bits
    wire [LINE_ADDR_W-1:0] oldest_ready_line;                    // its line address, 58 bits
    wire [PORT_W-1:0]      oldest_ready_port;                    // its port, 2 bits
    wire                   oldest_ready_credit_ok;               // iter 8: its port has a booking credit next cycle (from the tracker)
    wire                   oldest_ready_is_write;                // it writes
    wire                   oldest_ready_is_line_op;              // whole-line VPU access
    wire                   oldest_ready_is_amo;                  // one-request AMO
    wire                   arrival_book_valid;                   // an arrival of this cycle may be booked
    wire [NE-1:0]          arrival_book_entry_onehot;            // its entry, 8 bits
    wire [INDEX_W-1:0]     arrival_book_set;                     // its set, 6 bits
    wire [LINE_ADDR_W-1:0] arrival_book_line;                    // its line address, 58 bits
    wire [PORT_W-1:0]      arrival_book_port;                    // its port, 2 bits
    wire                   arrival_book_is_write;                // it writes
    wire                   arrival_book_is_line_op;              // whole-line VPU access
    wire                   arrival_book_is_amo;                  // one-request AMO
    wire                   arrival_book_credit_ok;               // tm_opt iter 3: its port has a booking credit next cycle (from the tracker)
    // Vivado iter 5: the arrival fields already gated by the pick (zero unless the arrival is booked now)
    wire                   book_arrival_allowed;                 // CPU work may book an arrival this cycle (from l1d_book_select)
    wire                   arrival_book_go;                      // the arrival is booked now (from the tracker pick)
    wire [NE-1:0]          arrival_booked_entry_onehot;          // its entry, 8 bits
    wire [LINE_ADDR_W-1:0] arrival_booked_line;                  // its line address, 58 bits
    wire [PORT_W-1:0]      arrival_booked_port;                  // its port, 2 bits
    wire                   arrival_booked_is_write;              // it writes
    wire                   arrival_booked_is_amo;                // one-request AMO
    // tm_opt iter 3: declaration moved up (it is now also a tracker input); computed in the credit block below
    reg  [LSU_PORTS-1:0] credit_book_next;                       // credit for the booking (exact, two outcomes selected late), 4 bits
    wire [NE-1:0]          book_entry_onehot;                    // the entry booked this cycle (late), 8 bits
    wire [NE-1:0]          book_ready_onehot;                    // Vivado iter 1: its oldest-READY leg only (early), 8 bits
    wire [NE-1:0]          issue_entry_onehot;                   // the booked entry that issues now (flops), 8 bits
    wire [NE*REQ_W-1:0]    entry_packet_flat;                    // every entry's packet, 8*148 bits
    wire [NE*PC_W-1:0]     entry_pc_flat;                        // every entry's PC (unused here: the tracker gives s2_seat_pc), 8*64 bits
    wire [NE-1:0]          entry_valid_q;                        // entries in use, 8 bits

    l1d_req_tracker #(`L1D_PARAMS_PASS, .BANK_ID(BANK_ID)) u_tracker (
        .clk                       (clk),
        .rst                       (rst),
        .arrival_request_0         (arrival_request_0),
        .arrival_request_1         (arrival_request_1),
        .arrival_request_2         (arrival_request_2),
        .arrival_request_3         (arrival_request_3),
        .arrival_packet_0          (arrival_packet_0),
        .arrival_packet_1          (arrival_packet_1),
        .arrival_packet_2          (arrival_packet_2),
        .arrival_packet_3          (arrival_packet_3),
        .arrival_pc_0              (arrival_pc_0),
        .arrival_pc_1              (arrival_pc_1),
        .arrival_pc_2              (arrival_pc_2),
        .arrival_pc_3              (arrival_pc_3),
        .drain_request             (drain_request),
        .drain_packet              (drain_packet),
        .drain_pc                  (drain_pc),
        .vec_slot_available        (vec_slot_available),
        .vec_write_slot            (vec_write_slot),
        .vec_write_valid           (vec_write_valid),
        .fresh_allowed_q           (fresh_allowed_q),
        .credit_q_0                (credit_q[0]),
        .credit_q_1                (credit_q[1]),
        .credit_q_2                (credit_q[2]),
        .credit_q_3                (credit_q[3]),
        .credit_book_next          (credit_book_next),
        .fresh_issue               (fresh_issue),
        .fresh_port_onehot         (fresh_port_onehot),
        .fresh_entry_onehot        (fresh_entry_onehot),
        .any_pins_arrival          (tracker_any_pins),
        .arrival_allocated_0       (arrival_allocated_0),
        .arrival_allocated_1       (arrival_allocated_1),
        .arrival_allocated_2       (arrival_allocated_2),
        .arrival_allocated_3       (arrival_allocated_3),
        .drain_allocated           (drain_allocated),
        .oldest_ready_valid        (oldest_ready_valid),
        .oldest_ready_onehot       (oldest_ready_onehot),
        .oldest_ready_set          (oldest_ready_set),
        .oldest_ready_line         (oldest_ready_line),
        .oldest_ready_port         (oldest_ready_port),
        .oldest_ready_credit_ok    (oldest_ready_credit_ok),
        .oldest_ready_is_write     (oldest_ready_is_write),
        .oldest_ready_is_line_op   (oldest_ready_is_line_op),
        .oldest_ready_is_amo       (oldest_ready_is_amo),
        .arrival_book_valid        (arrival_book_valid),
        .arrival_book_entry_onehot (arrival_book_entry_onehot),
        .arrival_book_set          (arrival_book_set),
        .arrival_book_line         (arrival_book_line),
        .arrival_book_port         (arrival_book_port),
        .arrival_book_is_write     (arrival_book_is_write),
        .arrival_book_is_line_op   (arrival_book_is_line_op),
        .arrival_book_is_amo       (arrival_book_is_amo),
        .arrival_book_credit_ok    (arrival_book_credit_ok),
        .arrival_book_allowed      (book_arrival_allowed),
        .arrival_book_go           (arrival_book_go),
        .arrival_booked_entry_onehot (arrival_booked_entry_onehot),
        .arrival_booked_line       (arrival_booked_line),
        .arrival_booked_port       (arrival_booked_port),
        .arrival_booked_is_write   (arrival_booked_is_write),
        .arrival_booked_is_amo     (arrival_booked_is_amo),
        .book_entry_onehot         (book_entry_onehot),
        .book_ready_onehot         (book_ready_onehot),  // oldest-READY booking leg: moves a booked in-use entry from READY to BOOKED, NE = 8 bits
        .issue_entry_onehot        (issue_entry_onehot),
        .entry_packet_flat         (entry_packet_flat),
        .entry_pc_flat             (entry_pc_flat),
        .s2_cpu_valid              (s2_cpu),
        .s2_seat                   (s2_seat),
        .s2_kill                   (s2_kill),
        .s2_answered               (s2_answered),
        .act_valid                 (act_valid),
        .act_code                  (act_code),
        .act_id                    (act_id),
        .s2_train_sent             (s2_train_sent),
        .s2_seat_trained           (s2_seat_trained),
        .s2_seat_pc                (s2_seat_pc),
        .mshr_freed                (mshr_freed),
        .vc_freed                  (vc_freed),
        .nc_freed                  (nc_freed),
        .free_count_q              (tracker_free_count_q),
        .entry_valid_q             (entry_valid_q),
        .tracker_empty             (tracker_empty),
        .r_seat                    (r_seat),
        .r_seat_deps               (r_deps)
    );

    // ======================================================================
    // Response credits (PLAN_v6 4.3). Per port p and this bank's FIFO in port p's merger:
    //   exact credit for an issue in cycle c, computed in c-1 into credit_q[p]:  free_next(p) >= promised(p) + 2
    //     promised(p) = (the op issued in c-1 is of port p) + (the op in R in c-1 is of port p) [it is in S1 in c]
    //     (+1 for the issue itself, +1 for an answer that may be pushed in c)
    //   credit for the BOOKING in c-1 of an entry of port p (issue in c): the same rule, built as two outcomes from flops
    //     (no issue of port p in c-1: free_next(p) >= r_is_p + 2; an issue: >= r_is_p + 3), the late issue bit of c-1
    //     selects (one 2:1 level at the end of the booking path). The earlier form always assumed the issue (+3) and
    //     deadlocked with a 2-deep response FIFO (fixed 2026-11-10, session 59).
    //   miss_ok (MSHR / NC answer pushed now): free(p) > R_p + S1_p for every port, as before (from flops, used in S3).
    // Example: a back-to-back stream on port 0, free stays 4 (push and pop in every cycle), R = S1 = 1: 4 >= 1 + 1 + 2.
    // CROSSBANK STEP 1 (2026-11-25): the rule is now free_now >= 2 + r_is + issue + own_push (free_now = the merger FIFO's flop, own_push =
    //   this bank's answer for the port this cycle). Same stream with RESP_FIFO_DEPTH = 5: free stays 5 (answer bypassed), 5 >= 1 + 1 + 1 + 2.
    //   With depth 4 the stream would be throttled (4 < 5), hence the depth 5 in l1d_defines.vh.
    // ======================================================================
    wire [LSU_PORTS*RCNT_W-1:0] free_now_flat;                   // free places now, 4*3 bits
    wire [LSU_PORTS*RCNT_W-1:0] free_next_flat;                  // free places after this cycle, 4*3 bits (UNUSED since crossbank step 1, kept for the port map)
    assign free_now_flat  = {lsu_response_fifo_free_3, lsu_response_fifo_free_2, lsu_response_fifo_free_1, lsu_response_fifo_free_0};
    assign free_next_flat = {lsu_response_fifo_free_next_3, lsu_response_fifo_free_next_2, lsu_response_fifo_free_next_1, lsu_response_fifo_free_next_0};
    wire                 book_entry_issue;                       // the booked tracker entry issues now (flops)
    reg  [PORT_W-1:0]    book_port_q;                            // flop: port of the booked entry, 2 bits
    reg                  book_is_write_q;                        // flop: the booked entry writes
    reg                  book_is_amo_q;                          // flop: the booked entry is a one-request AMO
    reg  [LSU_PORTS-1:0] r_is_port;                              // the tracked op in R is of port p, 4 bits
    reg  [LSU_PORTS-1:0] s1_is_port;                             // the tracked op in S1 is of port p, 4 bits
    reg  [LSU_PORTS-1:0] issue_is_port;                          // the op issued now is of port p (late), 4 bits
    reg  [LSU_PORTS-1:0] credit_next;                            // exact credit for the next cycle, 4 bits
    reg  [LSU_PORTS-1:0] credit_book_if_none;                    // booking credit if no op of port p issues now, 4 bits
    reg  [LSU_PORTS-1:0] credit_book_if_issue;                   // booking credit if an op of port p issues now, 4 bits
    reg  [LSU_PORTS-1:0] port_miss_ok;                           // an MSHR / NC answer may be pushed for port p, 4 bits
    reg  [RCNT_W:0]      free_now_ext;                           // scratch (crossbank step 1, replaces promised_next): free places now of port p, 4 bits
    reg                  credit_ge_one;                          // scratch: free_now >= 1 (CB-1)
    reg                  credit_ge_two;                          // scratch: free_now >= 2
    reg                  credit_ge_three;                        // scratch: free_now >= 3
    reg                  credit_ge_four;                         // scratch: free_now >= 4
    reg                  credit_ge_five;                         // scratch: free_now >= 5
    integer cr_port;                                             // loop variable: port
    always @* begin
        for (cr_port = 0; cr_port < LSU_PORTS; cr_port = cr_port + 1) begin
            r_is_port[cr_port]     = r_cpu_q && (r_port == cr_port);
            s1_is_port[cr_port]    = s1_lsu && !s1_pf && (s1_port == cr_port);
            issue_is_port[cr_port] = (book_entry_issue && (book_port_q == cr_port)) || (fresh_issue && fresh_port_onehot[cr_port]);
            // CROSSBANK STEP 1: the credit reads free_now (a flop of the merger FIFO), never free_next (cross-bank, LSU read pin).
            // free_next >= free_now - own_push always holds (only this bank pushes into its FIFO; a pop or bypass only adds places),
            // so the rule "free_next >= promised + 2" becomes "free_now >= promised + 2 + own_push": safe, never above the exact value.
            // promised_next (an adder) is gone: the same value is selected from four compares of free_now with constants.
            free_now_ext           = {1'b0, free_now_flat[cr_port*RCNT_W +: RCNT_W]};   // free places now, 4 bits
            credit_ge_two          = (free_now_ext >= CREDIT_TWO);
            credit_ge_three        = (free_now_ext >= CREDIT_THREE);
            credit_ge_four         = (free_now_ext >= CREDIT_FOUR);
            credit_ge_five         = (free_now_ext >= CREDIT_FIVE);
            // exact credit = booking credit rule (see below): free_now >= 2 + r_is + own_push + issue
            credit_next[cr_port]   = issue_is_port[cr_port]
                                   ? (own_response_push_port[cr_port] ? (r_is_port[cr_port] ? credit_ge_five  : credit_ge_four)
                                                                      : (r_is_port[cr_port] ? credit_ge_four  : credit_ge_three))
                                   : (own_response_push_port[cr_port] ? (r_is_port[cr_port] ? credit_ge_four  : credit_ge_three)
                                                                      : (r_is_port[cr_port] ? credit_ge_three : credit_ge_two));
            // booking credit (fix 2026-11-10, session 59): BOTH outcomes from flops, the late issue bit selects at the
            // end (one 2:1 level). The old form assumed an issue of the same port (free_next >= r_is + 3): with a 2-deep
            // response FIFO (tests with RESP_FIFO_DEPTH = 2) it could never be met and a waiting entry was never booked
            // again (deadlock seen in tb_l1d_coh -d TINY). The value now equals credit_next (the exact rule).
            // Vivado/ASAP7 iter 6 (C1): no adder on r_is_port. x >= r_is + c is the same as r_is ? (x > c) : (x >= c); the
            // late r_is_port (R-stage port decode) only selects between two compares of the early free count with constants.
            // CROSSBANK STEP 1: free_now and the own push of this cycle replace free_next; the own push is the outer select (late),
            // r_is_port the inner one: threshold = 2 (+1 own push) (+1 r_is_port) (+1 issue in the second form).
            credit_book_if_none[cr_port]  = own_response_push_port[cr_port] ? (r_is_port[cr_port] ? credit_ge_four  : credit_ge_three)
                                                                            : (r_is_port[cr_port] ? credit_ge_three : credit_ge_two);
            credit_book_if_issue[cr_port] = own_response_push_port[cr_port] ? (r_is_port[cr_port] ? credit_ge_five  : credit_ge_four)
                                                                            : (r_is_port[cr_port] ? credit_ge_four  : credit_ge_three);
            credit_book_next[cr_port] = issue_is_port[cr_port] ? credit_book_if_issue[cr_port] : credit_book_if_none[cr_port];
            // TIMING OPT (CB-1, 2026-11-26): free > r_is + s1_is without the adder. r_is + s1_is is 0, 1 or 2, so the answer is one of three
            // compares of the early free count (a merger flop) with constants; the late S1 port decode (s1_valid ... == port) is the LAST select,
            // the R decode the one before. free > 0 is the same as free >= 1 (credit_ge_one), free > 1 = credit_ge_two, free > 2 = credit_ge_three.
            // Example: free = 2, R is of port p, S1 is not: free > 1 is true -> port_miss_ok[p] = 1.
            credit_ge_one          = (free_now_ext >= CREDIT_ONE);
            port_miss_ok[cr_port]  = s1_is_port[cr_port] ? (r_is_port[cr_port] ? credit_ge_three : credit_ge_two)
                                                         : (r_is_port[cr_port] ? credit_ge_two   : credit_ge_one);
        end
    end
    assign miss_ok = &port_miss_ok;
    always @(posedge clk) begin : credit_proc
        if (rst) credit_q <= {LSU_PORTS{1'b1}};                  // empty FIFOs, empty pipeline
        else     credit_q <= credit_next;
    end // always credit_proc
    // the credit of the booking candidates' ports (AND-OR of the booking credits by the port number)
    wire entry_credit_ok;                                        // the oldest READY entry's port has a credit next cycle
    wire arrival_credit_ok;                                      // the arrival candidate's port has a credit next cycle
    // iter 8 (E1, under assumption A1 of the proof flow): the tracker looks up the credit of every entry's port from its flops and
    // picks it by the age one-hot, so the 8:1 port select and the 4:1 credit select leave the path. Equal to
    // credit_book_next[oldest_ready_port] whenever an entry is ready and the age pick is one-hot (l1d_book_select reads it only
    // together with entry_ready_valid).
    assign entry_credit_ok   = oldest_ready_credit_ok;
    // tm_opt iter 3: the tracker looks up the credit of every source port early and ORs it by the pick, so the 4:1 select
    // by the late arrival_book_port leaves the path. Was credit_book_next[arrival_book_port]: equal whenever
    // arrival_book_valid = 1, and l1d_book_select reads arrival_credit_ok only together with arrival_ready_valid.
    assign arrival_credit_ok = arrival_book_credit_ok;

    // ======================================================================
    // Snoop buffer (unchanged l1d_snoop_buf: 2 entries, served oldest first per line) and the snoop credit
    // ======================================================================
    wire                   sn_sel_valid;                         // a snoop is offered
    wire                   sn_sel_inv;                           // ... it invalidates
    wire [SNP_ID_W-1:0]    sn_sel_id;                            // its buffer entry, 1 bit
    wire [LINE_ADDR_W-1:0] sn_sel_line;                          // its line address, 58 bits
    wire                   gnt_snoop;                            // the booked snoop issues now (the buffer marks it in flight)
    wire                   snp_credit;                           // a free place in the snoop answer FIFO
    wire                   r_amor_any;                           // an AMO read half is in R (never in this version)
    wire                   s1_amor_any;                          // ... in S1
    wire                   s2_amor_any;                          // ... in S2
    wire                   amor_busy;                            // an AMO read is in the pipeline
    assign snp_credit  = (snoop_response_fifo_free != {SNPR_CNT_W{1'b0}});
    assign r_amor_any  = r_cpu_q && r_req[REQ_AMO_BIT]  && r_req[REQ_SEQ_BIT]  && !r_req[REQ_RDWR_BIT];
    assign s1_amor_any = s1_lsu  && s1_req[REQ_AMO_BIT] && s1_req[REQ_SEQ_BIT] && !s1_req[REQ_RDWR_BIT];
    assign s2_amor_any = s2_lsu  && s2_req[REQ_AMO_BIT] && s2_req[REQ_SEQ_BIT] && !s2_req[REQ_RDWR_BIT];
    assign amor_busy   = r_amor_any || s1_amor_any || s2_amor_any;
    wire amo_hold_snoops;                                        // hold snoops: an AMO read in the pipeline or a line handover
    assign amo_hold_snoops = amor_busy || refill_handover_busy;
    l1d_snoop_buf #(`L1D_PARAMS_PASS) u_snp (
        .clk                  (clk),
        .rst                  (rst),
        .new_snoop_valid      (snoop_request_valid),
        .new_snoop_line_addr  (snoop_request_line_addr),
        .new_snoop_invalidate (snoop_request_invalidate),
        .new_snoop_ready      (snoop_request_ready),
        .amo_lock_snoop_hold  (lock_snp),
        .amo_lock_line        (lock_line),
        .amo_read_in_pipeline (amo_hold_snoops),
        .amo_lock_set_valid   (ev_amor),
        .amo_lock_set_line    (s2_line),
        .offer_valid          (sn_sel_valid),
        .offer_entry_id       (sn_sel_id),
        .offer_line_addr      (sn_sel_line),
        .offer_invalidate     (sn_sel_inv),
        .offer_granted        (gnt_snoop),
        .s2_action_valid      (snp_act_valid),
        .s2_action_entry_id   (s2_sid),
        .s2_action_code       (snp_act_code),
        .s2_action_wait_id    (snp_act_id),
        .mshr_entry_freed     (mshr_freed),
        .victim_slot_freed    (vc_freed),
        .buffer_empty         ()
    );

    // ======================================================================
    // Prefetch register: one hint per bank, the newest replaces the waiting one, except while the hint is booked (frozen)
    // ======================================================================
    reg                    pf_valid_q;                           // a hint waits
    reg  [LINE_ADDR_W-1:0] pf_line_q;                            // its line address, 58 bits
    reg                    pf_unique_q;                          // it wants the line unique
    reg                    pf_booked_q;                          // it is booked: do not replace it
    wire                   pf_issue;                             // the booked hint issues now
    wire                   pf_take_new;                          // a new hint replaces the waiting one
    assign pf_take_new            = prefetch_request_valid && !pf_booked_q && (HAS_PF != 0);
    assign prefetch_request_ready = !pf_booked_q;
    wire [REQ_W-1:0]       pf_packet;                            // the hint as a request packet (a tagged load of line pf_line_q), 148 bits
    assign pf_packet = {pf_unique_q, 1'b1, 1'b0, {PORT_W{1'b0}}, {AMOOP_W{1'b0}}, 1'b0, 1'b0, 1'b0, 1'b1,
                        {MEM_SIZE_BITS{1'b0}}, {MASTER_ID_BITS{1'b0}}, {IDX_BITS{1'b0}}, {pf_line_q, OFF0}, {WRITE_DATA_WIDTH{1'b0}}};

    // ======================================================================
    // The booking (l1d_book_select) and the fields it captures
    // ======================================================================
    wire book_valid_q;                                           // flop: something is booked for this cycle
    wire [BOOK_KIND_W-1:0] book_kind_q;                          // flop: what, one-hot, 5 bits
    wire book_now_snoop, book_now_refill, book_now_maint;        // this cycle's booking decision (combinational)
    wire book_now_entry, book_now_prefetch;
    // Vivado iter 3: the max_fanout copy of iter 2 is removed (it put one more LUT in series, iter 2 routed report)
    wire book_now_arrival;                                       // this cycle books the arrival (late)
    wire book_now_prefetch_no_arrival;                           // Vivado iter 2: prefetch booked unless an arrival is (early)
    // the two late gates about the next cycle
    wire amo_issued_now;                                         // an AMO enters R next cycle
    wire store_in_flight_next;                                   // a store-class op in R / S1 / S2 next cycle, or an AMO in S3
    wire s2_amo_served;                                          // the AMO hit in S2 writes in S3 next cycle
    assign amo_issued_now       = (book_entry_issue && book_is_amo_q) || (fresh_issue && fresh_is_amo);
    assign s2_amo_served        = s2_cpu && s2_hit && !s2_kill && s2_req[REQ_AMO_BIT] && s2_req[REQ_SEQ_BIT];
    wire s2_amo_in_s2;                                           // an AMO op is in S2 (flops only, hit or not): it may write in S3 next cycle
    assign s2_amo_in_s2         = s2_cpu && s2_req[REQ_AMO_BIT] && s2_req[REQ_SEQ_BIT];
    assign store_in_flight_next = (book_entry_issue && book_is_write_q) || (fresh_issue && fresh_is_write) ||
                                  (r_cpu_q && r_wr) || (s1_lsu && !s1_pf && s1_wr) || s2_amo_served;
    l1d_book_select #(.REFILL_SKIP(REFILL_SKIP + 1), .SNP_STARVE(SNP_STARVE), .BOOK_KIND_W(BOOK_KIND_W)) u_book (
        .clk                  (clk),
        .rst                  (rst),
        .snoop_offer_valid    (sn_sel_valid),
        .snoop_credit_ok      (snp_credit),
        .refill_token_valid   (tok_valid),
        .maint_token_valid    (mt_valid),
        .entry_ready_valid    (oldest_ready_valid),
        .entry_credit_ok      (entry_credit_ok),
        .arrival_ready_valid  (arrival_book_valid),
        .arrival_credit_ok    (arrival_credit_ok),
        .prefetch_valid       (pf_valid_q && !pf_booked_q),   // BUGS.md B-1: not offered again while booked, so a hint issues once
        .amo_issued_now       (amo_issued_now),
        .store_in_flight_next (store_in_flight_next),
        .amo_write_next       (s2_amo_in_s2),
        .book_valid_q         (book_valid_q),
        .book_kind_q          (book_kind_q),
        .fresh_allowed_q      (fresh_allowed_q),
        .book_now_snoop       (book_now_snoop),
        .book_now_refill      (book_now_refill),
        .book_now_maint       (book_now_maint),
        .book_now_entry       (book_now_entry),
        .book_now_arrival     (book_now_arrival),
        .book_now_prefetch    (book_now_prefetch),
        .book_now_prefetch_no_arrival (book_now_prefetch_no_arrival),
        .book_arrival_allowed         (book_arrival_allowed)
    );
    assign book_ready_onehot = {NE{book_now_entry}} & oldest_ready_onehot;   // early leg, for the in-use entry states
    // Vivado iter 5: the arrival leg comes already gated by the tracker pick (was {NE{book_now_arrival}} & arrival_book_entry_onehot)
    assign book_entry_onehot = book_ready_onehot | arrival_booked_entry_onehot;
    // the fields of the booked operation, captured at the booking edge (the address lane reads book_set_q, a flop)
    wire [LINE_ADDR_W-1:0] mt_line;                              // line address of the flush/clean token: {zeros, set, bank}, 58 bits
    assign mt_line = {{TAG_W{1'b0}}, mt_set, BANK_C};
    reg  [LINE_ADDR_W-1:0] book_line_q;                          // flop: line address of the booked op, 58 bits
    reg  [INDEX_W-1:0]     book_set_q;                           // flop: its set (its own flops: the address lane), 6 bits
    reg  [NE-1:0]          book_entry_onehot_q;                  // flop: the booked tracker entry, one-hot, 8 bits
    reg  [NE-1:0]          book_entry_sel_q [0:3];               // flops: 4 copies of it for the 148-bit packet AND-OR (rule T-6), 4 x 8 bits
    reg  [SNP_ID_W-1:0]    book_sid_q;                           // flop: booked snoop entry, 1 bit
    reg                    book_sinv_q;                          // flop: booked snoop invalidates
    reg  [MSHR_ID_W-1:0]   book_mid_q;                           // flop: booked refill token's MSHR entry, 3 bits
    reg  [WAY_W-1:0]       book_mway_q;                          // flop: booked flush/clean token's way, 2 bits
    reg                    book_mop_q;                           // flop: booked flush/clean token's operation
    reg  [LINE_ADDR_W-1:0] book_line_next;                       // the line address being booked, 58 bits
    reg  [PORT_W-1:0]      book_port_next;                       // the port being booked, 2 bits
    reg                    book_is_write_next;                   // the booked op writes
    reg                    book_is_amo_next;                     // the booked op is an AMO
    // Vivado iter 3: the fields when NO arrival is booked (early terms only), as separate wires; the late book_now_arrival
    // selects between them and the arrival fields (no keep attribute, user decision 2026-11-17)
    wire [LINE_ADDR_W-1:0] book_line_no_arrival;   // line address, 58 bits
    wire [PORT_W-1:0]      book_port_no_arrival;   // port, 2 bits
    wire                   book_is_write_no_arrival; // writes
    wire                   book_is_amo_no_arrival;   // one-request AMO
    assign book_line_no_arrival     = ({LINE_ADDR_W{book_now_snoop}}               & sn_sel_line) |
                                      ({LINE_ADDR_W{book_now_refill}}              & tok_line) |
                                      ({LINE_ADDR_W{book_now_maint}}               & mt_line) |
                                      ({LINE_ADDR_W{book_now_prefetch_no_arrival}} & pf_line_q) |
                                      ({LINE_ADDR_W{book_now_entry}}               & oldest_ready_line);
    assign book_port_no_arrival     = {PORT_W{book_now_entry}} & oldest_ready_port;
    assign book_is_write_no_arrival = book_now_entry & oldest_ready_is_write;
    assign book_is_amo_no_arrival   = book_now_entry & oldest_ready_is_amo;
    always @* begin
        // a select by the one-hot decision (at most one book_now_* is set: l1d_book_select makes them exclude each other
        // for every input). tm_opt iter 1: one AND-OR instead of the last-wins if-chain (one mux level per source); the
        // late arrival leg (tracker arrival_book_* and book_now_arrival) enters only the last OR.
        // Vivado iter 2: carry-select on book_now_arrival. When the arrival is booked nothing else is (one-hot), so the field
        // is the arrival's; when it is not, the prefetch leg uses the early book_now_prefetch_no_arrival. The late
        // book_now_arrival is the select of the last mux (was: it also gated book_now_prefetch, two LUTs more, iter 1 report).
        // Vivado iter 3: the late select only (the other leg is the book_*_no_arrival above)
        // Vivado iter 5: the arrival fields are zero unless booked (tracker pick select), so the late select becomes an OR.
        // When the arrival is booked, snoop / refill / maint / entry are all off (cpu_go, not entry), so only the prefetch term
        // of book_line_no_arrival can be set: it is cut by arrival_book_go. Port, write and AMO have no prefetch term.
        book_line_next     = arrival_booked_line     | (book_line_no_arrival & ~{LINE_ADDR_W{arrival_book_go}});
        book_port_next     = arrival_booked_port     | book_port_no_arrival;
        book_is_write_next = arrival_booked_is_write | book_is_write_no_arrival;
        book_is_amo_next   = arrival_booked_is_amo   | book_is_amo_no_arrival;
    end
    integer cp_idx;                                              // loop variable: copy of the entry select
    always @(posedge clk) begin : book_fields_proc
        if (rst) begin
            book_entry_onehot_q <= {NE{1'b0}};
            for (cp_idx = 0; cp_idx < 4; cp_idx = cp_idx + 1) book_entry_sel_q[cp_idx] <= {NE{1'b0}};
            pf_booked_q <= 1'b0;
        end else begin
            book_line_q         <= book_line_next;
            book_set_q          <= book_line_next[LA_SET_LSB +: INDEX_W];
            book_port_q         <= book_port_next;
            book_is_write_q     <= book_is_write_next;
            book_is_amo_q       <= book_is_amo_next;
            book_entry_onehot_q <= book_entry_onehot;
            for (cp_idx = 0; cp_idx < 4; cp_idx = cp_idx + 1) book_entry_sel_q[cp_idx] <= book_entry_onehot;
            book_sid_q          <= sn_sel_id;
            book_sinv_q         <= sn_sel_inv;
            book_mid_q          <= tok_id;
            book_mway_q         <= mt_way;
            book_mop_q          <= mt_op;
            pf_booked_q         <= book_now_prefetch;
        end
    end // always book_fields_proc

    // ======================================================================
    // Cycle c: does the booked operation issue? (a source whose offer changed since the booking cancels: a bubble, rebooked)
    // ======================================================================
    wire snoop_issue;                                            // the booked snoop issues (the buffer still offers that entry)
    wire refill_issue;                                           // the booked refill token issues (the MSHR still offers that entry)
    wire maint_issue;                                            // the booked flush/clean token issues
    wire book_issue;                                             // some booked operation issues
    assign snoop_issue      = book_valid_q && book_kind_q[BOOK_SNOOP_B]  && sn_sel_valid && (sn_sel_id == book_sid_q);
    assign refill_issue     = book_valid_q && book_kind_q[BOOK_REFILL_B] && tok_valid && (tok_id == book_mid_q);
    assign maint_issue      = book_valid_q && book_kind_q[BOOK_MAINT_B]  && mt_valid;
    assign book_entry_issue = book_valid_q && book_kind_q[BOOK_ENTRY_B];
    assign pf_issue         = book_valid_q && book_kind_q[BOOK_PF_B];
    assign book_issue       = snoop_issue || refill_issue || maint_issue || book_entry_issue || pf_issue;
    assign gnt_snoop          = snoop_issue;
    assign gnt_refill         = refill_issue;
    assign gnt_maint          = maint_issue;
    assign issue_entry_onehot = {NE{book_entry_issue}} & book_entry_onehot_q;

    // ---- the prefetch register ----
    always @(posedge clk) begin : pf_proc
        if (rst) begin
            pf_valid_q <= 1'b0;
        end else begin
            if (pf_take_new)      pf_valid_q <= 1'b1;
            else if (pf_issue)    pf_valid_q <= 1'b0;
            if (pf_take_new) begin
                pf_line_q   <= prefetch_request_line_addr;
                pf_unique_q <= prefetch_request_unique;
            end
        end
    end // always pf_proc

    // ======================================================================
    // ADDRESS LANE: the set the SRAMs read this cycle (one 2:1 by a flop, rule T-3), and the read enable
    // ======================================================================
    assign g_set   = book_valid_q ? book_set_q : fresh_set;
    assign any_gnt = book_valid_q || any_pins_arrival;           // a read that is not used costs power only (invariant A-1)

    // ======================================================================
    // G -> R registers (the Stage R registers read by l1d_bank_r)
    // ======================================================================
    // the booked entry's packet: AND-OR of the 8 entries by the one-hot flop (4 copies of the select, one per 37-bit group)
    reg  [REQ_W-1:0] booked_entry_packet;                        // 148 bits
    integer be_entry;                                            // loop variable: entry
    integer be_group;                                            // loop variable: bit group of the packet
    localparam GROUP_W = (REQ_W + 3) / 4;                        // bits per select copy, 37
    always @* begin
        booked_entry_packet = {REQ_W{1'b0}};
        for (be_entry = 0; be_entry < NE; be_entry = be_entry + 1)
            for (be_group = 0; be_group < 4; be_group = be_group + 1)
                if (be_group * GROUP_W < REQ_W)
                    booked_entry_packet[be_group*GROUP_W +: GROUP_W] = booked_entry_packet[be_group*GROUP_W +: GROUP_W] |
                        ({GROUP_W{book_entry_sel_q[be_group][be_entry]}} & entry_packet_flat[be_entry*REQ_W + be_group*GROUP_W +: GROUP_W]);
    end
    wire [REQ_W-1:0] booked_packet;                              // the packet of the booked op (entry or prefetch), 148 bits
    assign booked_packet = book_kind_q[BOOK_PF_B] ? pf_packet : booked_entry_packet;
    wire [1:0] booked_kind;                                      // K_* of the booked op
    assign booked_kind = book_kind_q[BOOK_SNOOP_B]  ? K_SNOOP :
                         book_kind_q[BOOK_REFILL_B] ? K_REFILL :
                         book_kind_q[BOOK_MAINT_B]  ? K_MAINT : K_LSU;
    // entry numbers from the one-hot vectors (OR-encode: the vectors are one-hot or zero)
    reg  [SEAT_W-1:0] booked_seat;                               // the booked entry's number, 3 bits
    reg  [SEAT_W-1:0] fresh_seat;                                // the fresh entry's number, 3 bits
    integer en_idx;                                              // loop variable: entry
    always @* begin
        booked_seat = {SEAT_W{1'b0}};
        fresh_seat  = {SEAT_W{1'b0}};
        for (en_idx = 0; en_idx < NE; en_idx = en_idx + 1) begin
            if (book_entry_onehot_q[en_idx]) booked_seat = booked_seat | en_idx[SEAT_W-1:0];
            if (fresh_entry_onehot[en_idx])  fresh_seat  = fresh_seat  | en_idx[SEAT_W-1:0];
        end
    end
    always @(posedge clk) begin : g_to_r_proc
        if (rst) begin
            r_valid <= 1'b0;
            r_cpu_q <= 1'b0;
            r_pf_q  <= 1'b0;
            r_fast  <= 1'b0;
        end else begin
            r_valid <= book_issue || fresh_issue;
            r_cpu_q <= book_entry_issue || fresh_issue;
            r_pf_q  <= pf_issue;
            r_fast  <= fresh_issue;
            // the wide fields load every cycle from the booked op or the arrival (no late enable, rule SS-6); r_valid qualifies them
            r_kind  <= book_valid_q ? booked_kind : K_LSU;
            r_req   <= book_valid_q ? booked_packet : fresh_packet;
            r_line  <= book_valid_q ? book_line_q : fresh_line;
            r_seat  <= book_valid_q ? booked_seat : fresh_seat;
            r_mid   <= book_mid_q;
            r_mway  <= book_mway_q;
            r_mop   <= book_mop_q;
            r_sinv  <= book_sinv_q;
            r_sid   <= book_sid_q;
        end
    end // always g_to_r_proc

    // ======================================================================
    // SIMULATION CHECKS (always on): the address lane invariant and the bookkeeping of the booking
    // ======================================================================
`ifndef SYNTHESIS
    reg [INDEX_W-1:0] g_set_q;                                   // the set the SRAMs read last cycle (check only), 6 bits
    always @(posedge clk) begin
        g_set_q <= g_set;
        if (!rst) begin
            if (r_valid && (r_line[LA_SET_LSB +: INDEX_W] != g_set_q))
                $display("[%0t] L1D B%0d S0 ADDR LANE ERROR: the op in R has set %0d but the SRAMs read set %0d", $time, BANK_ID,
                         r_line[LA_SET_LSB +: INDEX_W], g_set_q);
            if (fresh_issue && book_valid_q)
                $display("[%0t] L1D B%0d S0 ERROR: fresh issue while an operation is booked", $time, BANK_ID);
            if (book_valid_q && !book_issue)
                $display("[%0t] L1D B%0d S0 NOTE: booked %b cancelled (source offer changed): bubble", $time, BANK_ID, book_kind_q);
        end
    end
`endif
`ifdef L1D_DEBUG
    always @(posedge clk) begin
        if (!rst) begin
            if (snoop_issue)
                $display("[%0t] L1D B%0d S0 ISSUE snoop %s line=%h", $time, BANK_ID, book_sinv_q ? "INV" : "READ", {book_line_q, OFF0});
            else if (refill_issue)
                $display("[%0t] L1D B%0d S0 ISSUE refill token  mshr=%0d line=%h", $time, BANK_ID, book_mid_q, {book_line_q, OFF0});
            else if (maint_issue)
                $display("[%0t] L1D B%0d S0 ISSUE %s token  set=%0d way=%0d", $time, BANK_ID,
                         (book_mop_q == MAINT_FLUSH) ? "flush" : "clean", book_set_q, book_mway_q);
            else if (book_entry_issue)
                $display("[%0t] L1D B%0d S0 ISSUE BOOKED entry %0d port %0d idx=%0d %s addr=%h", $time, BANK_ID, booked_seat,
                         booked_entry_packet[REQ_PORT_LSB +: PORT_W], booked_entry_packet[REQ_IDX_LSB +: IDX_BITS],
                         (booked_entry_packet[REQ_RDWR_BIT] ? "ST" : "LD"), booked_entry_packet[REQ_ADDR_LSB +: ADDR_W]);
            else if (pf_issue)
                $display("[%0t] L1D B%0d S0 ISSUE PREFETCH line=%h unique=%b", $time, BANK_ID, {pf_line_q, OFF0}, pf_unique_q);
            else if (fresh_issue)
                $display("[%0t] L1D B%0d S0 ISSUE FRESH entry %0d port %0d idx=%0d %s addr=%h", $time, BANK_ID, fresh_seat,
                         fresh_packet[REQ_PORT_LSB +: PORT_W], fresh_packet[REQ_IDX_LSB +: IDX_BITS],
                         (fresh_packet[REQ_RDWR_BIT] ? "ST" : "LD"), fresh_packet[REQ_ADDR_LSB +: ADDR_W]);
        end
    end
`endif

endmodule

`default_nettype wire
`endif // L1D_BANK_S0_V
