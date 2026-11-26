// ============================================================================
// l1d_req_tracker.v  -  the request tracker of ONE bank: 8 entries that hold every CPU request from arrival to completion
// ============================================================================
// IN SIMPLE WORDS  (cache_v6, PLAN_v6.md section 3)
//   Picture 8 chairs in front of one shop counter (the bank). Every customer (a CPU request from any of the 4 ports) gets a
//   chair the moment he arrives, even when four arrive in the same cycle (one per port): they all sit down at once, lower port
//   first, and the FIRST of them is served at once (his SRAM read starts in that same cycle: the 4-cycle hit). The others are
//   served one per cycle, oldest first. A customer keeps his chair until he is done (answer sent, or handed to an MSHR / NC
//   entry that will answer him), so the counter always knows who is in the shop. If all 8 chairs are taken, the newcomers
//   wait outside in their port's skid (l1d_port_front) and come in, in arrival order, when a chair is free.
//
//   ORDER RULE (the old in-flight tracker, rule R4 of the lean S0, now as bits): when a customer sits down, one look at the
//   older customers decides whom he must wait for: those on the SAME SET when he or they WRITE (store, SC, AMO, vector store).
//   That look is a 6-bit set compare per older entry, done ONCE at arrival (rule T-2); the result is kept as dependency bits
//   that are cleared when the older entry leaves. The pick of who goes next (the booking, l1d_book_select) reads flops only.
//
//   A fresh customer (served in his arrival cycle) is served ON TRUST, before his dependency bits are known: stage R reads
//   them (l1d_bank_r) and cancels him if any is set; he runs on as a ghost and his entry goes back to READY at S2 (rule F-0).
//
//      arrival_request_N / drain_request --> rank, free pick --> entry write (packet, pc, set, write, deps)
//      fresh_allowed_q, credit_q ---------> fresh_issue (the lowest arrival goes now)        --> l1d_bank_s0
//      entries (flops) ---------------------> oldest READY entry with deps = 0 (age matrix)   --> l1d_book_select
//      stage S2 results ---------------------> entry freed / WAIT (masks) / READY again
//
// STAGE / CYCLE
//   Stage G (S0). Arrivals are written at the end of their arrival cycle. fresh_issue is combinational in the arrival cycle
//   (pins -> rank -> AND with flops, about 6 levels). The booking outputs are combinational from flops (used by
//   l1d_book_select in cycle c-1 for cycle c).
//
// INSTANTIATED BY
//   l1d_bank_s0, instance u_tracker (one per bank).
//
// PARAMETERS (all from l1d_defines.vh through `L1D_PARAMS; used here)
//   TRK_ENTRIES     default 8: entries per bank
//   TRK_ALLOC_MAX   default 4: arrivals allocated in one cycle (4 = one per port; 2 = shorter free-pick chain, PLAN_v6 O-2)
//   PC_W            default 64: instruction PC kept per entry (prefetch training only)
//   REQ_W (148), SETS (64), LSU_PORTS (must be 4), MSHR_ENTRIES (8), VC_SLOTS (8)
//   BANK_ID         default 0: number of this bank (messages only)
//
// PORTS (name, direction, width at the defaults, meaning)       N = 0..3: the request port
//   clk, rst                     in   1       clock; synchronous reset, active high (every entry free)
//   arrival_request_N            in   1       port N's pins request is for this bank and may be allocated (its ring is empty)
//   arrival_packet_N             in   148     its request packet
//   arrival_pc_N                 in   64      its instruction PC
//   drain_request                in   1       the ring head of this bank (a skidded request) asks for an entry (ring not empty)
//   drain_packet, drain_pc       in   148/64  its packet and PC
//   vec_slot_available           in   1       a vector data place is free (a vector store needs one to be allocated)
//   vec_write_slot               in   1       the free place; written into the packet's data field
//   vec_write_valid              out  1       a vector store is allocated now: keep its line in place vec_write_slot
//   fresh_allowed_q              in   1       flop: nothing is booked for this cycle and the door is open (l1d_book_select)
//   credit_q_N                   in   1       flop: port N has a response credit for this cycle (l1d_bank_s0)
//   credit_book_next             in   4       booking credit of each port for the next cycle (l1d_bank_s0; tm_opt bank iter 3)
//   fresh_issue                  out  1       the lowest allocated pins arrival enters the pipeline now (combinational)
//   fresh_port_onehot            out  4       which port it is (one-hot; zero when none)
//   fresh_entry_onehot           out  8       the entry it takes (one-hot)
//   any_pins_arrival             out  1       some pins arrival this cycle (SRAM read enable of the address lane)
//   arrival_allocated_N          out  1       port N's arrival got an entry (the front pushes the others into the skids)
//   drain_allocated              out  1       the drained request got an entry (the front pops its skid)
//   oldest_ready_valid           out  1       a READY entry with no dependency exists
//   oldest_ready_onehot          out  8       the oldest of them (one-hot)
//   oldest_ready_set/_line/_port/_is_write/_is_line_op/_is_amo   out   its fields for the booking
//   arrival_book_valid           out  1       an arrival of this cycle (not the fresh one) has no dependency: it may be booked
//   arrival_book_entry_onehot    out  8       its entry
//   arrival_book_set/_line/_port/_is_write/_is_line_op/_is_amo   out   its fields
//   arrival_book_credit_ok       out  1       its port has a booking credit next cycle (tm_opt bank iter 3)
//   arrival_book_allowed         in   1       Vivado iter 5: CPU work may book an arrival this cycle (door open, nothing else wins)
//   arrival_book_go              out  1       Vivado iter 5: the arrival is booked now (= book_now_arrival, from the pick)
//   arrival_booked_entry_onehot/_line/_port/_is_write/_is_amo   out   Vivado iter 5: the arrival_book_* fields, ZERO unless booked
//   book_entry_onehot            in   8       the booking takes this entry for the next cycle (READY -> BOOKED)
//   issue_entry_onehot           in   8       the booked entry issues now (BOOKED -> INFL, flops of l1d_bank_s0)
//   entry_packet_flat            out  8*148   every entry's packet (the booked one is picked by l1d_bank_s0)
//   entry_pc_flat                out  8*64    every entry's PC
//   s2_cpu_valid                 in   1       stage S2 holds a tracked CPU request (not a prefetch)
//   s2_seat                      in   3       its entry
//   s2_kill                      in   1       it was cancelled in R (ghost): back to READY
//   s2_answered                  in   1       it answers now (s3_load): the entry is freed
//   act_valid/_code/_id          in   1/3/3   its S2 result (ACT_*): OWNED frees, WECC -> READY, W* -> WAIT with a mask
//   s2_train_sent                in   1       S2 sent its training record: remember it
//   s2_seat_trained              out  1       that entry already sent its record
//   s2_seat_pc                   out  64      that entry's PC (for the training record)
//   mshr_freed, vc_freed, nc_freed  in 8/8/1  wake-up pulses
//   free_count_q                 out  4       flop: free entries (0..8), read by the port front for its skid decision
//   entry_valid_q                out  8       flop: entries in use
//   tracker_empty                out  1       no entry in use (bank idle check)
//
// WORKED EXAMPLE (ports 0 and 2 send to this bank in cycle c0, bank free, entries 0 and 5 in use)
//   c0: eligible = {P0, P2}; rank(P0) = 0, rank(P2) = 1; free entries 1,2,3,4,6,7 -> P0 takes entry 1 (rank 0 = first free),
//       P2 takes entry 2. fresh_issue = 1 for P0 (fresh_allowed_q, credit): entry 1 is written INFL, entry 2 READY. If P2's
//       set equals P0's and one of them writes, entry 2 gets deps[1] = 1. Else arrival_book_valid = 1 for entry 2 and the
//       booking takes it for c1 (answer at c5: the "4 and 5" case).
//   c3: P0's request hits in S2 and answers: entry 1 freed, deps[*][1] cleared.
//
// ASSUMPTIONS (checked in simulation)
//   A1 arrival_request_N and drain_request never come in the same cycle (the front sends pins arrivals only while the ring is
//      empty); A2 at most one S2 event per cycle; A3 LSU_PORTS = 4.
// ============================================================================
`ifndef L1D_REQ_TRACKER_V
`define L1D_REQ_TRACKER_V
`default_nettype none
`include "l1d_defines.vh"

module l1d_req_tracker #(
    `L1D_PARAMS,                                       // all cache parameters (l1d_defines.vh)
    parameter BANK_ID = 0                              // default 0: number of this bank (messages only)
) (
    clk, rst, arrival_request_0, arrival_request_1, arrival_request_2, arrival_request_3,
    arrival_packet_0, arrival_packet_1, arrival_packet_2, arrival_packet_3, arrival_pc_0, arrival_pc_1,
    arrival_pc_2, arrival_pc_3, drain_request, drain_packet, drain_pc, vec_slot_available,
    vec_write_slot, vec_write_valid, fresh_allowed_q, credit_q_0, credit_q_1, credit_q_2,
    credit_q_3, credit_book_next, fresh_issue, fresh_port_onehot, fresh_entry_onehot, any_pins_arrival, arrival_allocated_0,
    arrival_allocated_1, arrival_allocated_2, arrival_allocated_3, drain_allocated, oldest_ready_valid, oldest_ready_onehot,
    oldest_ready_set, oldest_ready_line, oldest_ready_port, oldest_ready_credit_ok, oldest_ready_is_write, oldest_ready_is_line_op, oldest_ready_is_amo,
    arrival_book_valid, arrival_book_entry_onehot, arrival_book_set, arrival_book_line, arrival_book_port, arrival_book_is_write,
    arrival_book_is_line_op, arrival_book_is_amo, arrival_book_credit_ok, arrival_book_allowed, arrival_book_go, arrival_booked_entry_onehot, arrival_booked_line,
    arrival_booked_port, arrival_booked_is_write, arrival_booked_is_amo, book_entry_onehot, book_ready_onehot, issue_entry_onehot, entry_packet_flat, entry_pc_flat,
    s2_cpu_valid, s2_seat, s2_kill, s2_answered, act_valid, act_code,
    act_id, s2_train_sent, s2_seat_trained, s2_seat_pc, mshr_freed, vc_freed,
    nc_freed, free_count_q, entry_valid_q, tracker_empty,
    r_seat, r_seat_deps
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // ======================================================================
    // Ports (declared after the includes so that the derived widths of l1d_fields.vh can be used)
    // ======================================================================
    // ---- clock and reset ----
    input  wire clk;                      // clock: all registers change on its rising edge
    input  wire rst;                      // synchronous reset, active high
    // ---- arrivals from the port front (pins requests for this bank while its ring is empty) ----
    input  wire arrival_request_0;        // port 0 (load / LR) pins request for this bank
    input  wire arrival_request_1;        // port 1 (load / LR)
    input  wire arrival_request_2;        // port 2 (store / SC / AMO)
    input  wire arrival_request_3;        // port 3 (VPU)
    input  wire [REQ_W-1:0]  arrival_packet_0;         // port 0 packet, REQ_W = 148 bits
    input  wire [REQ_W-1:0]  arrival_packet_1;         // port 1 packet, 148 bits
    input  wire [REQ_W-1:0]  arrival_packet_2;         // port 2 packet, 148 bits
    input  wire [REQ_W-1:0]  arrival_packet_3;         // port 3 packet, 148 bits
    input  wire [PC_W-1:0]             arrival_pc_0;             // port 0 instruction PC, PC_W = 64 bits
    input  wire [PC_W-1:0]             arrival_pc_1;             // port 1 PC, 64 bits
    input  wire [PC_W-1:0]             arrival_pc_2;             // port 2 PC, 64 bits
    input  wire [PC_W-1:0]             arrival_pc_3;             // port 3 PC, 64 bits (0: the VPU has no PC pin)
    // ---- the drained skid head (ring not empty): one per cycle ----
    input  wire drain_request;            // the ring head asks for an entry
    input  wire [REQ_W-1:0]  drain_packet;             // its packet, 148 bits
    input  wire [PC_W-1:0]             drain_pc;                 // its PC, 64 bits
    // ---- vector store data place (l1d_vec_store_buf) ----
    input  wire vec_slot_available;       // a vector data place is free
    input  wire [VEC_SLOT_W-1:0] vec_write_slot;       // the lowest free place, VEC_SLOT_W = 1 bit
    output wire vec_write_valid;          // a vector store is allocated now: its line goes into that place
    // ---- the fresh issue (the lowest pins arrival goes now) ----
    input  wire fresh_allowed_q;          // flop: nothing booked for this cycle, door open
    input  wire credit_q_0;               // flop: port 0 has a response credit this cycle
    input  wire credit_q_1;               // flop: port 1 credit
    input  wire credit_q_2;               // flop: port 2 credit
    input  wire credit_q_3;               // flop: port 3 credit
    // tm_opt bank iter 3 (new port, user approved): the booking credit of every port for the next cycle, from l1d_bank_s0
    input  wire [LSU_PORTS-1:0] credit_book_next;      // bit p: port p has a response credit next cycle, LSU_PORTS = 4 bits
    output wire fresh_issue;              // the lowest allocated pins arrival enters the pipeline now
    output wire [3:0]                  fresh_port_onehot;        // its port, one-hot (bit N = port N)
    output wire [TRK_ENTRIES-1:0]      fresh_entry_onehot;       // its entry, one-hot, 8 bits
    output wire any_pins_arrival;         // some pins arrival this cycle (address lane read enable)
    output wire arrival_allocated_0;      // port 0's arrival got an entry
    output wire arrival_allocated_1;      // port 1's arrival got an entry
    output wire arrival_allocated_2;      // port 2's arrival got an entry
    output wire arrival_allocated_3;      // port 3's arrival got an entry
    output wire drain_allocated;          // the drained request got an entry
    // ---- booking candidates (combinational from flops; read by l1d_book_select in cycle c-1) ----
    output wire oldest_ready_valid;       // a READY entry without dependencies exists
    output wire [TRK_ENTRIES-1:0]      oldest_ready_onehot;      // the oldest of them, 8 bits
    output wire [INDEX_W-1:0] oldest_ready_set;        // its set, INDEX_W = 6 bits
    output wire [LINE_ADDR_W-1:0] oldest_ready_line;   // its line address, LINE_ADDR_W = 58 bits
    output wire [PORT_W-1:0] oldest_ready_port;        // its port, PORT_W = 2 bits
    output wire oldest_ready_credit_ok;   // iter 8: its port has a booking credit next cycle (read only with oldest_ready_valid)
    output wire oldest_ready_is_write;    // it writes (store class)
    output wire oldest_ready_is_line_op;  // it is a whole-line VPU access
    output wire oldest_ready_is_amo;      // it is a one-request AMO
    output wire arrival_book_valid;       // an arrival of this cycle without dependencies (not the fresh one)
    output wire [TRK_ENTRIES-1:0]      arrival_book_entry_onehot;  // its entry, 8 bits
    output wire [INDEX_W-1:0] arrival_book_set;        // its set, 6 bits
    output wire [LINE_ADDR_W-1:0] arrival_book_line;   // its line address, 58 bits
    output wire [PORT_W-1:0] arrival_book_port;        // its port, 2 bits
    output wire arrival_book_is_write;    // it writes
    output wire arrival_book_is_line_op;  // whole-line VPU access
    output wire arrival_book_is_amo;      // one-request AMO
    // tm_opt bank iter 3 (new port, user approved): credit_book_next[arrival_book_port], looked up per source early
    output wire arrival_book_credit_ok;   // the arrival candidate's port has a credit next cycle (read only with arrival_book_valid)
    // Vivado iter 5 (new ports): the pick selects the booked fields itself, so the late book_now_arrival select (fanout 108)
    // leaves the booking path. The input feeds ONLY the five new outputs below (every old output is unchanged).
    input  wire arrival_book_allowed;     // CPU work may book an arrival this cycle (cpu_go and not the oldest READY entry), from l1d_book_select
    output wire arrival_book_go;          // the arrival is booked now: allowed, a candidate picked, its port has a credit
    output wire [TRK_ENTRIES-1:0]      arrival_booked_entry_onehot;  // its entry, 0 unless booked, 8 bits
    output wire [LINE_ADDR_W-1:0]      arrival_booked_line;          // its line address, 0 unless booked, 58 bits
    output wire [PORT_W-1:0]           arrival_booked_port;          // its port, 0 unless booked, 2 bits
    output wire                        arrival_booked_is_write;      // it writes, 0 unless booked
    output wire                        arrival_booked_is_amo;        // one-request AMO, 0 unless booked
    input  wire [TRK_ENTRIES-1:0]      book_entry_onehot;        // the booking takes this entry for the next cycle (late in c-1)
    // Vivado iter 1 (new port): book_now_entry & oldest_ready_onehot, the early leg of book_entry_onehot. An in-use entry
    // (valid_q = 1) can only be booked by this leg: the arrival leg books a FREE entry (free_pick). So the in-use state
    // update reads this early input and the late arrival chain (set compare -> deps -> pick) reaches only free entries.
    input  wire [TRK_ENTRIES-1:0]      book_ready_onehot;        // oldest-READY leg of the booking, 8 bits
    input  wire [TRK_ENTRIES-1:0]      issue_entry_onehot;       // the booked entry issues now (flops of l1d_bank_s0)
    output wire [TRK_ENTRIES*REQ_W-1:0] entry_packet_flat;  // every entry's packet, 8*148 bits
    output wire [TRK_ENTRIES*PC_W-1:0] entry_pc_flat;            // every entry's PC, 8*64 bits
    // ---- stage S2 results ----
    input  wire s2_cpu_valid;             // S2 holds a tracked CPU request (not a prefetch, not a token)
    input  wire [SEAT_W-1:0] s2_seat;                  // its entry, SEAT_W = 3 bits
    input  wire s2_kill;                  // it was cancelled in R: back to READY
    input  wire s2_answered;              // it answers now (s3_load)
    input  wire act_valid;                // it has an S2 action code
    input  wire [2:0]                  act_code;                 // ACT_* (l1d_encodings.vh)
    input  wire [WID_W-1:0]  act_id;                   // MSHR entry or victim slot it waits for, WID_W = 3 bits
    input  wire s2_train_sent;            // S2 sent the training record of this entry
    output wire s2_seat_trained;          // this entry's record was already sent
    output wire [PC_W-1:0]             s2_seat_pc;               // this entry's PC, 64 bits
    input  wire [MSHR_ENTRIES-1:0]     mshr_freed;               // wake-ups: one pulse per MSHR entry freed, 8 bits
    input  wire [VC_SLOTS-1:0]         vc_freed;                 // wake-ups: one pulse per victim slot freed, 8 bits
    input  wire nc_freed;                 // wake-up: an NC place freed
    // ---- the R check (l1d_bank_r): the dependency bits of the entry in R ----
    input  wire [SEAT_W-1:0] r_seat;                   // the entry of the CPU request in R, SEAT_W = 3 bits
    output wire [TRK_ENTRIES-1:0] r_seat_deps;         // its dependency bits (flops), 8 bits
    // ---- status ----
    output reg  [TRK_CNT_W-1:0] free_count_q;          // flop: free entries 0..8, TRK_CNT_W = 4 bits
    output wire [TRK_ENTRIES-1:0]      entry_valid_q;            // flop per entry: in use, 8 bits
    output wire tracker_empty;             // no entry in use

    // ---- local constants (value at the defaults) ----
    localparam NE          = TRK_ENTRIES;                        // entries, 8
    localparam NS          = LSU_PORTS + 1;                      // arrival sources: 4 ports + the drained skid head = 5
    localparam RANK_W      = 3;                                  // rank of an arrival among this cycle's arrivals, 0..4, 3 bits
    localparam NA          = (TRK_ALLOC_MAX < NS) ? TRK_ALLOC_MAX : NS;   // most allocations per cycle, 4
    localparam SET_IN_REQ  = REQ_ADDR_LSB + OFFSET_W + LA_SET_LSB;        // lowest bit of the set number in a packet
    localparam LINE_IN_REQ = REQ_ADDR_LSB + OFFSET_W;                     // lowest bit of the line address in a packet
    localparam [TRK_CNT_W-1:0] NE_C = NE;                        // NE at the counter width, 4 bits
    localparam [REQ_W-1:0] SLOT_MASK = {{(REQ_W-VEC_SLOT_W){1'b0}}, {VEC_SLOT_W{1'b1}}} << REQ_DATA_LSB;   // vector place bits, 148 bits

    // ======================================================================
    // THE ENTRIES (flops)
    // ======================================================================
    reg  [NE-1:0]           valid_q;                             // entry in use, 8 bits
    // tm_opt iter 16 (new register, user approved): a copy of valid_q that feeds only the free count
    // (ks_ge) and free_rank of the next-pick logic: valid_q drives about 30 loads (50 ps of buffers)
    reg  [NE-1:0]           valid_dup_q;                         // copy of valid_q, 8 bits
    reg  [TRK_STATE_W-1:0]  state_q [0:NE-1];                    // one-hot state per entry, 8 x 4 bits (TRK_*_B)
    reg  [REQ_W-1:0]        packet_q [0:NE-1];                   // request packets, 8 x 148 bits
    reg  [PC_W-1:0]         pc_q [0:NE-1];                       // instruction PCs, 8 x 64 bits
    reg  [INDEX_W-1:0]      set_q [0:NE-1];                      // set numbers (own flops for the compares), 8 x 6 bits
    reg  [NE-1:0]           write_q;                             // store class per entry, 8 bits
    reg  [NE-1:0]           deps_q [0:NE-1];                     // deps_q[e][k] = 1: e waits for older entry k, 8 x 8 bits
    reg  [MSHR_ENTRIES-1:0] wait_mshr_q [0:NE-1];                // WAIT masks: MSHR entries waited for, 8 x 8 bits
    reg  [VC_SLOTS-1:0]     wait_vc_q [0:NE-1];                  // WAIT masks: victim slots waited for, 8 x 8 bits
    reg  [NE-1:0]           wait_nc_q;                           // WAIT: an NC place, 8 bits
    reg  [NE-1:0]           trained_q;                           // training record sent, 8 bits

    // ======================================================================
    // ARRIVAL SOURCES: 4 ports (pins) and the drained skid head, side by side
    // ======================================================================
    wire [NS-1:0]       src_valid;                               // a request of source s this cycle, 5 bits
    wire [NS*REQ_W-1:0] src_packet;                              // their packets, 5 x 148 bits
    wire [NS*PC_W-1:0]  src_pc;                                  // their PCs, 5 x 64 bits
    wire [NS-1:0]       src_write;                               // source writes (store class), 5 bits
    wire [NS-1:0]       src_line_op;                             // source is a whole-line VPU access, 5 bits
    wire [NS-1:0]       src_line_store;                          // source is a vector store (needs a data place), 5 bits
    wire [NS-1:0]       src_place_ok;                            // not a vector store, or a place is free, 5 bits
    wire [NS-1:0]       eligible;                                // may take an entry this cycle (room permitting), 5 bits
    wire [NS*INDEX_W-1:0] src_set;                               // their sets, 5 x 6 bits
    wire [NS-1:0]       src_credit_ok;                           // tm_opt bank iter 3: the source's port has a booking credit, 5 bits

    assign src_valid  = {drain_request, arrival_request_3, arrival_request_2, arrival_request_1, arrival_request_0};
    assign src_packet = {drain_packet, arrival_packet_3, arrival_packet_2, arrival_packet_1, arrival_packet_0};
    assign src_pc     = {drain_pc, arrival_pc_3, arrival_pc_2, arrival_pc_1, arrival_pc_0};
    genvar src_idx;                                              // loop variable: arrival source
    generate
        for (src_idx = 0; src_idx < NS; src_idx = src_idx + 1) begin : g_src
            assign src_write[src_idx]      = src_packet[src_idx*REQ_W + REQ_RDWR_BIT];
            assign src_line_op[src_idx]    = src_packet[src_idx*REQ_W + REQ_LINE_BIT];
            assign src_line_store[src_idx] = src_line_op[src_idx] & src_write[src_idx];
            assign src_place_ok[src_idx]   = !src_line_store[src_idx] | vec_slot_available;
            assign eligible[src_idx]       = src_valid[src_idx] & src_place_ok[src_idx];
            assign src_set[src_idx*INDEX_W +: INDEX_W] = src_packet[src_idx*REQ_W + SET_IN_REQ +: INDEX_W];
            // tm_opt bank iter 3: the credit of the port in this source's packet (PORT_W = 2 bits), in parallel
            // with the dependency compares, so the 4:1 credit select leaves the booking path
            assign src_credit_ok[src_idx]  = credit_book_next[src_packet[src_idx*REQ_W + REQ_PORT_LSB +: PORT_W]];
        end
    endgenerate
    assign any_pins_arrival = |src_valid[LSU_PORTS-1:0];

    // ---- rank of every source = number of eligible lower sources (0..4); 2 levels from the pins for the 4 ports ----
    reg  [RANK_W-1:0] rank [0:NS-1];                             // 5 x 3 bits
    integer rank_src;                                            // loop variable: source whose rank is built
    integer rank_lower;                                          // loop variable: a lower source
    always @* begin
        for (rank_src = 0; rank_src < NS; rank_src = rank_src + 1) begin
            rank[rank_src] = {RANK_W{1'b0}};
            // tm_opt iter 5: rank counts in the saturating Johnson code of free_below (iter 4), RANK_W = 3 bits:
            //   0 = 000, 1 = 001, 2 = 011, 3 = 111, 4 = 110   (rank <= NS-1 = 4)
            // the +1 is AND / OR only, so synthesis builds no FA / HA cells on the pins -> rank -> alloc path (208 ps in
            // iter 4); every reader of rank compares with these codes
            for (rank_lower = 0; rank_lower < rank_src; rank_lower = rank_lower + 1)
                rank[rank_src] = eligible[rank_lower] ?
                                 {rank[rank_src][1] | rank[rank_src][2],
                                  rank[rank_src][0], ~rank[rank_src][2]} :
                                 rank[rank_src];
        end
    end

    // ---- allocation: an eligible source gets an entry when its rank is below the free count and below TRK_ALLOC_MAX ----
    wire [NS-1:0] alloc;                                         // source takes an entry this cycle, 5 bits
    // tm_opt iter 12 (new signal, user approved): free_room[k] = (free_count_q > k), all k at once
    // from the flop: ones below free_count_q (a count of 9 or more gives all ones, as > does)
    wire [NE:0]   free_room;                                     // thermometer of the free count, NE+1 = 9 bits
    // tm_opt iter 13 (new register, user approved): the thermometer is held in flops next to
    // free_count_q (free_room_proc), so the 88 ps shift is no longer on the path from the flop
    reg  [NE:0]   free_room_q;                                   // = ~(ones << free_count_q), 9 bits
    // tm_opt iter 15 (B4, new register, user approved): a second copy of the thermometer for
    // rank_alloc only, so the room bits feeding the 5 rank_in_room checks and the ones feeding
    // rank_alloc are separate nets (the copy had fan-out 9 to 10 and 86 ps of buffers, iter 14)
    reg  [NE:0]   free_room_dup_q;                               // copy of free_room_q, 9 bits
    assign free_room = free_room_q;
    genvar al_idx;                                               // loop variable: source
    generate
        for (al_idx = 0; al_idx < NS; al_idx = al_idx + 1) begin : g_alloc
            wire rank_in_room;                                   // rank < free entries
            wire rank_in_max;                                    // rank < TRK_ALLOC_MAX
            // tm_opt iter 5: rank is a Johnson code (codes of 0..4: 000 001 011 111 110, RANK_W = 3 bits). rank < free
            // count: the late rank code selects one of the early compares free_count_q > k of the flop (no comparator)
            // tm_opt iter 12: free_count_q > k is bit k of the thermometer free_room (a shift, not a
            // compare), so Yosys cannot merge it with the free_count_q - k values of free_count_next
            // into one shared ALU (that put HA cells on this path in iterations 7 and 9)
            assign rank_in_room = ((rank[al_idx] == 3'b000) && free_room[0]) ||
                                  ((rank[al_idx] == 3'b001) && free_room[1]) ||
                                  ((rank[al_idx] == 3'b011) && free_room[2]) ||
                                  ((rank[al_idx] == 3'b111) && free_room[3]) ||
                                  ((rank[al_idx] == 3'b110) && free_room[4]);
            // rank < NA: the rank code is one of the codes of 0 .. NA-1 (the k < NA terms are constants)
            assign rank_in_max  = ((rank[al_idx] == 3'b000) && (0 < NA)) ||
                                  ((rank[al_idx] == 3'b001) && (1 < NA)) ||
                                  ((rank[al_idx] == 3'b011) && (2 < NA)) ||
                                  ((rank[al_idx] == 3'b111) && (3 < NA)) ||
                                  ((rank[al_idx] == 3'b110) && (4 < NA));
            assign alloc[al_idx] = eligible[al_idx] & rank_in_room & rank_in_max;
        end
    endgenerate
    assign arrival_allocated_0 = alloc[0];
    assign arrival_allocated_1 = alloc[1];
    assign arrival_allocated_2 = alloc[2];
    assign arrival_allocated_3 = alloc[3];
    assign drain_allocated     = alloc[LSU_PORTS];
    assign vec_write_valid     = |(alloc & src_line_store);

    // ======================================================================
    // FREE PICK: free_pick[k] = one-hot of the k-th free entry (k = 0 .. NA-1), from the valid flops only
    // ======================================================================
    wire [NA*NE-1:0] free_pick;                                  // 4 x 8 bits
    // tm_opt iter 11 (new register, user approved): free_pick held in flops, built one cycle
    // ahead from the next free set (free_pick_proc below), so every path that starts with the
    // pick starts at a flop (valid_q -> free_below -> free_pick was 150 to 190 ps)
    reg  [NA*NE-1:0] free_pick_q;                                // registered free pick, 4 x 8 bits
    generate
        if (NA <= 2) begin : g_pick_chain
            // two find-first chains (about 3 + 3 levels): the first free entry, then the first free entry after it
            wire [NE-1:0] free_now;                              // free entries, 8 bits
            wire [NE-1:0] first_free;                            // one-hot, 8 bits
            wire [NE-1:0] rest_free;                             // free entries without the first, 8 bits
            wire [NE-1:0] second_free;                           // one-hot, 8 bits
            assign free_now = ~valid_q;
            genvar ch_idx;                                       // loop variable: entry
            for (ch_idx = 0; ch_idx < NE; ch_idx = ch_idx + 1) begin : g_chain
                if (ch_idx == 0) begin : g_first
                    assign first_free[ch_idx]  = free_now[ch_idx];
                    assign second_free[ch_idx] = 1'b0;
                end else begin : g_rest
                    assign first_free[ch_idx]  = free_now[ch_idx] & ~|free_now[ch_idx-1:0];
                    assign second_free[ch_idx] = rest_free[ch_idx] & ~|rest_free[ch_idx-1:0];
                end
            end
            assign rest_free = free_now & ~first_free;
            assign free_pick[0 +: NE] = first_free;
            if (NA == 2) begin : g_two
                assign free_pick[NE +: NE] = second_free;
            end
        end else begin : g_pick_count
            // entry e is the k-th free entry when it is free and exactly k free entries lie below it (popcount of up to 7 flop
            // bits, about 5 to 6 levels; PLAN_v6 path D', O-2)
            // tm_opt iter 4: free_below holds the count in a saturating Johnson code, RANK_W = 3 bits:
            //   0 = 000, 1 = 001, 2 = 011, 3 = 111, 4 = 110, 5 or more = 100   (only 0 .. NA-1 <= 4 are read)
            // so the +1 is pure AND / OR (bit 0 <- ~bit 2, bit 1 <- bit 0, bit 2 <- bit 1 | bit 2), no XOR: synthesis
            // (extract_fa) cannot rebuild the FA / HA chain that was 456 ps of the worst flop-to-flop path
            reg [RANK_W-1:0] free_below [0:NE-1];                // free entries below e (Johnson code), 8 x 3 bits
            integer fb_entry;                                    // loop variable: entry
            integer fb_lower;                                    // loop variable: a lower entry
            always @* begin
                for (fb_entry = 0; fb_entry < NE; fb_entry = fb_entry + 1) begin
                    free_below[fb_entry] = {RANK_W{1'b0}};
                    for (fb_lower = 0; fb_lower < fb_entry; fb_lower = fb_lower + 1)
                        free_below[fb_entry] = ~valid_q[fb_lower] ?
                                               {free_below[fb_entry][1] | free_below[fb_entry][2],
                                                free_below[fb_entry][0], ~free_below[fb_entry][2]} :
                                               free_below[fb_entry];
                end
            end
            genvar pk_k, pk_e;                                   // loop variables: rank k, entry e
            for (pk_k = 0; pk_k < NA; pk_k = pk_k + 1) begin : g_pick_k
                for (pk_e = 0; pk_e < NE; pk_e = pk_e + 1) begin : g_pick_e
                    // the Johnson code of k (RANK_W = 3 bits): k ones from bit 0 up for k <= 3, then zeros from bit 0 up
                    // tm_opt iter 11: the pick comes from its flop (same value: free_pick_proc loads
                    // the pick of the next free set); free_below is no longer read here
                    assign free_pick[pk_k*NE + pk_e] = free_pick_q[pk_k*NE + pk_e];
                end
            end
        end
    endgenerate

    // ---- tm_opt iter 6 (new signals, user approved): the rank of every ALLOCATED source as a one-hot ----
    // From the pins and free_count_q only (no valid_q), so it is ready long before free_pick.
    // The allocated sources hold the ranks 0 .. n-1, one source per rank (eligible ranks count up
    // by one in source order), so rank_alloc is a thermometer: rank k is taken this cycle.
    // alloc_en, age_groups and deps_load use it to read free_pick directly by rank, without the
    // late src_entry select (src_entry -> alloc_en was 150 ps of the worst flop-to-flop path).
    reg  [NA-1:0] rank_onehot [0:NS-1];                          // rank_onehot[s][k]: s allocated with rank k, 5 x 4 bits
    reg  [NA-1:0] rank_alloc;                                    // rank k taken by an allocated source, 4 bits
    integer ro_src;                                              // loop variable: source
    integer ro_rank;                                             // loop variable: rank
    always @* begin
        rank_alloc = {NA{1'b0}};
        for (ro_src = 0; ro_src < NS; ro_src = ro_src + 1)
            for (ro_rank = 0; ro_rank < NA; ro_rank = ro_rank + 1) begin
                // the Johnson code of ro_rank (RANK_W = 3 bits), as in src_entry
                // tm_opt iter 15 (F1): rank_onehot from the PINS only (eligible and rank, no room
                // check): source s is the eligible arrival of rank k. The room bit of the flop is
                // applied once, last, in rank_alloc (rank k is taken when an eligible arrival has
                // rank k and free_count_q > k; rank k < NA by the loop bound): same rank_alloc as
                // before. rank_deps_old / rank_pair / deps_load need no room bit at all, as deps_q
                // only loads them when the entry is allocated (alloc_en, which has the room bit).
                rank_onehot[ro_src][ro_rank] = eligible[ro_src] &&
                    (rank[ro_src] == ((ro_rank < RANK_W) ? ((1 << ro_rank) - 1) :
                                      (((1 << RANK_W) - 1) & ~((1 << (ro_rank - RANK_W)) - 1))));
                rank_alloc[ro_rank] = rank_alloc[ro_rank] | (rank_onehot[ro_src][ro_rank] & free_room_dup_q[ro_rank]);
            end
    end

    // ---- the entry of every source (one-hot): the free entry of its rank; and the data select of every entry ----
    reg  [NE-1:0] src_entry [0:NS-1];                            // entry of source s if allocated, one-hot, 5 x 8 bits
    reg  [NS-1:0] data_sel [0:NE-1];                             // data_sel[e][s]: entry e loads source s (eligible, rank match), 8 x 5 bits
    reg  [NE-1:0] alloc_en;                                      // entry e is allocated this cycle, 8 bits
    reg  [NA*NE-1:0] age_groups;                                 // allocated entries by rank, for the age matrix, 4 x 8 bits
    integer se_src;                                              // loop variable: source
    integer se_rank;                                             // loop variable: rank
    integer se_entry;                                            // loop variable: entry
    always @* begin
        for (se_src = 0; se_src < NS; se_src = se_src + 1) begin
            src_entry[se_src] = {NE{1'b0}};
            // tm_opt iter 5: rank is a Johnson code: compare with the code of se_rank (RANK_W = 3 bits; k ones from bit 0
            // up for k <= 3, then zeros from bit 0 up), as free_pick does for free_below
            for (se_rank = 0; se_rank < NA; se_rank = se_rank + 1)
                if (rank[se_src] == ((se_rank < RANK_W) ? ((1 << se_rank) - 1) :
                                     (((1 << RANK_W) - 1) & ~((1 << (se_rank - RANK_W)) - 1))))
                    src_entry[se_src] = src_entry[se_src] | free_pick[se_rank*NE +: NE];
        end
        for (se_entry = 0; se_entry < NE; se_entry = se_entry + 1) begin
            alloc_en[se_entry] = 1'b0;
            for (se_src = 0; se_src < NS; se_src = se_src + 1) begin
                data_sel[se_entry][se_src] = eligible[se_src] & src_entry[se_src][se_entry];
                // tm_opt iter 6: entry e is allocated when it is the k-th free entry and rank k is taken
                // (same value as OR over s of alloc[s] & src_entry[s][e]). se_src walks the ranks here,
                // 0 .. NA-1 (NA <= NS); the index is held at 0 above NA-1, where the term is 0 anyway.
                alloc_en[se_entry] = alloc_en[se_entry] |
                    ((se_src < NA) &&
                     rank_alloc[(se_src < NA) ? se_src : 0] &&
                     free_pick[((se_src < NA) ? se_src : 0)*NE + se_entry]);
            end
        end
        for (se_rank = 0; se_rank < NA; se_rank = se_rank + 1) begin
            age_groups[se_rank*NE +: NE] = {NE{1'b0}};
            // tm_opt iter 6: the entry of the rank-k arrival is free_pick[k] when rank k is taken (same
            // value as the OR over the source of rank k of its src_entry); one pass of se_src adds it
            for (se_src = 0; se_src < NS; se_src = se_src + 1)
                if ((se_src == se_rank) && rank_alloc[se_rank])
                    age_groups[se_rank*NE +: NE] = age_groups[se_rank*NE +: NE] | free_pick[se_rank*NE +: NE];
        end
    end

    // ---- the data every free entry loads (every cycle, no late enable: rule SS-6): the AND-OR of the sources selected for it ----
    reg  [REQ_W-1:0] load_packet [0:NE-1];                       // 8 x 148 bits
    reg  [PC_W-1:0]  load_pc [0:NE-1];                           // 8 x 64 bits
    reg  [NE-1:0]    load_line_store;                            // the loaded request is a vector store: put the place number in, 8 bits
    integer ld_entry;                                            // loop variable: entry
    integer ld_src;                                              // loop variable: source
    always @* begin
        for (ld_entry = 0; ld_entry < NE; ld_entry = ld_entry + 1) begin
            load_packet[ld_entry]     = {REQ_W{1'b0}};
            load_pc[ld_entry]         = {PC_W{1'b0}};
            load_line_store[ld_entry] = 1'b0;
            for (ld_src = 0; ld_src < NS; ld_src = ld_src + 1) begin
                load_packet[ld_entry] = load_packet[ld_entry] | ({REQ_W{data_sel[ld_entry][ld_src]}} & src_packet[ld_src*REQ_W +: REQ_W]);
                load_pc[ld_entry]     = load_pc[ld_entry]     | ({PC_W{data_sel[ld_entry][ld_src]}}  & src_pc[ld_src*PC_W +: PC_W]);
                load_line_store[ld_entry] = load_line_store[ld_entry] | (data_sel[ld_entry][ld_src] & src_line_store[ld_src]);
            end
            // a vector store keeps its data place number in the data field (set at the bank door, as cache_v5 C-20)
            if (load_line_store[ld_entry])
                load_packet[ld_entry] = (load_packet[ld_entry] & ~SLOT_MASK) |
                                        ({{(REQ_W-VEC_SLOT_W){1'b0}}, vec_write_slot} << REQ_DATA_LSB);
        end
    end

    // ======================================================================
    // FRESH ISSUE: the lowest allocated pins arrival (rank 0) goes now when nothing is booked and its port has a credit
    // ======================================================================
    // The address lane of l1d_bank_s0 reads the set of the lowest REQUESTING port (the cheapest choice: 2 levels from the pins).
    // The fresh issue must use the same request, so it is the lowest requesting port AND that port is allocated (eligible, rank 0,
    // room). A lower port that requests but is not eligible (a vector store without a data place) blocks the fresh issue of this
    // cycle; the higher ports are still allocated and booked for the next cycle.
    wire [3:0] credit_q;                                         // per port credit flops, 4 bits
    reg  [LSU_PORTS-1:0] lowest_req;                             // the lowest requesting pins port, one-hot, 4 bits
    reg  [LSU_PORTS-1:0] lowest_port;                            // ... and it is allocated: the fresh candidate, one-hot, 4 bits
    reg                  lowest_credit;                          // its port has a credit
    reg                  lower_req_seen;                         // a lower port requests
    integer lo_port;                                             // loop variable: port
    assign credit_q = {credit_q_3, credit_q_2, credit_q_1, credit_q_0};
    always @* begin
        lowest_credit  = 1'b0;
        lower_req_seen = 1'b0;
        for (lo_port = 0; lo_port < LSU_PORTS; lo_port = lo_port + 1) begin
            // tm_opt iter 12: parallel first-one (no ripple through lower_req_seen): no lower port requests
            lowest_req[lo_port]  = src_valid[lo_port] & ~|(src_valid[LSU_PORTS-1:0] & ~({LSU_PORTS{1'b1}} << lo_port));
            lower_req_seen       = lower_req_seen | src_valid[lo_port];
            lowest_port[lo_port] = lowest_req[lo_port] & alloc[lo_port];
            lowest_credit        = lowest_credit | (lowest_port[lo_port] & credit_q[lo_port]);
        end
    end
    assign fresh_issue        = fresh_allowed_q & (|lowest_port) & lowest_credit;
    assign fresh_port_onehot  = lowest_port;
    assign fresh_entry_onehot = free_pick[0 +: NE];              // the rank-0 arrival takes the first free entry

    // ======================================================================
    // DEPENDENCIES AT ALLOCATION (rule T-2): older valid entries of the same set when one of the two writes, and the lower
    // arrivals of this cycle (they are older) under the same rule. 6-bit compares, about 6 levels, into flops.
    // ======================================================================
    wire [NE-1:0] freed_now;                                     // entries freed this cycle (declared below), 8 bits
    reg  [NE-1:0] deps_new [0:NS-1];                             // deps a source's entry gets, 5 x 8 bits
    reg           same_set_pair;                                 // scratch: two sources are on one set
    integer dp_src;                                              // loop variable: source
    integer dp_entry;                                            // loop variable: entry
    integer dp_lower;                                            // loop variable: a lower source
    // tm_opt iter 6 (new signals, user approved): the two parts of deps_new kept apart for deps_load
    // TIMING OPT (CB-2b, 2026-11-26): deps_old WITHOUT ~freed_now (pins and flops only). Was: S2 decision -> freed_now -> deps_old
    //   -> == 0 -> cand_nofresh -> arrival_pick -> arrival_booked_* -> book_line_q in one cycle. The exact mask is applied LAST in
    //   deps_load (deps_q bit-identical) and deps_new (same value). cand_nofresh reads the unmasked value: conservative.
    // Example: entry 3 (store, set 5) answers in S2 now; port 1 sends a load of set 5 beside a fresh port 0: deps_old = 0000_1000 ->
    //   not booked now; deps_q = 0 -> READY -> booked c+1, issued c+2 (was c+1; dependent case only).
    reg  [NE-1:0] deps_old [0:NS-1];                             // deps on older valid entries, S2 free NOT applied (CB-2b), 5 x 8 bits
    reg  [NS-1:0] lower_pair [0:NS-1];                           // lower_pair[s][l]: s waits for lower arrival l, 5 x 5 bits
    always @* begin
        for (dp_src = 0; dp_src < NS; dp_src = dp_src + 1) begin
            for (dp_entry = 0; dp_entry < NE; dp_entry = dp_entry + 1)
                // iter 7: same value (distributive law): the pin bit src_write picks between two flop-only terms, so the OR with
                // write_q is no longer a separate level in front of the set compare
                deps_old[dp_src][dp_entry] = (src_write[dp_src] ? valid_q[dp_entry]
                                                                : (valid_q[dp_entry] & write_q[dp_entry])) &
                                             // CB-2b: no ~freed_now here (applied last in deps_load and deps_new)
                                             (set_q[dp_entry] == src_set[dp_src*INDEX_W +: INDEX_W]);
            deps_new[dp_src]   = deps_old[dp_src] & ~freed_now;   // CB-2b: S2 free applied last (debug / testbench view, value unchanged)
            lower_pair[dp_src] = {NS{1'b0}};
            for (dp_lower = 0; dp_lower < dp_src; dp_lower = dp_lower + 1) begin
                same_set_pair = alloc[dp_lower] &&
                                (src_set[dp_lower*INDEX_W +: INDEX_W] == src_set[dp_src*INDEX_W +: INDEX_W]) &&
                                (src_write[dp_lower] || src_write[dp_src]);
                // tm_opt iter 15 (F1): lower_pair from the pins (eligible lower source, not alloc): it
                // is only read for an allocated source s, and then every eligible lower source is
                // allocated too (lower rank, same room), so the value read is the same
                lower_pair[dp_src][dp_lower] = eligible[dp_lower] &&
                    (src_set[dp_lower*INDEX_W +: INDEX_W] == src_set[dp_src*INDEX_W +: INDEX_W]) &&
                    (src_write[dp_lower] || src_write[dp_src]);
                if (same_set_pair)
                    deps_new[dp_src] = deps_new[dp_src] | src_entry[dp_lower];
            end
        end
    end

    // ---- tm_opt iter 6 (new signals, user approved): the deps every entry loads, by rank ----
    // The rank-k arrival takes the entry free_pick[k]. Its deps are its old-entry deps plus the
    // entries free_pick[j] of the lower arrivals (rank j) it waits for. The per-rank parts come
    // from the pins only; the late free_pick enters in the last two AND-OR levels. Same value as
    // the last-wins select of deps_new by alloc & src_entry that tracker_proc used before.
    reg  [NE-1:0] rank_deps_old [0:NA-1];                        // old-entry deps of the rank-k arrival, 4 x 8 bits
    reg  [NA-1:0] rank_pair [0:NA-1];                            // rank_pair[k][j]: rank-k arrival waits for rank j, 4 x 4 bits
    reg  [NE-1:0] deps_load [0:NE-1];                            // deps entry e loads when allocated, 8 x 8 bits
    reg  [NE-1:0] deps_load_older [0:NE-1];                      // CB-2b: old-entry part of deps_load, before the S2 free mask, 8 x 8 bits
    reg  [NE-1:0] deps_load_lower [0:NE-1];                      // CB-2b: lower-arrival part (free entries, never masked), 8 x 8 bits
    integer rd_rank;                                             // loop variable: rank of the arrival
    integer rd_src;                                              // loop variable: source
    integer rd_lower;                                            // loop variable: a lower source
    integer dl_entry;                                            // loop variable: entry
    integer dl_rank;                                             // loop variable: rank of the arrival
    integer dl_lower;                                            // loop variable: rank of a lower arrival
    always @* begin
        for (rd_rank = 0; rd_rank < NA; rd_rank = rd_rank + 1) begin
            rank_deps_old[rd_rank] = {NE{1'b0}};
            rank_pair[rd_rank]     = {NA{1'b0}};
            for (rd_src = 0; rd_src < NS; rd_src = rd_src + 1) begin
                if (rank_onehot[rd_src][rd_rank])
                    rank_deps_old[rd_rank] = rank_deps_old[rd_rank] | deps_old[rd_src];
                // a waited-for lower arrival is allocated: rank_onehot gives its rank.
                // tm_opt iter 13: the loop runs on every pass (the rank test is in the
                // if), so rd_lower is always assigned: no latch (Yosys warning, iter 6)
                for (rd_lower = 0; rd_lower < NS; rd_lower = rd_lower + 1)
                    if (rank_onehot[rd_src][rd_rank] && lower_pair[rd_src][rd_lower])
                        rank_pair[rd_rank] = rank_pair[rd_rank] | rank_onehot[rd_lower];
            end
        end
        for (dl_entry = 0; dl_entry < NE; dl_entry = dl_entry + 1) begin
            deps_load_older[dl_entry] = {NE{1'b0}};
            deps_load_lower[dl_entry] = {NE{1'b0}};
            for (dl_rank = 0; dl_rank < NA; dl_rank = dl_rank + 1) begin
                if (free_pick[dl_rank*NE + dl_entry])
                    deps_load_older[dl_entry] = deps_load_older[dl_entry] | rank_deps_old[dl_rank];
                for (dl_lower = 0; dl_lower < NA; dl_lower = dl_lower + 1)
                    if (free_pick[dl_rank*NE + dl_entry] && rank_pair[dl_rank][dl_lower])
                        deps_load_lower[dl_entry] = deps_load_lower[dl_entry] | free_pick[dl_lower*NE +: NE];
            end
            // CB-2b: the S2 free LAST, on the old-entry part only: OR_k (x_k & ~f) = (OR_k x_k) & ~f, so exactly the old deps_load
            deps_load[dl_entry] = (deps_load_older[dl_entry] & ~freed_now) | deps_load_lower[dl_entry];
        end
    end

    // ---- arrival booking candidate: the lowest allocated source without dependencies that is not the fresh one ----
    reg  [NS-1:0] arrival_cand;                                  // sources that may be booked, 5 bits
    reg  [NS-1:0] arrival_pick;                                  // the lowest of them, one-hot, 5 bits
    reg           arrival_seen;                                  // a lower candidate was found
    integer ab_src;                                              // loop variable: source
    // tm_opt iter 15 (B1, B2; new signals, user approved):
    // B2 (CB-2b: deps_old is unmasked, so deps_old == 0 implies the masked test: conservative): deps_new == 0 is the same as "no old dependency (deps_old == 0) and no waited-for lower
    //   arrival that gets an entry". A waited-for lower arrival l (lower_pair) of an allocated s
    //   is allocated, and its entry is free_pick[rank of l]; that is nonzero exactly when the
    //   flop row free_pick[k] has a bit (pick_any, early). So the late 8-bit deps_new compare and
    //   the src_entry select leave the booking path. Exact in every state (no invariant).
    // B1: carry-select on fresh_issue (late: alloc, credits): the candidates and the pick are
    //   built for fresh_issue = 0 and = 1 side by side, and fresh_issue selects at the end.
    reg  [NA-1:0] pick_any;                                      // free_pick[k] has an entry, 4 bits
    reg  [NS-1:0] pair_block;                                    // s waits for a lower arrival with an entry, 5 bits
    reg  [NS-1:0] cand_nofresh;                                  // arrival_cand when fresh_issue = 0, 5 bits
    reg  [NS-1:0] cand_fresh;                                    // arrival_cand when fresh_issue = 1, 5 bits
    reg  [NS-1:0] pick_nofresh;                                  // arrival_pick when fresh_issue = 0, 5 bits
    reg  [NS-1:0] pick_fresh;                                    // arrival_pick when fresh_issue = 1, 5 bits
    integer pb_rank;                                             // loop variable: rank
    integer pb_lower;                                            // loop variable: a lower source
    always @* begin
        for (pb_rank = 0; pb_rank < NA; pb_rank = pb_rank + 1)
            pick_any[pb_rank] = |free_pick[pb_rank*NE +: NE];
        arrival_seen = 1'b0;
        for (ab_src = 0; ab_src < NS; ab_src = ab_src + 1) begin
            pair_block[ab_src] = 1'b0;
            for (pb_lower = 0; pb_lower < NS; pb_lower = pb_lower + 1)
                if (lower_pair[ab_src][pb_lower])
                    pair_block[ab_src] = pair_block[ab_src] | (|(rank_onehot[pb_lower] & pick_any));
            cand_nofresh[ab_src] = alloc[ab_src] && (deps_old[ab_src] == {NE{1'b0}}) && !pair_block[ab_src];
            cand_fresh[ab_src]   = cand_nofresh[ab_src] &&
                                   !((rank[ab_src] == {RANK_W{1'b0}}) && (ab_src < LSU_PORTS));
        end
        for (ab_src = 0; ab_src < NS; ab_src = ab_src + 1) begin
            // parallel first-one (iter 12) for each case, then the late select
            pick_nofresh[ab_src] = cand_nofresh[ab_src] && !(|(cand_nofresh & ~({NS{1'b1}} << ab_src)));
            pick_fresh[ab_src]   = cand_fresh[ab_src]   && !(|(cand_fresh   & ~({NS{1'b1}} << ab_src)));
            arrival_cand[ab_src] = fresh_issue ? cand_fresh[ab_src] : cand_nofresh[ab_src];
            arrival_pick[ab_src] = fresh_issue ? pick_fresh[ab_src] : pick_nofresh[ab_src];
            arrival_seen         = arrival_seen || arrival_cand[ab_src];
        end
    end
    reg  [NE-1:0]          ab_entry;                             // its entry, 8 bits
    reg  [REQ_W-1:0]       ab_packet;                            // its packet, 148 bits
    // tm_opt iter 15 (B1): the entry and packet of both picks, fresh_issue selects last
    reg  [NE-1:0]          ab_entry_nofresh;                     // entry when fresh_issue = 0, 8 bits
    reg  [NE-1:0]          ab_entry_fresh;                       // entry when fresh_issue = 1, 8 bits
    reg  [REQ_W-1:0]       ab_packet_nofresh;                    // packet when fresh_issue = 0, 148 bits
    reg  [REQ_W-1:0]       ab_packet_fresh;                      // packet when fresh_issue = 1, 148 bits
    integer ab_mux;                                              // loop variable: source
    always @* begin
        ab_entry_nofresh  = {NE{1'b0}};
        ab_entry_fresh    = {NE{1'b0}};
        ab_packet_nofresh = {REQ_W{1'b0}};
        ab_packet_fresh   = {REQ_W{1'b0}};
        for (ab_mux = 0; ab_mux < NS; ab_mux = ab_mux + 1) begin
            ab_entry_nofresh  = ab_entry_nofresh  | ({NE{pick_nofresh[ab_mux]}}    & src_entry[ab_mux]);
            ab_entry_fresh    = ab_entry_fresh    | ({NE{pick_fresh[ab_mux]}}      & src_entry[ab_mux]);
            ab_packet_nofresh = ab_packet_nofresh | ({REQ_W{pick_nofresh[ab_mux]}} & src_packet[ab_mux*REQ_W +: REQ_W]);
            ab_packet_fresh   = ab_packet_fresh   | ({REQ_W{pick_fresh[ab_mux]}}   & src_packet[ab_mux*REQ_W +: REQ_W]);
        end
        ab_entry  = fresh_issue ? ab_entry_fresh  : ab_entry_nofresh;
        ab_packet = fresh_issue ? ab_packet_fresh : ab_packet_nofresh;
    end
    // Vivado iter 1: |arrival_pick == |arrival_cand (the pick is the lowest candidate), so the valid skips the first-one levels
    assign arrival_book_valid        = arrival_seen;
    assign arrival_book_entry_onehot = ab_entry;
    assign arrival_book_set          = ab_packet[SET_IN_REQ +: INDEX_W];
    assign arrival_book_line         = ab_packet[LINE_IN_REQ +: LINE_ADDR_W];
    assign arrival_book_port         = ab_packet[REQ_PORT_LSB +: PORT_W];
    assign arrival_book_is_write     = ab_packet[REQ_RDWR_BIT];
    assign arrival_book_is_line_op   = ab_packet[REQ_LINE_BIT];
    assign arrival_book_is_amo       = ab_packet[REQ_AMO_BIT] & ab_packet[REQ_SEQ_BIT];
    // tm_opt bank iter 3: equals credit_book_next[arrival_book_port] whenever arrival_book_valid = 1 (the pick is
    // one-hot and the port is that source's packet port); 0 when no source is picked (then l1d_book_select ignores it)
    assign arrival_book_credit_ok    = |(arrival_pick & src_credit_ok);

    // ---- Vivado iter 5: the booked arrival's fields, selected by the pick itself (zero unless booked) ----
    // arrival_pick is one-hot or zero, so book_now_arrival & field[pick] == OR_s (arrival_booked_source[s] & field[s]) with
    // arrival_booked_source[s] = arrival_pick[s] & credit[s] & allowed. The select of 5 AND terms replaces the credit OR,
    // the book_now_arrival gate and its select net (fanout 108) between the pick and the booking flops: two LUT levels less.
    wire [NS-1:0] arrival_booked_source;                         // the picked source when it is booked now, one-hot or zero, 5 bits
    reg  [NE-1:0]    booked_entry_or;                            // AND-OR of the booked source's entry, 8 bits
    reg  [LINE_ADDR_W-1:0] booked_line_or;                       // ... its line address, 58 bits
    reg  [PORT_W-1:0] booked_port_or;                            // ... its port, 2 bits
    reg              booked_is_write_or;                         // ... its write bit
    reg              booked_is_amo_or;                           // ... its AMO bit (AMO and sequence bit)
    integer bk_src;                                              // loop variable: source
    assign arrival_booked_source = arrival_pick & src_credit_ok & {NS{arrival_book_allowed}};
    always @* begin
        booked_entry_or    = {NE{1'b0}};
        booked_line_or     = {LINE_ADDR_W{1'b0}};
        booked_port_or     = {PORT_W{1'b0}};
        booked_is_write_or = 1'b0;
        booked_is_amo_or   = 1'b0;
        for (bk_src = 0; bk_src < NS; bk_src = bk_src + 1) begin
            booked_entry_or    = booked_entry_or    | ({NE{arrival_booked_source[bk_src]}}        & src_entry[bk_src]);
            booked_line_or     = booked_line_or     | ({LINE_ADDR_W{arrival_booked_source[bk_src]}} & src_packet[bk_src*REQ_W + LINE_IN_REQ +: LINE_ADDR_W]);
            booked_port_or     = booked_port_or     | ({PORT_W{arrival_booked_source[bk_src]}}     & src_packet[bk_src*REQ_W + REQ_PORT_LSB +: PORT_W]);
            booked_is_write_or = booked_is_write_or | (arrival_booked_source[bk_src] & src_packet[bk_src*REQ_W + REQ_RDWR_BIT]);
            booked_is_amo_or   = booked_is_amo_or   | (arrival_booked_source[bk_src] & src_packet[bk_src*REQ_W + REQ_AMO_BIT] &
                                                       src_packet[bk_src*REQ_W + REQ_SEQ_BIT]);
        end
    end
    assign arrival_book_go              = |arrival_booked_source;
    assign arrival_booked_entry_onehot  = booked_entry_or;
    assign arrival_booked_line          = booked_line_or;
    assign arrival_booked_port          = booked_port_or;
    assign arrival_booked_is_write      = booked_is_write_or;
    assign arrival_booked_is_amo        = booked_is_amo_or;

    // ======================================================================
    // OLDEST READY ENTRY (for the booking): valid, READY, no dependency left; the age matrix picks the oldest
    // ======================================================================
    reg  [NE-1:0] ready_cand;                                    // candidates, 8 bits
    integer rc_entry;                                            // loop variable: entry
    always @* begin
        for (rc_entry = 0; rc_entry < NE; rc_entry = rc_entry + 1)
            ready_cand[rc_entry] = valid_q[rc_entry] & state_q[rc_entry][TRK_READY_B] & (deps_q[rc_entry] == {NE{1'b0}});
    end
    wire [NE*NE-1:0] older_than_flat;                            // unused here (the deps carry the order), 64 bits
    wire [SEAT_W-1:0] oldest_index_unused;                       // unused (one-hot is used), 3 bits
    l1d_age_matrix #(.N(NE), .IDX_W(SEAT_W), .ONEHOT_ALLOC(1), .ONEHOT_ENC(1), .MULTI_ALLOC(1), .MULTI_GROUPS(NA)) u_age (
        .clk                   (clk),
        .rst                   (rst),
        .allocate_valid        (1'b0),
        .allocate_index        ({SEAT_W{1'b0}}),
        .allocate_onehot       ({NE{1'b0}}),
        .allocate_group_onehot (age_groups),
        .candidate_mask        (ready_cand),
        .older_than_flat       (older_than_flat),
        .oldest_onehot         (oldest_ready_onehot),
        .oldest_index          (oldest_index_unused),
        .any_candidate         (oldest_ready_valid)
    );
    reg  [REQ_W-1:0] or_packet;                                  // the oldest ready entry's packet, 148 bits
    integer or_entry;                                            // loop variable: entry
    always @* begin
        or_packet = {REQ_W{1'b0}};
        for (or_entry = 0; or_entry < NE; or_entry = or_entry + 1)
            or_packet = or_packet | ({REQ_W{oldest_ready_onehot[or_entry]}} & packet_q[or_entry]);
    end
    assign oldest_ready_set        = or_packet[SET_IN_REQ +: INDEX_W];
    assign oldest_ready_line       = or_packet[LINE_IN_REQ +: LINE_ADDR_W];
    assign oldest_ready_port       = or_packet[REQ_PORT_LSB +: PORT_W];
    // iter 8 (E1): the credit of the oldest READY entry's port. Each entry's port credit is looked up from its flops (early);
    // the age one-hot picks it (one AND-OR level) instead of the 8:1 port select followed by the 4:1 credit select. Equal to
    // credit_book_next[oldest_ready_port] when an entry is ready and the age pick is one-hot (assumption A1 of the proof flow);
    // 0 when none is ready (l1d_book_select ignores it then).
    reg     oldest_credit_or;                                    // AND-OR of the per-entry credit by the age one-hot
    integer oc_entry;                                            // loop variable: entry
    always @* begin
        oldest_credit_or = 1'b0;
        for (oc_entry = 0; oc_entry < NE; oc_entry = oc_entry + 1)
            oldest_credit_or = oldest_credit_or | (oldest_ready_onehot[oc_entry] &
                               credit_book_next[packet_q[oc_entry][REQ_PORT_LSB +: PORT_W]]);
    end
    assign oldest_ready_credit_ok  = oldest_credit_or;
    assign oldest_ready_is_write   = or_packet[REQ_RDWR_BIT];
    assign oldest_ready_is_line_op = or_packet[REQ_LINE_BIT];
    assign oldest_ready_is_amo     = or_packet[REQ_AMO_BIT] & or_packet[REQ_SEQ_BIT];

    // ---- every entry's packet and PC, flat (l1d_bank_s0 picks the booked one with book_entry_onehot_q) ----
    genvar fl_idx;                                               // loop variable: entry
    generate
        for (fl_idx = 0; fl_idx < NE; fl_idx = fl_idx + 1) begin : g_flat
            assign entry_packet_flat[fl_idx*REQ_W +: REQ_W] = packet_q[fl_idx];
            assign entry_pc_flat[fl_idx*PC_W +: PC_W]       = pc_q[fl_idx];
        end
    endgenerate

    // ======================================================================
    // STAGE S2 RESULTS: the entry of the CPU request in S2 is freed, sent back to READY, or put to WAIT
    // ======================================================================
    reg  [NE-1:0] s2_entry_onehot;                               // the entry in S2, one-hot (zero when none), 8 bits
    integer s2_dec;                                              // loop variable: entry
    always @* begin
        for (s2_dec = 0; s2_dec < NE; s2_dec = s2_dec + 1)
            s2_entry_onehot[s2_dec] = s2_cpu_valid && (s2_seat == s2_dec);
    end
    wire s2_free_entry;                                          // the request is done: answered or handed to a miss unit
    wire s2_to_ready;                                            // ghost, or ECC scrub: READY again
    wire s2_to_wait;                                             // wait for a miss-unit event
    assign s2_free_entry = !s2_kill && (s2_answered || (act_valid && (act_code == ACT_OWNED)));
    assign s2_to_ready   = s2_kill || (act_valid && (act_code == ACT_WECC));
    assign s2_to_wait    = !s2_kill && act_valid && ((act_code == ACT_WMSHR) || (act_code == ACT_WANY) ||
                                                     (act_code == ACT_WVC)   || (act_code == ACT_WNC));
    assign freed_now = s2_entry_onehot & {NE{s2_free_entry}};
    // the wait masks of the request in S2 (same form as the lean S0 replay queue); a wake-up in this very cycle is not lost
    wire [MSHR_ENTRIES-1:0] push_mshr_mask;                      // 8 bits
    wire [VC_SLOTS-1:0]     push_vc_mask;                        // 8 bits
    wire                    push_nc_bit;
    wire                    push_ready_now;                      // the event happens in this cycle: READY, not WAIT
    assign push_mshr_mask = (act_code == ACT_WMSHR) ? ({{(MSHR_ENTRIES-1){1'b0}}, 1'b1} << act_id[MSHR_ID_W-1:0]) :
                            (act_code == ACT_WANY)  ? {MSHR_ENTRIES{1'b1}} : {MSHR_ENTRIES{1'b0}};
    assign push_vc_mask   = (act_code == ACT_WVC) ? ({{(VC_SLOTS-1){1'b0}}, 1'b1} << act_id[VC_ID_W-1:0]) : {VC_SLOTS{1'b0}};
    assign push_nc_bit    = (act_code == ACT_WNC);
    assign push_ready_now = (|(push_mshr_mask & mshr_freed)) || (|(push_vc_mask & vc_freed)) || (push_nc_bit && nc_freed);

    assign s2_seat_trained = trained_q[s2_seat];
    assign s2_seat_pc      = pc_q[s2_seat];
    assign r_seat_deps     = deps_q[r_seat];

    // ======================================================================
    // THE CLOCKED PART: entries, states, dependencies, free count
    // ======================================================================
    reg  [RANK_W-1:0]    alloc_cnt;                              // entries allocated this cycle, 0..4, 3 bits
    integer ac_src;                                              // loop variable: source
    always @* begin
        alloc_cnt = {RANK_W{1'b0}};
        // tm_opt iter 3: the allocated sources are the first eligible ones, with ranks 0, 1, .. n-1 in source order, so
        // the count n is the rank of the last allocated source + 1 (RANK_W = 3 bits, +1 as bit logic): a mux chain, no
        // popcount adder
        for (ac_src = 0; ac_src < NS; ac_src = ac_src + 1)
            // tm_opt iter 5: rank is a Johnson code; its binary rank + 1 (000 -> 1, 001 -> 2, 011 -> 3, 111 -> 4,
            // 110 -> 5) in three AND / OR bits
            alloc_cnt = alloc[ac_src] ? {rank[ac_src][2], ~rank[ac_src][2] & rank[ac_src][0],
                                         ~rank[ac_src][0] | (rank[ac_src][1] & ~rank[ac_src][2])} : alloc_cnt;
    end
    // tm_opt iter 12 (new signals, user approved): the next free count as one parallel select.
    // n (entries allocated now) comes as one-hot from the thermometer rank_alloc, not from the
    // 5-deep alloc_cnt chain; the S2 free bit is s2_cpu_valid & s2_free_entry & (s2_seat < NE),
    // which equals |freed_now without the OR tree after the decode. The values free count + 1 - n
    // and free count - n come from the flop alone (early); the late bits only select.
    reg  [TRK_CNT_W-1:0] free_count_pick;                        // the next free count, 4 bits
    reg                  s2_frees;                               // S2 frees an entry (= |freed_now)
    integer fc_count;                                            // loop variable: n
    // tm_opt iter 13 (new signals, user approved): the next thermometer of the free count, built
    // like free_count_pick: both candidate counts (modulo 16, as the flop keeps them) and their
    // thermometers come from the flop alone (early); s2_frees and n only select, last
    reg  [TRK_CNT_W-1:0] count_plus;                             // free_count_q + 1 - n, 4 bits
    reg  [TRK_CNT_W-1:0] count_minus;                            // free_count_q - n, 4 bits
    reg  [NE:0]          free_room_next;                         // thermometer of free_count_next, 9 bits
    // tm_opt iter 14 (new signals, user approved): the next thermometer WITHOUT adders. Bit i of
    // thermometer(fc - n) is (i + n < fc), i.e. bit i + n of thermometer(fc): a shift by the
    // constant n is wiring. In iter 13 the 4-bit +1-n / -n adders (HA / XOR carry chain, 191 ps)
    // sat in front of the shift. Only the wrap cases (modulo 16, as the flop counts) need a fix,
    // from early bits of room_now. Exact for the bits 0 .. NA that rank_in_room reads, in every
    // state (the upper bits are never read).
    reg  [NE:0]          room_now;                               // thermometer of free_count_q, 9 bits
    reg  [NE:0]          room_plus;                              // thermometer of free_count_q + 1 - n, 9 bits
    reg  [NE:0]          room_minus;                             // thermometer of free_count_q - n, 9 bits
    always @* begin
        s2_frees        = s2_cpu_valid && s2_free_entry && (s2_seat < NE);
        free_count_pick = {TRK_CNT_W{1'b0}};
        free_room_next  = {(NE+1){1'b0}};
        room_now        = ~({(NE+1){1'b1}} << free_count_q);   // decode of the flop, early
        for (fc_count = 0; fc_count <= NA; fc_count = fc_count + 1) begin
            count_plus  = free_count_q + 1 - fc_count;
            count_minus = free_count_q - fc_count;
            // fc - n: below 0 (fc < n) wraps to 16 - .. >= 12: all ones; else shift right by n
            if ((fc_count > 0) && !room_now[(fc_count > 0) ? fc_count - 1 : 0])
                room_minus = {(NE+1){1'b1}};
            else
                room_minus = room_now >> fc_count;
            // fc + 1 - n: n = 0 is (room << 1) | 1, except fc = 15 + 1 wraps to 0; n >= 2 with
            // fc + 1 < n wraps to >= 13: all ones; else shift right by n - 1
            if (fc_count == 0)
                room_plus = (free_count_q == {TRK_CNT_W{1'b1}}) ? {(NE+1){1'b0}} :
                            ((room_now << 1) | {{NE{1'b0}}, 1'b1});
            else if ((fc_count > 1) && !room_now[(fc_count > 1) ? fc_count - 2 : 0])
                room_plus = {(NE+1){1'b1}};
            else
                room_plus = room_now >> ((fc_count > 0) ? fc_count - 1 : 0);
            if (((fc_count == 0)  || rank_alloc[(fc_count > 0) ? fc_count - 1 : 0]) &&
                ((fc_count == NA) || !rank_alloc[(fc_count < NA) ? fc_count : 0])) begin
                free_count_pick = free_count_pick | (s2_frees ? count_plus : count_minus);
                free_room_next  = free_room_next | (s2_frees ? room_plus : room_minus);
            end
        end
    end
    // the registered thermometer; at reset free_count_q = NE, so bits 0 .. NE-1 are set
    always @(posedge clk) begin : free_room_proc
        if (rst) begin
            free_room_q     <= ~({(NE+1){1'b1}} << NE_C);
            free_room_dup_q <= ~({(NE+1){1'b1}} << NE_C);
        end else begin
            free_room_q     <= free_room_next;
            free_room_dup_q <= free_room_next;
        end
    end
    wire [TRK_CNT_W-1:0] free_count_next;                        // 4 bits
    // tm_opt iter 3: the late alloc_cnt (0..7, RANK_W = 3 bits) only selects one of the 8 values free count + freed - k,
    // which are built early from the flop and the S2 pins (same modulo-16 values as the subtract it replaces)
    // tm_opt iter 12: the parallel select above (same values for every n = 0 .. NA that can occur;
    // alloc_cnt stays for the debug text only)
    assign free_count_next = free_count_pick;

    // ======================================================================
    // tm_opt iter 11 (new signals and register, user approved): THE FREE PICK OF THE NEXT CYCLE
    // ======================================================================
    // Next free set = (free entries without the n allocated now) + the entry S2 frees (at most one).
    // The n allocated entries are the n lowest free ones (iter 6: rank k takes free_pick[k]), so
    // the k-th entry of the next set is, with c = free entries left below the freed entry f:
    //   f itself                 when f exists and c == k
    //   the (k-1+n)-th free now  when f exists and c <  k
    //   the (k+n)-th free now    otherwise
    // "k-th free now" for k = 0 .. 2*NA-1 comes from free_ge, a thermometer count of the free
    // entries below each entry (AND / OR only, no adder cells): free_ge[e][j] = at least j.
    // c = max(free entries below f - n, 0), as the n allocated ones are the n LOWEST free entries,
    // so c >= m (m >= 1) is free_ge of f at m + n.
    // Example (NE = 8): free = {1, 3, 4, 6}, n = 2 (entries 1, 3 allocated), S2 frees entry 2:
    //   cnt below 2 = 1 (entry 1), c = max(1 - 2, 0) = 0, so pick 0 = entry 2, pick 1 = 4th free
    //   (k-1+n = 2: entry 4), pick 2 = entry 6, pick 3 = none: next free set {2, 4, 6}. Right.
    localparam GE_W = 2 * NA + 1;                                // thermometer width, 0 .. 2*NA, 9 bits
    reg  [GE_W-1:0]  free_ge [0:NE-1];                           // free entries below e, at least j, 8 x 9 bits
    reg  [2*NA*NE-1:0] free_rank;                                // one-hot k-th free entry now, k < 2*NA, 8 x 8 bits
    // TIMING OPT (CB-2c, 2026-11-26): carry-select on s2_free_entry. WHICH entry S2 would free is known from flops; only WHETHER is
    //   late. Both picks are built early, s2_free_entry selects last (was about 8 levels after the S2 decision). Exact (Shannon expansion).
    // Example: free = {1,3,4,6}, n = 2, S2 entry 2: if_freed = {2,4,6,-}, if_kept = {4,6,-,-}; S2 answers -> {2,4,6,-}.
    reg  [NE-1:0]    s2_freeable_onehot;                         // CB-2c: the valid entry in S2 (freed IF S2 decides so), 8 bits
    reg  [GE_W-1:0]  s2_freeable_ge;                             // CB-2c: its free_ge (0 when none), flops only, 9 bits
    reg              s2_freeable_any;                            // CB-2c: such an entry exists
    reg  [NA*NE-1:0] free_pick_if_freed;                         // CB-2c: free_pick_next if S2 frees it, 4 x 8 bits
    reg  [NA*NE-1:0] free_pick_if_kept;                          // CB-2c: free_pick_next if not, 4 x 8 bits
    reg  [NA:0]      alloc_n_onehot;                             // n allocated this cycle as one-hot, 0 .. NA, 5 bits
    reg  [NA*NE-1:0] free_pick_next;                             // the pick of the next free set, 4 x 8 bits
    integer fg_entry;                                            // loop variable: entry
    integer fg_lower;                                            // loop variable: a lower entry
    integer fr_rank;                                             // loop variable: rank k of the free entry
    integer fn_pick;                                             // loop variable: rank k of the next pick
    integer fn_count;                                            // loop variable: n, entries allocated now

    // ---- tm_opt iter 14 (new signals, user approved): the free counts as a PARALLEL prefix ----
    // Kogge-Stone prefix count: step s adds, to every entry, the count of the entry 2^(s-1) below,
    // so after KS_LV = log2(NE) steps entry e holds the count of entries 0 .. e (inclusive).
    // A step is one thermometer merge, c[j] = OR over i of a[i] & b[j-i] (2 gate levels), so the
    // count takes 3 merges for NE = 8 instead of the 7-step ripple of fg_lower (166 ps in iter 12).
    // Bits above GE_W-1 are dropped: "at least j" saturates, which keeps every bit below exact.
    localparam KS_LV = $clog2(NE);                               // prefix steps, 3
    reg  [GE_W-1:0] ks_ge [0:(KS_LV+1)*NE-1];                    // thermometer per step and entry, 4 x 8 x 9 bits
    integer ks_lv;                                               // loop variable: step
    integer ks_entry;                                            // loop variable: entry
    integer ks_bit;                                              // loop variable: thermometer bit j of the sum
    integer ks_part;                                             // loop variable: bit i taken from the entry itself
    always @* begin
        // step 0: each entry alone (bit 0 = at least 0 = 1, bit 1 = the entry is free)
        for (ks_entry = 0; ks_entry < NE; ks_entry = ks_entry + 1)
            ks_ge[ks_entry] = {{(GE_W-2){1'b0}}, !valid_dup_q[ks_entry], 1'b1};
        // steps 1 .. KS_LV: merge with the entry 2^(step-1) below, where there is one
        for (ks_lv = 1; ks_lv <= KS_LV; ks_lv = ks_lv + 1)
            for (ks_entry = 0; ks_entry < NE; ks_entry = ks_entry + 1) begin
                ks_ge[ks_lv*NE + ks_entry] = {GE_W{1'b0}};
                for (ks_bit = 0; ks_bit < GE_W; ks_bit = ks_bit + 1)
                    for (ks_part = 0; ks_part <= ks_bit; ks_part = ks_part + 1)
                        // below the first 2^(step-1) entries there is nothing to add: the other
                        // operand is the count 0 (only bit 0 set); the index is held at 0 there
                        ks_ge[ks_lv*NE + ks_entry][ks_bit] = ks_ge[ks_lv*NE + ks_entry][ks_bit] |
                            (ks_ge[(ks_lv-1)*NE + ks_entry][ks_part] &
                             ((ks_entry >= (1 << (ks_lv-1))) ?
                                  ks_ge[(ks_lv-1)*NE + ((ks_entry >= (1 << (ks_lv-1))) ? ks_entry - (1 << (ks_lv-1)) : 0)][ks_bit-ks_part] :
                                  (ks_bit == ks_part)));
            end
    end

    always @* begin
        // thermometer counts and the k-th free entry now (from the valid flops only)
        for (fg_entry = 0; fg_entry < NE; fg_entry = fg_entry + 1) begin
            // tm_opt iter 14: free entries below e = the inclusive prefix of entry e-1 (parallel, above)
            free_ge[fg_entry] = (fg_entry == 0) ? {{(GE_W-1){1'b0}}, 1'b1} :
                                ks_ge[KS_LV*NE + ((fg_entry > 0) ? fg_entry - 1 : 0)];
            for (fr_rank = 0; fr_rank < 2 * NA; fr_rank = fr_rank + 1)
                free_rank[fr_rank*NE + fg_entry] = !valid_dup_q[fg_entry] && free_ge[fg_entry][fr_rank] &&
                                                   !free_ge[fg_entry][fr_rank+1];
        end
        // the entry S2 frees IF it decides so (CB-2c: from flops only, s2_entry_onehot is the seat decode): its thermometer count.
        // Only a VALID entry is really freed (tracker_proc clears valid_q only when it is set), so it is masked with valid_q:
        // an S2 free of an already free entry changes nothing, as in tracker_proc
        s2_freeable_onehot = s2_entry_onehot & valid_q;
        s2_freeable_any    = |s2_freeable_onehot;
        s2_freeable_ge     = {GE_W{1'b0}};
        for (fg_entry = 0; fg_entry < NE; fg_entry = fg_entry + 1)
            if (s2_freeable_onehot[fg_entry])
                s2_freeable_ge = s2_freeable_ge | free_ge[fg_entry];
        // n as one-hot from the thermometer rank_alloc (rank k taken <=> n > k)
        for (fn_count = 0; fn_count <= NA; fn_count = fn_count + 1)
            alloc_n_onehot[fn_count] = ((fn_count == 0) || rank_alloc[(fn_count > 0) ? fn_count - 1 : 0]) &&
                                       ((fn_count == NA) || !rank_alloc[(fn_count < NA) ? fn_count : 0]);
        // the pick of the next free set, one AND-OR over n, built twice (CB-2c): if S2 frees its entry, and if it does not;
        // s2_free_entry (the late S2 decision) picks between the two last
        free_pick_if_freed = {NA*NE{1'b0}};
        free_pick_if_kept  = {NA*NE{1'b0}};
        for (fn_pick = 0; fn_pick < NA; fn_pick = fn_pick + 1)
            for (fn_count = 0; fn_count <= NA; fn_count = fn_count + 1)
                if (alloc_n_onehot[fn_count]) begin
                    // S2 frees its entry: the three cases of the iter 11 comment
                    if (s2_freeable_any && ((fn_pick == 0) || s2_freeable_ge[fn_pick+fn_count]) && !s2_freeable_ge[fn_pick+fn_count+1])
                        // c == k: the freed entry
                        free_pick_if_freed[fn_pick*NE +: NE] = free_pick_if_freed[fn_pick*NE +: NE] | s2_freeable_onehot;
                    else if (s2_freeable_any && (fn_pick > 0) && !s2_freeable_ge[fn_pick+fn_count])
                        // c < k: the freed entry lies below, the pick moves down one rank
                        // (the index is held at fn_count for k = 0, where this branch is never taken)
                        free_pick_if_freed[fn_pick*NE +: NE] = free_pick_if_freed[fn_pick*NE +: NE] |
                            free_rank[(((fn_pick > 0) ? fn_pick - 1 : 0) + fn_count)*NE +: NE];
                    else if (fn_pick + fn_count < 2 * NA)
                        free_pick_if_freed[fn_pick*NE +: NE] = free_pick_if_freed[fn_pick*NE +: NE] |
                            free_rank[(fn_pick+fn_count)*NE +: NE];
                    // S2 keeps it (nothing freed): only the third case
                    if (fn_pick + fn_count < 2 * NA)
                        free_pick_if_kept[fn_pick*NE +: NE] = free_pick_if_kept[fn_pick*NE +: NE] |
                            free_rank[(fn_pick+fn_count)*NE +: NE];
                end
        free_pick_next = s2_free_entry ? free_pick_if_freed : free_pick_if_kept;   // the late S2 decision, last 2:1
    end
    // the registered pick; at reset no entry is valid, so pick k is entry k
    integer fq_pick;                                             // loop variable: rank k
    always @(posedge clk) begin : free_pick_proc
        if (rst) begin
            for (fq_pick = 0; fq_pick < NA; fq_pick = fq_pick + 1)
                free_pick_q[fq_pick*NE +: NE] <= {{(NE-1){1'b0}}, 1'b1} << fq_pick;
        end else begin
            free_pick_q <= free_pick_next;
        end
    end

    integer ent;                                                 // loop variable: entry
    always @(posedge clk) begin : tracker_proc
        if (rst) begin
            valid_q      <= {NE{1'b0}};
            valid_dup_q  <= {NE{1'b0}};                          // tm_opt iter 16: the copy, same value
            write_q      <= {NE{1'b0}};
            wait_nc_q    <= {NE{1'b0}};
            trained_q    <= {NE{1'b0}};
            free_count_q <= NE_C;
            for (ent = 0; ent < NE; ent = ent + 1) begin
                state_q[ent]     <= {TRK_STATE_W{1'b0}};
                deps_q[ent]      <= {NE{1'b0}};
                wait_mshr_q[ent] <= {MSHR_ENTRIES{1'b0}};
                wait_vc_q[ent]   <= {VC_SLOTS{1'b0}};
            end
        end else begin
            free_count_q <= free_count_next;
            // tm_opt iter 16 retry: the copy follows valid_q every cycle (the alloc / free writes below override it), so it
            // equals valid_q one cycle after any state; this makes the equivalence induction close
            valid_dup_q  <= valid_q;
            for (ent = 0; ent < NE; ent = ent + 1) begin
                // wide data of a FREE entry loads every cycle (no late enable); valid / state / deps use the late enables
                if (!valid_q[ent]) begin
                    packet_q[ent] <= load_packet[ent];
                    pc_q[ent]     <= load_pc[ent];
                    set_q[ent]    <= load_packet[ent][SET_IN_REQ +: INDEX_W];
                    write_q[ent]  <= load_packet[ent][REQ_RDWR_BIT];
                end
                if (alloc_en[ent]) begin
                    valid_q[ent]     <= 1'b1;
                    valid_dup_q[ent] <= 1'b1;
                    trained_q[ent]   <= 1'b0;
                    wait_nc_q[ent]   <= 1'b0;
                    wait_mshr_q[ent] <= {MSHR_ENTRIES{1'b0}};
                    wait_vc_q[ent]   <= {VC_SLOTS{1'b0}};
                    state_q[ent]     <= {TRK_STATE_W{1'b0}};
                    if (fresh_issue && fresh_entry_onehot[ent])
                        state_q[ent][TRK_INFL_B]   <= 1'b1;      // served on trust: the R check follows
                    else if (book_entry_onehot[ent])
                        state_q[ent][TRK_BOOKED_B] <= 1'b1;      // an arrival booked in its arrival cycle (issues next cycle)
                    else
                        state_q[ent][TRK_READY_B]  <= 1'b1;
                    // the deps of this source (the source whose entry this is)
                    // tm_opt iter 6: one AND-OR by rank (deps_load), not the 5-deep select
                    deps_q[ent] <= deps_load[ent];
                end else if (valid_q[ent]) begin
                    deps_q[ent] <= deps_q[ent] & ~freed_now;     // an older entry left: the dependency on it ends
                    if (s2_entry_onehot[ent]) begin
                        if (s2_free_entry) begin
                            valid_q[ent] <= 1'b0;
                            valid_dup_q[ent] <= 1'b0;
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                        end else if (s2_to_ready) begin
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                            state_q[ent][TRK_READY_B] <= 1'b1;
                        end else if (s2_to_wait) begin
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                            if (push_ready_now) state_q[ent][TRK_READY_B] <= 1'b1;
                            else                state_q[ent][TRK_WAIT_B]  <= 1'b1;
                            wait_mshr_q[ent] <= push_mshr_mask;
                            wait_vc_q[ent]   <= push_vc_mask;
                            wait_nc_q[ent]   <= push_nc_bit;
                        end
                        if (s2_train_sent) trained_q[ent] <= 1'b1;
                    end else begin
                        // wake-up of a waiting entry
                        if (state_q[ent][TRK_WAIT_B] &&
                            ((|(wait_mshr_q[ent] & mshr_freed)) || (|(wait_vc_q[ent] & vc_freed)) || (wait_nc_q[ent] && nc_freed))) begin
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                            state_q[ent][TRK_READY_B] <= 1'b1;
                        end
                        // booked (late in c-1): READY -> BOOKED; issued (c): BOOKED -> INFL
                        if (book_ready_onehot[ent] && state_q[ent][TRK_READY_B]) begin  // Vivado iter 1: early leg (in-use entry)
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                            state_q[ent][TRK_BOOKED_B] <= 1'b1;
                        end
                        if (issue_entry_onehot[ent]) begin
                            state_q[ent] <= {TRK_STATE_W{1'b0}};
                            state_q[ent][TRK_INFL_B] <= 1'b1;
                        end
                    end
                end
            end
        end
    end // always tracker_proc

    assign entry_valid_q = valid_q;
    assign tracker_empty = ~|valid_q;

    // ======================================================================
    // SIMULATION CHECKS (always on): the assumptions of the header and the invariants of the allocation
    // ======================================================================
`ifndef SYNTHESIS
    integer chk_src;                                             // loop variable: source
    integer chk_entry;                                           // loop variable: entry
    integer chk_hits;                                            // sources that chose one entry
    initial if (LSU_PORTS != 4)
        $display("L1D TRACKER ERROR bank %0d: LSU_PORTS = %0d, the ports _0 .. _3 need LSU_PORTS = 4", BANK_ID, LSU_PORTS);
    always @(posedge clk) begin
        if (!rst) begin
            if (drain_request && any_pins_arrival)
                $display("[%0t] L1D TRACKER ERROR bank %0d: pins arrival and drain in the same cycle (front rule A1 broken)", $time, BANK_ID);
            for (chk_entry = 0; chk_entry < NE; chk_entry = chk_entry + 1) begin
                chk_hits = 0;
                for (chk_src = 0; chk_src < NS; chk_src = chk_src + 1)
                    if (alloc[chk_src] && src_entry[chk_src][chk_entry]) chk_hits = chk_hits + 1;
                if (chk_hits > 1)
                    $display("[%0t] L1D TRACKER ERROR bank %0d: entry %0d allocated by %0d sources at once", $time, BANK_ID, chk_entry, chk_hits);
                if (alloc_en[chk_entry] && valid_q[chk_entry])
                    $display("[%0t] L1D TRACKER ERROR bank %0d: entry %0d allocated while in use", $time, BANK_ID, chk_entry);
            end
            for (chk_src = 0; chk_src < NS; chk_src = chk_src + 1)
                if (alloc[chk_src] && (src_entry[chk_src] == {NE{1'b0}}))
                    $display("[%0t] L1D TRACKER ERROR bank %0d: source %0d allocated without a free entry (request lost)", $time, BANK_ID, chk_src);
            if (s2_cpu_valid && !valid_q[s2_seat])
                $display("[%0t] L1D TRACKER ERROR bank %0d: S2 result for entry %0d, which is free", $time, BANK_ID, s2_seat);
            if (s2_cpu_valid && !state_q[s2_seat][TRK_INFL_B])
                $display("[%0t] L1D TRACKER ERROR bank %0d: S2 result for entry %0d, which is not in flight", $time, BANK_ID, s2_seat);
            if ((|book_entry_onehot) && (|(book_entry_onehot & issue_entry_onehot)))
                $display("[%0t] L1D TRACKER ERROR bank %0d: an entry is booked and issued in one cycle", $time, BANK_ID);
        end
    end
`endif
`ifdef L1D_DEBUG
    always @(posedge clk) begin
        if (!rst) begin
            for (chk_src = 0; chk_src < NS; chk_src = chk_src + 1)
                if (alloc[chk_src])
                    $display("[%0t] L1D TRACKER bank %0d: source %0d (port %0d) -> entry %b rank %0d%s deps=%b", $time, BANK_ID, chk_src,
                             src_packet[chk_src*REQ_W + REQ_PORT_LSB +: PORT_W], src_entry[chk_src], rank[chk_src],
                             (fresh_issue && rank[chk_src] == 0 && chk_src < LSU_PORTS) ? " FRESH" : "", deps_new[chk_src]);
            if (|freed_now)
                $display("[%0t] L1D TRACKER bank %0d: entry %0d freed (free next = %0d)", $time, BANK_ID, s2_seat, free_count_next);
        end
    end
`endif

endmodule

`default_nettype wire
`endif // L1D_REQ_TRACKER_V
