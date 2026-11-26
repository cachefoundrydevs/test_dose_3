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
    credit_q_3, fresh_issue, fresh_port_onehot, fresh_entry_onehot, any_pins_arrival, arrival_allocated_0,
    arrival_allocated_1, arrival_allocated_2, arrival_allocated_3, drain_allocated, oldest_ready_valid, oldest_ready_onehot,
    oldest_ready_set, oldest_ready_line, oldest_ready_port, oldest_ready_is_write, oldest_ready_is_line_op, oldest_ready_is_amo,
    arrival_book_valid, arrival_book_entry_onehot, arrival_book_set, arrival_book_line, arrival_book_port, arrival_book_is_write,
    arrival_book_is_line_op, arrival_book_is_amo, book_entry_onehot, issue_entry_onehot, entry_packet_flat, entry_pc_flat,
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
    input  wire [TRK_ENTRIES-1:0]      book_entry_onehot;        // the booking takes this entry for the next cycle (late in c-1)
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
            for (rank_lower = 0; rank_lower < rank_src; rank_lower = rank_lower + 1)
                rank[rank_src] = rank[rank_src] + {{(RANK_W-1){1'b0}}, eligible[rank_lower]};
        end
    end

    // ---- allocation: an eligible source gets an entry when its rank is below the free count and below TRK_ALLOC_MAX ----
    wire [NS-1:0] alloc;                                         // source takes an entry this cycle, 5 bits
    genvar al_idx;                                               // loop variable: source
    generate
        for (al_idx = 0; al_idx < NS; al_idx = al_idx + 1) begin : g_alloc
            wire rank_in_room;                                   // rank < free entries
            wire rank_in_max;                                    // rank < TRK_ALLOC_MAX
            assign rank_in_room = ({{(TRK_CNT_W-RANK_W){1'b0}}, rank[al_idx]} < free_count_q);
            assign rank_in_max  = (rank[al_idx] < NA);
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
            reg [RANK_W-1:0] free_below [0:NE-1];                // free entries below e, 8 x 3 bits
            integer fb_entry;                                    // loop variable: entry
            integer fb_lower;                                    // loop variable: a lower entry
            always @* begin
                for (fb_entry = 0; fb_entry < NE; fb_entry = fb_entry + 1) begin
                    free_below[fb_entry] = {RANK_W{1'b0}};
                    for (fb_lower = 0; fb_lower < fb_entry; fb_lower = fb_lower + 1)
                        free_below[fb_entry] = free_below[fb_entry] + {{(RANK_W-1){1'b0}}, ~valid_q[fb_lower]};
                end
            end
            genvar pk_k, pk_e;                                   // loop variables: rank k, entry e
            for (pk_k = 0; pk_k < NA; pk_k = pk_k + 1) begin : g_pick_k
                for (pk_e = 0; pk_e < NE; pk_e = pk_e + 1) begin : g_pick_e
                    assign free_pick[pk_k*NE + pk_e] = ~valid_q[pk_e] & (free_below[pk_e] == pk_k);
                end
            end
        end
    endgenerate

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
            for (se_rank = 0; se_rank < NA; se_rank = se_rank + 1)
                if (rank[se_src] == se_rank)
                    src_entry[se_src] = src_entry[se_src] | free_pick[se_rank*NE +: NE];
        end
        for (se_entry = 0; se_entry < NE; se_entry = se_entry + 1) begin
            alloc_en[se_entry] = 1'b0;
            for (se_src = 0; se_src < NS; se_src = se_src + 1) begin
                data_sel[se_entry][se_src] = eligible[se_src] & src_entry[se_src][se_entry];
                alloc_en[se_entry] = alloc_en[se_entry] | (alloc[se_src] & src_entry[se_src][se_entry]);
            end
        end
        for (se_rank = 0; se_rank < NA; se_rank = se_rank + 1) begin
            age_groups[se_rank*NE +: NE] = {NE{1'b0}};
            for (se_src = 0; se_src < NS; se_src = se_src + 1)
                if (alloc[se_src] && (rank[se_src] == se_rank))
                    age_groups[se_rank*NE +: NE] = age_groups[se_rank*NE +: NE] | src_entry[se_src];
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
            lowest_req[lo_port]  = src_valid[lo_port] & !lower_req_seen;
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
    always @* begin
        for (dp_src = 0; dp_src < NS; dp_src = dp_src + 1) begin
            for (dp_entry = 0; dp_entry < NE; dp_entry = dp_entry + 1)
                deps_new[dp_src][dp_entry] = valid_q[dp_entry] & ~freed_now[dp_entry] &
                                             (set_q[dp_entry] == src_set[dp_src*INDEX_W +: INDEX_W]) &
                                             (write_q[dp_entry] | src_write[dp_src]);
            for (dp_lower = 0; dp_lower < dp_src; dp_lower = dp_lower + 1) begin
                same_set_pair = alloc[dp_lower] &&
                                (src_set[dp_lower*INDEX_W +: INDEX_W] == src_set[dp_src*INDEX_W +: INDEX_W]) &&
                                (src_write[dp_lower] || src_write[dp_src]);
                if (same_set_pair)
                    deps_new[dp_src] = deps_new[dp_src] | src_entry[dp_lower];
            end
        end
    end

    // ---- arrival booking candidate: the lowest allocated source without dependencies that is not the fresh one ----
    reg  [NS-1:0] arrival_cand;                                  // sources that may be booked, 5 bits
    reg  [NS-1:0] arrival_pick;                                  // the lowest of them, one-hot, 5 bits
    reg           arrival_seen;                                  // a lower candidate was found
    integer ab_src;                                              // loop variable: source
    always @* begin
        arrival_seen = 1'b0;
        for (ab_src = 0; ab_src < NS; ab_src = ab_src + 1) begin
            arrival_cand[ab_src] = alloc[ab_src] && (deps_new[ab_src] == {NE{1'b0}}) &&
                                   !(fresh_issue && (rank[ab_src] == {RANK_W{1'b0}}) && (ab_src < LSU_PORTS));
            arrival_pick[ab_src] = arrival_cand[ab_src] && !arrival_seen;
            arrival_seen         = arrival_seen || arrival_cand[ab_src];
        end
    end
    reg  [NE-1:0]          ab_entry;                             // its entry, 8 bits
    reg  [REQ_W-1:0]       ab_packet;                            // its packet, 148 bits
    integer ab_mux;                                              // loop variable: source
    always @* begin
        ab_entry  = {NE{1'b0}};
        ab_packet = {REQ_W{1'b0}};
        for (ab_mux = 0; ab_mux < NS; ab_mux = ab_mux + 1) begin
            ab_entry  = ab_entry  | ({NE{arrival_pick[ab_mux]}}    & src_entry[ab_mux]);
            ab_packet = ab_packet | ({REQ_W{arrival_pick[ab_mux]}} & src_packet[ab_mux*REQ_W +: REQ_W]);
        end
    end
    assign arrival_book_valid        = |arrival_pick;
    assign arrival_book_entry_onehot = ab_entry;
    assign arrival_book_set          = ab_packet[SET_IN_REQ +: INDEX_W];
    assign arrival_book_line         = ab_packet[LINE_IN_REQ +: LINE_ADDR_W];
    assign arrival_book_port         = ab_packet[REQ_PORT_LSB +: PORT_W];
    assign arrival_book_is_write     = ab_packet[REQ_RDWR_BIT];
    assign arrival_book_is_line_op   = ab_packet[REQ_LINE_BIT];
    assign arrival_book_is_amo       = ab_packet[REQ_AMO_BIT] & ab_packet[REQ_SEQ_BIT];

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
        for (ac_src = 0; ac_src < NS; ac_src = ac_src + 1)
            alloc_cnt = alloc_cnt + {{(RANK_W-1){1'b0}}, alloc[ac_src]};
    end
    wire [TRK_CNT_W-1:0] free_count_next;                        // 4 bits
    assign free_count_next = free_count_q - {{(TRK_CNT_W-RANK_W){1'b0}}, alloc_cnt} + {{(TRK_CNT_W-1){1'b0}}, |freed_now};

    integer ent;                                                 // loop variable: entry
    always @(posedge clk) begin : tracker_proc
        if (rst) begin
            valid_q      <= {NE{1'b0}};
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
                    deps_q[ent] <= {NE{1'b0}};
                    for (ac_src = 0; ac_src < NS; ac_src = ac_src + 1)
                        if (alloc[ac_src] && src_entry[ac_src][ent])
                            deps_q[ent] <= deps_new[ac_src];
                end else if (valid_q[ent]) begin
                    deps_q[ent] <= deps_q[ent] & ~freed_now;     // an older entry left: the dependency on it ends
                    if (s2_entry_onehot[ent]) begin
                        if (s2_free_entry) begin
                            valid_q[ent] <= 1'b0;
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
                        if (book_entry_onehot[ent] && state_q[ent][TRK_READY_B]) begin
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
