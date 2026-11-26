// ============================================================================
// l1d_bank.v  -  ONE BANK of the L1 data cache (cache_v6: tracker S0, ECC, prefetch)
// ============================================================================
// IN SIMPLE WORDS
//   The cache is split into NUM_BANKS (4) independent banks, selected by the address bits just above the line offset.
//   l1d_top places four copies of this file in a generate loop (BANK_ID = 0..3). A bank is a small complete cache
//   with its own tag RAM, data RAM, MESI / PLRU / pf flops, request tracker, miss handling (MSHR), victim cache, a snoop
//   buffer, and the LR/SC reservation unit. Requests that arrive go through a 4-step PIPELINE.
//
// THE PIPELINE   (one operation can enter per cycle; nothing stalls once it has entered)
//
//   G   (cache_v6, PLAN_v6.md section 4, l1d_bank_s0) Every CPU request of this bank gets an entry in the REQUEST TRACKER
//       (8 per bank) in its arrival cycle, up to 4 in one cycle (one per port); the lowest port issues FRESH at once when
//       nothing is booked and its port has a response credit. Internal work (snoop > refill token > flush/clean token >
//       oldest READY entry > an arrival that could not issue > prefetch hint) is BOOKED one cycle early from flops. The set
//       address goes to the tag and data SRAMs through one 2:1 select (booked set, or the lowest arriving port's set).
//       Same-set dependencies are computed once at allocation and kept as bits; a fresh request is checked in stage R.
//   R   (l1d_bank_r) The SRAMs are read; their coded outputs (4 tags with 7 check bits, 4 lines with 64 check bits) are
//       REGISTERED at the end of R. The R CHECK cancels a fresh request with a dependency bit set or whose set was written at
//       its read edge: it runs on as a ghost. The SRAM write port (refill, store, ECC scrub, S3 AMO) with its ECC encoders.
//   S1  (l1d_bank_s1) Tag compare in all ways (hit?) on the registered tags; state, PLRU and pf bits read and written here.
//       The load word of EVERY way is cut out beside the compare. 32 + 4 SYNDROME units check the registered words; a tag
//       error in a valid way blocks the request. A refill picks its victim way and a victim-cache slot (held if none).
//   S2  (l1d_bank_s2) CPU hits pick their word with the registered hit way and answer AT ONCE (pushed into the response
//       merger this cycle) when the hit way is clean; a correctable error is scrubbed (corrected line written) and the
//       request re-issued from its entry; a store hit writes its merged line; misses: MSHR, then victim cache, then a new
//       MSHR entry. Prefetch ops are decided here (drop / prefetch MSHR entry) with feedback; the training record and the
//       eviction event for the prefetch unit are made here. Snoops, refill installs, victim inserts, reservation events.
//   Requests that cannot finish now WAIT IN THEIR TRACKER ENTRY (or the snoop buffer entry) and are booked again after a
//   wake-up; the CPU never sees a retry. Load / store hit at the bank pins: 4 cycles (c0 issue, c0+1 R, c0+2 S1, c0+3 S2 ->
//   output register, valid at c0+4). Latency table: PLAN_v6.md 4.7.
//
// SPLIT INTO ONE MODULE PER STAGE (cache_v5 C-21; cache_v6 keeps it)
//   This file is a THIN WRAPPER: the stages are their own modules, so each one can be synthesised, placed and routed and
//   timed alone (TOP=<module> in flow/run_pnr.sh) and a GR-1 gap can be named per stage.
//     u_s0       l1d_bank_s0       stage G: request tracker, booking, address lane, decision lane, credits, G -> R
//     u_r        l1d_bank_r        stage R: tag / data SRAMs (coded words), ECC encoders of the write port, R check, R -> S1
//     u_s1       l1d_bank_s1       stage S1: tag compare, MESI / PLRU / pf bits, victim choice, SC check, syndromes, S1 -> S2
//     u_s2       l1d_bank_s2       stage S2: decisions, ECC scrub, prefetch decision, training / eviction / feedback, S2 -> S3
//     u_s3       l1d_bank_s3       S3 AMO merge, response arbiter (pipeline / MSHR / NC), memory channels
//   The miss units keep their own modules and are placed here too: u_rsv (reservation / AMO lock), u_mshr, u_nc (non-
//   cacheable queue), u_vc (victim cache), u_maint (flush / clean walker), u_vec_buf (vector store data places). The only
//   logic left in this file: bit fields taken out of s2_req for the miss units, the read-beat / write-response routing to
//   the MSHR / NC / VC, and the bank-idle AND for the flush walker.
//
// HOW TO READ THE INTERNAL SIGNAL NAMES  (the PORTS carry long names; internal wires keep the short stage names)
//   r_ s1_ s2_ s3_   register or wire that belongs to pipeline stage R / 1 / 2 / 3 (s1_valid = stage 1 holds an operation)
//   any_gnt, g_set   the SRAM read of this cycle and its set (stage G)
//   gnt_refill / gnt_maint   the refill token / the flush-clean token issues now
//   tok_*            the MSHR REFILL TOKEN (tok_valid, tok_id = MSHR entry, tok_line)
//   mt_*             the flush/clean MAINTENANCE token (mt_valid, mt_set, mt_way, mt_op)
//   act_*            what Stage 2 decided for a CPU request (act_code = ACT_*, act_id = what it waits for); snp_act_* for a snoop
//   m_alloc/m_swap/m_resp_*/m_rd_*  the cacheable MSHR (allocate, swap, its response, its read request)
//   n_*              the non-cacheable queue (n_alloc, n_rd_*, n_wr_*, n_resp_*)
//   vc_*             the victim cache (vc_hit, vc_slot, vc_state, vc_data, vc_ins, vc_swap, vc_wr_*)
//   q1_* q2_*        data of the MSHR entry being INSTALLED, read in Stage 1 (state, swap, slot, prefetch) and Stage 2 (line)
//   ev_*             events for the reservation unit (an LR / SC / store completed)
//   tag_words_q, s1_words   the registered coded SRAM words of all ways (R -> S1)
//   lock_*           the AMO lock of this bank (never set in this version) from l1d_reservation_unit
//
// CREDITS (why no response or snoop answer is ever lost)
//   A CPU request issues only if its port's response FIFO (in l1d_resp_merge) has a free place left after counting the
//   operations already promised (credit_q per port in l1d_bank_s0, from the FIFO's free_slots and this bank's own push, crossbank step 1). MSHR / NC responses
//   need a place beyond those in R / S1 (miss_ok). A snoop is booked only if the snoop-answer FIFO has a free place.
//
// PORTS (name, meaning)                                                        N = 0..3: the request port
//   arrival_request_N / arrival_packet_N / arrival_pc_N   port N's pins request is for this bank (its ring is empty); its packet, PC
//   drain_request / drain_packet / drain_pc              the oldest skidded request for this bank (ring not empty); its packet, PC
//   arrival_allocated_N / drain_allocated                the tracker took it this cycle (the front pushes the others into the skids)
//   tracker_free_count_q                                 flop: free tracker entries (the front's room rule)
//   vec_slot_available                                   a vector store data place is free (the front's room rule for vector stores)
//   lsu_request_line_data                                the line of a vector store arriving at the bank door (C-20)
//   lsu_response_valid / _packet / _line_data            the bank pushes a response into its FIFO in the response merger; the VPU line
//   lsu_response_fifo_free_N / _free_next_N              free places in this bank's response FIFO of port N: now, and after this cycle
//   snoop_request_* / snoop_response_*                   snoops from l1d_snoop_ctrl and the answers back; snoop_response_fifo_free = credit
//   prefetch_request_valid / _line_addr / _unique / _ready  a prefetch hint for this bank (l1d_prefetch_unit); ready = not booked
//   axi_read_* / axi_write_*                             the memory channels to l1d_l2_port (names kept from cache_v2_bank_pipe)
//   refill_handover_busy                                 the L2 port is handing a line to this bank: start no snoop
//   maint_start / maint_op_is_flush / maint_done         start a flush or clean walk of this bank; the walk finished (pulse)
//   amo_lock_timeout_pulse                               one-cycle pulse: an AMO lock timed out (never in this version)
//   rsv_event_pulse / other_bank_rsv_event               an LR or SC completed here / in another bank last cycle (global reservation)
//   train_valid / _port / _pc / _addr / _is_store / _miss / _pf_hit / _no_train   the training record (registered, to the prefetch unit)
//   evict_valid / evict_line_addr                        a line left the array (registered, to the prefetch unit)
//   pf_fb_valid / pf_fb_line_addr / pf_fb_result         the prefetch feedback (registered, to the prefetch unit)
//   ecc_correctable / ecc_uncorrectable                  one-cycle pulses: a scrub was written / an uncorrectable error was seen
//
// VERILOG WORDS USED IN THIS FILE
//   module l1d_bank #(`L1D_PARAMS, parameter BANK_ID = 0) (...)   parameters first, then the port NAMES; each port is
//                                  declared (input / output) in the body ("non-ANSI" style)
//   wire x;                        a connection between two instances (no logic)
//   .port_name (signal)            connects the port of an instance to a signal of this module
//   x[A +: W]                      the W bits of x starting at bit A (a field of a packet)
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

module l1d_bank #(`L1D_PARAMS, parameter BANK_ID = 0) (
    clk, rst,
    arrival_request_0, arrival_request_1, arrival_request_2, arrival_request_3,
    arrival_packet_0, arrival_packet_1, arrival_packet_2, arrival_packet_3,
    arrival_pc_0, arrival_pc_1, arrival_pc_2, arrival_pc_3,
    drain_request, drain_packet, drain_pc,
    arrival_allocated_0, arrival_allocated_1, arrival_allocated_2, arrival_allocated_3, drain_allocated,
    tracker_free_count_q, vec_slot_available,
    lsu_request_line_data,
    lsu_response_valid, lsu_response_packet, lsu_response_line_data,
    lsu_response_fifo_free_0, lsu_response_fifo_free_1, lsu_response_fifo_free_2, lsu_response_fifo_free_3,
    lsu_response_fifo_free_next_0, lsu_response_fifo_free_next_1, lsu_response_fifo_free_next_2, lsu_response_fifo_free_next_3,
    snoop_request_valid, snoop_request_line_addr, snoop_request_invalidate, snoop_request_ready,
    snoop_response_valid, snoop_response_packet, snoop_response_fifo_free,
    prefetch_request_valid, prefetch_request_line_addr, prefetch_request_unique, prefetch_request_ready,
    axi_read_req_valid, axi_read_req_ready, axi_read_req_addr, axi_read_req_burst_len, axi_read_req_size, axi_read_req_sub_id, axi_read_req_non_cacheable, axi_read_req_unique,
    axi_read_data_valid, axi_read_data_sub_id, axi_read_data_non_cacheable, axi_read_data, axi_read_data_last, axi_read_data_error, axi_read_data_shared,
    axi_write_req_valid, axi_write_req_ready, axi_write_req_addr, axi_write_req_burst_len, axi_write_req_size, axi_write_req_sub_id, axi_write_req_non_cacheable, axi_write_req_data, axi_write_req_strobe,
    axi_write_resp_valid, axi_write_resp_sub_id, axi_write_resp_non_cacheable, axi_write_resp_error,
    refill_handover_busy,
    maint_start, maint_op_is_flush, maint_done,
    amo_lock_timeout_pulse,
    rsv_event_pulse, other_bank_rsv_event,
    train_valid, train_port, train_pc, train_addr, train_is_store, train_miss, train_pf_hit, train_no_train,
    evict_valid, evict_line_addr,
    pf_fb_valid, pf_fb_line_addr, pf_fb_result,
    ecc_correctable, ecc_uncorrectable
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"
    localparam S2_WAY_COPIES = GRANULES + 4;                  // CB-3: one-hot copies of s2_way (u_s1 -> u_s2), 12 (same value in l1d_bank_s1 / _s2)

    // ======================================================================
    // Ports
    // ======================================================================
    input  wire                     clk;                  // clock: all registers change on its rising edge
    input  wire                     rst;                  // synchronous reset, active high (control state goes to its idle value)
    // ---- arrivals from the port front (cache_v6) ----
    input  wire                     arrival_request_0;    // port 0's pins request is for this bank (ring empty)
    input  wire                     arrival_request_1;    // port 1
    input  wire                     arrival_request_2;    // port 2
    input  wire                     arrival_request_3;    // port 3 (VPU)
    input  wire [REQ_W-1:0]         arrival_packet_0;     // port 0 request packet, REQ_W = 148 bits
    input  wire [REQ_W-1:0]         arrival_packet_1;     // port 1 packet, 148 bits
    input  wire [REQ_W-1:0]         arrival_packet_2;     // port 2 packet, 148 bits
    input  wire [REQ_W-1:0]         arrival_packet_3;     // port 3 packet, 148 bits
    input  wire [PC_W-1:0]          arrival_pc_0;         // port 0 instruction PC, PC_W = 64 bits
    input  wire [PC_W-1:0]          arrival_pc_1;         // port 1 PC, 64 bits
    input  wire [PC_W-1:0]          arrival_pc_2;         // port 2 PC, 64 bits
    input  wire [PC_W-1:0]          arrival_pc_3;         // port 3 PC, 64 bits (0: the VPU has no PC pin)
    input  wire                     drain_request;        // the oldest skidded request for this bank asks for an entry
    input  wire [REQ_W-1:0]         drain_packet;         // its packet, 148 bits
    input  wire [PC_W-1:0]          drain_pc;             // its PC, 64 bits
    output wire                     arrival_allocated_0;  // the tracker took port 0's arrival this cycle
    output wire                     arrival_allocated_1;  // port 1
    output wire                     arrival_allocated_2;  // port 2
    output wire                     arrival_allocated_3;  // port 3
    output wire                     drain_allocated;      // the tracker took the drained request this cycle
    output wire [TRK_CNT_W-1:0]     tracker_free_count_q; // flop: free tracker entries, TRK_CNT_W = 4 bits
    output wire                     vec_slot_available;   // a vector store data place is free (C-20, the front's room rule)
    // cache_v5 C-20: the line data of the VPU port beside the packets (the packets keep their 64-bit data field)
    input  wire [LINE_W-1:0]        lsu_request_line_data;   // the line of a vector store at the bank door, LINE_W = 512 bits
    // ---- response to this bank's FIFO in the response merger ----
    output wire                     lsu_response_valid;   // the bank pushes a response into its FIFO in the response merger
    output wire [RESP_W-1:0]        lsu_response_packet;  // the response packet, RESP_W = 74 bits
    output wire [LINE_W-1:0]        lsu_response_line_data;  // the line a vector load answers with (valid with its answer), 512 bits
    input  wire [RCNT_W-1:0]        lsu_response_fifo_free_0;  // free places of port 0 response FIFO for this bank, RCNT_W = 3 bits
    input  wire [RCNT_W-1:0]        lsu_response_fifo_free_1;  // ... port 1, 3 bits
    input  wire [RCNT_W-1:0]        lsu_response_fifo_free_2;  // ... port 2, 3 bits
    input  wire [RCNT_W-1:0]        lsu_response_fifo_free_3;  // ... port 3 (VPU), 3 bits
    input  wire [RCNT_W-1:0]        lsu_response_fifo_free_next_0;  // free places of port 0's FIFO after this cycle (the next credit), 3 bits
    input  wire [RCNT_W-1:0]        lsu_response_fifo_free_next_1;  // ... port 1
    input  wire [RCNT_W-1:0]        lsu_response_fifo_free_next_2;  // ... port 2
    input  wire [RCNT_W-1:0]        lsu_response_fifo_free_next_3;  // ... port 3
    // ---- A2 global LR/SC reservation: this bank reports its LR/SC, the top tells it about the LR/SC of the other banks ----
    output wire                     rsv_event_pulse;      // an LR or SC completed in Stage 2 of this bank
    input  wire                     other_bank_rsv_event; // an LR or SC completed in another bank last cycle (registered in l1d_top)
    // ---- snoop request from l1d_snoop_ctrl and the answer back ----
    input  wire                     snoop_request_valid;  // a snoop from the snoop controller
    input  wire [LINE_ADDR_W-1:0]   snoop_request_line_addr;  // line address of the snoop, LINE_ADDR_W = 58 bits
    input  wire                     snoop_request_invalidate;  // 1 = invalidate, 0 = read
    output wire                     snoop_request_ready;  // the snoop buffer has a free entry
    output wire                     snoop_response_valid; // the bank pushes a snoop answer
    output wire [SNPR_W-1:0]        snoop_response_packet;  // the answer packet (present, dirty, line address, line data), 572 bits
    input  wire [SNPR_CNT_W-1:0]    snoop_response_fifo_free;  // free places in the snoop controller's FIFO for this bank, 2 bits
    // ---- prefetch hint (l1d_prefetch_unit through l1d_top) ----
    input  wire                     prefetch_request_valid;  // a prefetch hint for this bank
    input  wire [LINE_ADDR_W-1:0]   prefetch_request_line_addr;  // its line address, 58 bits
    input  wire                     prefetch_request_unique; // it wants the line unique (store-line prefetch)
    output wire                     prefetch_request_ready;  // a new hint may replace the waiting one (0 while it is booked)
    // ---- memory read channel (to l1d_l2_port) ----
    output wire                     axi_read_req_valid;   // the bank offers a read request
    input  wire                     axi_read_req_ready;   // the L2 port took it
    output wire [AXI_ADDR_W-1:0]    axi_read_req_addr;    // read address, AXI_ADDR_W = 64 bits
    output wire [7:0]               axi_read_req_burst_len;  // beats minus one
    output wire [2:0]               axi_read_req_size;    // beat size
    output wire [AXI_SUBID_W-1:0]   axi_read_req_sub_id;  // MSHR entry number (0 for non-cacheable), part of the id, 3 bits
    output wire                     axi_read_req_non_cacheable;  // 1 = non-cacheable access, part of the id
    output wire                     axi_read_req_unique;  // the line is wanted unique
    input  wire                     axi_read_data_valid;  // a read beat for this bank
    input  wire [AXI_SUBID_W-1:0]   axi_read_data_sub_id; // entry number from the id, 3 bits
    input  wire                     axi_read_data_non_cacheable;  // non-cacheable flag from the id
    input  wire [AXI_DATA_W-1:0]    axi_read_data;        // beat data, AXI_DATA_W = 128 bits
    input  wire                     axi_read_data_last;   // last beat of the burst
    input  wire                     axi_read_data_error;  // error response
    input  wire                     axi_read_data_shared; // another cache keeps a copy
    input  wire                     refill_handover_busy; // the L2 port is handing a line to this bank beat by beat: start no snoop
    // ---- memory write channel (to l1d_l2_port) ----
    output wire                     axi_write_req_valid;  // the bank offers a write request
    input  wire                     axi_write_req_ready;  // the L2 port took it
    output wire [AXI_ADDR_W-1:0]    axi_write_req_addr;   // write address, 64 bits
    output wire [7:0]               axi_write_req_burst_len;  // beats minus one
    output wire [2:0]               axi_write_req_size;   // beat size
    output wire [AXI_SUBID_W-1:0]   axi_write_req_sub_id; // victim-cache slot number (0 for non-cacheable), part of the id, 3 bits
    output wire                     axi_write_req_non_cacheable;  // 1 = non-cacheable access
    output wire [LINE_W-1:0]        axi_write_req_data;   // whole line (beat k at [k*AXI_DATA_W +: AXI_DATA_W]), 512 bits
    output wire [AXI_STRB_W-1:0]    axi_write_req_strobe; // strobes of a single-beat (non-cacheable) write, 16 bits
    input  wire                     axi_write_resp_valid; // a write response for this bank
    input  wire [AXI_SUBID_W-1:0]   axi_write_resp_sub_id;  // slot number from the id, 3 bits
    input  wire                     axi_write_resp_non_cacheable;  // non-cacheable flag from the id
    input  wire                     axi_write_resp_error; // error response
    // ---- flush / clean ----
    input  wire                     maint_start;          // start a flush or clean walk of this bank
    input  wire                     maint_op_is_flush;    // 0 = clean, 1 = flush
    output wire                     maint_done;           // one-cycle pulse: this bank finished the walk
    // ---- AMO lock time-out (pulse) ----
    output wire                     amo_lock_timeout_pulse;  // one-cycle pulse: an AMO lock timed out
    // ---- cache_v6: records for the prefetch unit (registered in l1d_bank_s2, valid one cycle) ----
    output wire                     train_valid;          // a training record this cycle
    output wire [PORT_W-1:0]        train_port;           // its request port, PORT_W = 2 bits
    output wire [PC_W-1:0]          train_pc;             // its instruction PC, 64 bits
    output wire [ADDR_W-1:0]        train_addr;           // its byte address, ADDR_W = 64 bits
    output wire                     train_is_store;       // store class
    output wire                     train_miss;           // the access missed
    output wire                     train_pf_hit;         // the access hit a prefetched line (first use)
    output wire                     train_no_train;       // atomic, non-cacheable or not the LSU: ignore
    output wire                     evict_valid;          // a line left the array this cycle
    output wire [LINE_ADDR_W-1:0]   evict_line_addr;      // its line address, 58 bits
    output wire                     pf_fb_valid;          // a prefetch op finished in S2
    output wire [LINE_ADDR_W-1:0]   pf_fb_line_addr;      // its line address, 58 bits
    output wire [1:0]               pf_fb_result;         // PF_FB_* (l1d_encodings.vh)
    // ---- cache_v6: ECC pulses (registered in l1d_bank_s2) ----
    output wire                     ecc_correctable;      // a corrected line or tag was written (scrub)
    output wire                     ecc_uncorrectable;    // an uncorrectable error was seen

    // ======================================================================
    // Wires between the stages (named as in the stage modules; see the header for the name codes)
    // ======================================================================
    // ---- from u_s0 (stage G) ----
    wire                    any_gnt;                      // the SRAMs read set g_set this cycle
    wire [INDEX_W-1:0]      g_set;                        // the set they read, INDEX_W = 6 bits
    wire                    gnt_refill, gnt_maint;        // the refill token / the flush-clean token issues now
    wire                    r_valid;                      // Stage R holds an operation
    wire [1:0]              r_kind;                       // its kind (K_LSU, K_REFILL, K_MAINT, K_SNOOP)
    wire [SEAT_W-1:0]       r_seat;                       // its tracker entry, SEAT_W = 3 bits
    wire [REQ_W-1:0]        r_req;                        // its request packet, REQ_W = 148 bits
    wire                    r_fast;                       // the CPU request issued fresh (the R check applies)
    wire [MSHR_ID_W-1:0]    r_mid;                        // MSHR entry of a refill token, MSHR_ID_W = 3 bits
    wire [LINE_ADDR_W-1:0]  r_line;                       // its line address, LINE_ADDR_W = 58 bits
    wire [WAY_W-1:0]        r_mway;                       // way of a flush/clean token, WAY_W = 2 bits
    wire                    r_mop;                        // operation of a flush/clean token
    wire                    r_sinv;                       // snoop: 1 = invalidate
    wire [SNP_ID_W-1:0]     r_sid;                        // snoop buffer entry, SNP_ID_W = 1 bit
    wire [TRK_ENTRIES-1:0]  r_deps;                       // dependency bits of the entry in R (tracker flops), 8 bits
    wire                    tracker_empty;                // no tracker entry in use (bank idle check)
    wire                    miss_ok;                      // an MSHR / NC answer may be pushed now
    // CROSSBANK STEP 1 (2026-11-25): this bank pushes an answer for port p this cycle = lsu_response_valid and the port field of the packet.
    // Only this bank pushes into its own FIFO of port p in the merger, so the credit in l1d_bank_s0 can use free_slots (flop) instead of free_slots_next.
    wire [LSU_PORTS-1:0]    own_response_push_port;       // bit p: an answer for port p is pushed now, LSU_PORTS = 4 bits
    // TIMING OPT (CB-1): the one-hot port of the push now comes from l1d_bank_s3 (own_response_push_port), decoded per source, not from the packet mux.
    wire                    s2_seat_trained;              // the entry in S2 already sent its training record
    wire [PC_W-1:0]         s2_seat_pc;                   // its PC, 64 bits
    // ---- from u_r (stage R) ----
    wire                    s1_valid;                     // stage S1 holds an operation
    wire [1:0]              s1_kind;                      // its kind
    wire [SEAT_W-1:0]       s1_seat;                      // its tracker entry, 3 bits
    wire [REQ_W-1:0]        s1_req;                       // its request packet, 148 bits
    wire [MSHR_ID_W-1:0]    s1_mid;                       // MSHR entry of the refill token, 3 bits
    wire [LINE_ADDR_W-1:0]  s1_line;                      // its line address, 58 bits
    wire [WAY_W-1:0]        s1_mway;                      // way of the flush/clean token, 2 bits
    wire                    s1_mop;                       // operation of the flush/clean token
    wire                    s1_sinv;                      // snoop: 1 = invalidate
    wire [SNP_ID_W-1:0]     s1_sid;                       // snoop buffer entry, 1 bit
    wire                    s1_kill;                      // a fresh request cancelled in R (ghost)
    wire [WAYS*TWORD_W-1:0] tag_words_q;                  // registered coded tags of all ways, WAYS*TWORD_W = 228 bits
    wire [WAYS*DWORD_W-1:0] s1_words;                     // registered coded lines of all ways, WAYS*DWORD_W = 2304 bits
    // ---- from u_s1 (stage S1) ----
    wire                    r_held;                       // the refill token in S1 is held (no victim-cache slot)
    wire                    rsv_clear_valid;              // a snoop or an eviction clears the reservation of rsv_clear_line
    wire [LINE_ADDR_W-1:0]  rsv_clear_line;               // that line, 58 bits
    wire                    clear_eq_s2_line;             // CB-3: rsv_clear_line == s2_line (u_s1 -> u_rsv)
    wire                    clear_eq_rsv_line;            // CB-3: rsv_clear_line == reservation line (u_s1 -> u_rsv)
    wire [S2_WAY_COPIES*WAYS-1:0]    s2_way_onehot_copy;  // CB-3: 12 registered one-hot copies of s2_way (u_s1 -> u_s2), 48 bits
    wire [GRANULES*MSHR_ENTRIES-1:0] s2_mid_onehot_copy;  // CB-3: 8 registered one-hot copies of s2_mid (u_s1 -> u_mshr), 64 bits
    wire                    s2_valid;                     // stage S2 holds an operation
    wire [1:0]              s2_kind;                      // its kind
    wire [SEAT_W-1:0]       s2_seat;                      // its tracker entry, 3 bits
    wire [REQ_W-1:0]        s2_req;                       // its request packet, 148 bits
    wire [MSHR_ID_W-1:0]    s2_mid;                       // MSHR entry of the refill token being installed, 3 bits
    wire [LINE_ADDR_W-1:0]  s2_line;                      // its line address, 58 bits
    wire                    s2_sinv;                      // snoop: 1 = invalidate
    wire [SNP_ID_W-1:0]     s2_sid;                       // snoop buffer entry, 1 bit
    wire                    s2_hit;                       // the CPU request was a served cache hit in S1
    wire                    s2_scfail;                    // an SC whose reservation check failed in S1
    wire                    s2_snp_hit;                   // snoop: the tags hit
    wire                    s2_hand;                      // snoop: the line was M, its data is handed over
    wire [WAY_W-1:0]        s2_way;                       // the way used in S1, 2 bits
    wire                    s2_held;                      // the refill / maintenance token was held in S1
    wire                    s2_swap;                      // the refill is a swap back from the victim cache
    wire                    s2_kill;                      // the fresh request was cancelled in R
    wire [WAYS*WRITE_DATA_WIDTH-1:0] s2_words;            // the load word of every way, WAYS*64 = 256 bits
    wire [WAYS*LINE_W-1:0]  s2_lines;                     // the lines of all ways, 2048 bits
    wire [WAYS*TAG_W-1:0]   s2_tags;                      // the raw tags of all ways, 200 bits
    wire [LINE_BYTES-1:0]   s2_st_be;                     // byte enables of a store hit, LINE_BYTES = 64 bits
    wire [LINE_W-1:0]       s2_st_din;                    // store data at its byte position, LINE_W = 512 bits
    wire                    s2_wbpend;                    // the victim is a flush/clean line
    wire                    s2_vvalid;                    // a line is being evicted
    wire                    s2_vdirty;                    // ... dirty
    wire                    s2_vshared;                   // ... shared
    wire [LINE_ADDR_W-1:0]  s2_vline;                     // ... its line address, 58 bits
    wire [VC_ID_W-1:0]      s2_slot;                      // victim-cache slot that takes it, VC_ID_W = 3 bits
    wire                    s2_raw_hit;                   // the tags hit (prefetch decision)
    wire                    s2_pf_hit;                    // a demand hit used a prefetched line
    wire                    s2_tag_blocked;               // the CPU request was blocked by a tag ECC error
    wire [WAYS*GRANULES*DATA_HAM_W-1:0] s2_data_synd;     // data syndromes of all ways and granules, 224 bits
    wire [WAYS*GRANULES-1:0] s2_data_fix;                 // correctable per way and granule, 32 bits
    wire [WAYS-1:0]         s2_data_ce;                   // per way: some granule correctable, 4 bits
    wire [WAYS-1:0]         s2_data_ue;                   // per way: some granule uncorrectable, 4 bits
    wire [WAYS*TAG_HAM_W-1:0] s2_tag_synd;                // tag syndromes of all ways, 24 bits
    wire [WAYS-1:0]         s2_tag_ce;                    // per way: tag correctable and valid, 4 bits
    wire [WAYS-1:0]         s2_tag_ue;                    // per way: tag uncorrectable and valid, 4 bits
    // ---- from u_s2 (stage S2) ----
    wire                    s2_refill;                    // the refill installs (line + tag write) this cycle
    wire                    s2_maint;                     // a flush/clean token is in S2
    wire                    s2_st_wr;                     // a line is written now (merged store line or corrected line)
    wire                    s2_tag_fix_wr;                // a corrected tag is written now
    wire [WAY_W-1:0]        s2_write_way;                 // the way of the S2 write, 2 bits
    wire [WAYS-1:0]         s2_write_way_onehot;          // CB-3: the same way, one-hot (u_s2 -> u_r), 4 bits
    wire [LINE_W-1:0]       s2_write_line;                // the line S2 writes, 512 bits
    wire [TAG_W-1:0]        s2_write_tag;                 // the tag S2 writes, TAG_W = 50 bits
    wire                    s2_uniq;                      // the request needs the line in E / M
    wire                    s2_amo1;                      // the request is a one-request AMO
    wire                    s2_alloc_is_write;            // an MSHR allocation is a write
    wire                    s2_alloc_no_response;         // the MSHR fetches without answering
    wire                    s2_alloc_is_prefetch;         // the MSHR allocation is a prefetch
    wire [LINE_W-1:0]       line_q;                       // the line of the way used in S1, 512 bits
    wire [LINE_W-1:0]       st_merged;                    // the store bytes merged into that line, 512 bits
    wire                    act_valid;                    // the CPU request in S2 waits, re-issues or was handed to a miss unit
    wire [2:0]              act_code;                     // ACT_* (l1d_encodings.vh)
    wire [WID_W-1:0]        act_id;                       // MSHR entry or victim slot it waits for, WID_W = 3 bits
    wire                    s3_load;                      // the CPU request in S2 answers now
    wire [RESP_W-1:0]       s2_pkt;                       // its response packet, RESP_W = 74 bits
    wire                    m_alloc, m_swap;              // allocate an MSHR entry; as a swap from the victim cache
    wire                    vc_swap;                      // start the swap on victim-cache slot vc_slot
    wire                    n_alloc;                      // queue the request in the non-cacheable unit
    wire                    ev_lr, ev_sc, ev_wr;          // reservation events: LR / SC / cacheable write completed
    wire                    ev_amor, ev_amow;             // AMO read / write half completed (always 0 in this version)
    wire                    snp_act_valid;                // the snoop in S2 has a result
    wire [1:0]              snp_act_code;                 // SACT_DONE / SACT_WMSHR / SACT_WVC
    wire [WID_W-1:0]        snp_act_id;                   // MSHR entry or victim slot the snoop waits for, 3 bits
    wire                    vc_snp;                       // the snoop acts on a VALID victim-cache slot
    wire                    vc_ins;                       // insert a victim into the victim cache
    wire                    vc_rel;                       // release a MOVING victim-cache slot
    wire                    s3_amo_v;                     // S3 holds an AMO hit: it writes its line this cycle
    wire [LINE_ADDR_W-1:0]  s3_line;                      // its line address, 58 bits
    wire [WAY_W-1:0]        s3_way;                       // its way, 2 bits
    wire [LINE_W-1:0]       s3_old;                       // the old line, 512 bits
    wire [LINE_BYTES-1:0]   s3_be;                        // bytes of the AMO word, 64 bits
    wire [OFFSET_W-1:0]     s3_off;                       // byte offset of the AMO word, OFFSET_W = 6 bits
    wire [WRITE_DATA_WIDTH-1:0] s3_new;                   // the AMO unit's result, 64 bits
    wire                    s2_train_sent;                // S2 sends the training record of the entry in S2 now
    // ---- from u_s3 ----
    wire [LINE_W-1:0]       s3_merged;                    // the old line with the AMO result in place, 512 bits
    wire                    done_valid;                   // a response leaves the bank (unused since the tracker frees at S2)
    wire [SEAT_W-1:0]       done_seat;                    // that entry, 3 bits (unused)
    wire                    m_resp_ack, n_resp_ack;       // the MSHR / NC answer is taken this cycle
    wire                    m_rd_ready, n_rd_ready;       // the MSHR / NC read request is taken
    wire                    vc_wr_ready, n_wr_ready;      // the victim-cache / NC write request is taken
    // ---- from the reservation unit ----
    wire                    rsv_sc_match;                 // the reservation covers the line in S1 (SC check)
    wire                    lock_valid, lock_snp;         // a line is AMO-locked; a snoop to it must wait (never in this version)
    wire [LINE_ADDR_W-1:0]  lock_line;                    // the locked line, 58 bits
    wire [LINE_ADDR_W-1:0]  reservation_line_addr;        // CB-3: the reservation line (u_rsv flop -> u_s1), 58 bits
    // ---- from the MSHR ----
    wire                    mshr_hit, mshr_can, mshr_hasd;  // lookup of s2_line: hit; an entry can be allocated; the entry holds data
    wire [MSHR_ID_W-1:0]    mshr_hid, mshr_aid;           // the hit entry; the entry an allocation takes, 3 bits
    wire                    pf_can_alloc;                 // a prefetch may take an MSHR entry (flop)
    wire                    m_rd_valid, m_rd_unique;      // the MSHR offers a line read; wanted unique
    wire [LINE_ADDR_W-1:0]  m_rd_line;                    // its line address, 58 bits
    wire [MSHR_ID_W-1:0]    m_rd_sub;                     // its MSHR entry, 3 bits
    wire                    m_resp_valid;                 // an MSHR answer waits
    wire [RESP_W-1:0]       m_resp_pkt;                   // its packet, 74 bits
    wire [SEAT_W-1:0]       m_resp_seat;                  // its entry, 3 bits
    wire                    tok_valid;                    // the MSHR offers a refill token
    wire [MSHR_ID_W-1:0]    tok_id;                       // its entry, 3 bits
    wire [LINE_ADDR_W-1:0]  tok_line;                     // its line address, 58 bits
    wire [1:0]              q1_state;                     // the entry being installed (refill token in S1): state to install
    wire                    q1_swap;                      // ... it is a swap
    wire [VC_ID_W-1:0]      q1_slot;                      // ... its victim-cache slot, 3 bits
    wire                    q1_is_prefetch;               // ... it is a prefetch (the installed line gets the pf bit)
    wire [LINE_W-1:0]       q2_data;                      // the line of the entry being installed (S2), 512 bits
    wire                    mshr_busy;                    // some MSHR entry is busy
    wire [MSHR_ENTRIES-1:0] mshr_freed;                   // one pulse per MSHR entry that became free, MSHR_ENTRIES = 8 bits
    // ---- from the non-cacheable queue ----
    wire                    nc_can;                       // the NC queue can take a request
    wire                    n_rd_valid, n_wr_valid;       // it offers a read / a write
    wire [ADDR_W-1:0]       n_rd_addr, n_wr_addr;         // their byte addresses, ADDR_W = 64 bits
    wire [2:0]              n_rd_size, n_wr_size;         // their sizes
    wire [AXI_DATA_W-1:0]   n_wr_data;                    // the write beat, AXI_DATA_W = 128 bits
    wire [AXI_STRB_W-1:0]   n_wr_strb;                    // its strobes, AXI_STRB_W = 16 bits
    wire                    n_resp_valid;                 // an NC answer waits
    wire [RESP_W-1:0]       n_resp_pkt;                   // its packet, 74 bits
    wire [SEAT_W-1:0]       n_resp_seat;                  // its entry, 3 bits
    wire                    nc_empty;                     // the NC queue is empty
    wire                    nc_freed;                     // an NC place became free (wake-up)
    // ---- from the victim cache ----
    wire                    vc_hit, vc_dirty, vc_shared;  // lookup of s2_line: hit; the line is dirty; shared
    wire [VC_ID_W-1:0]      vc_slot;                      // the hit slot, 3 bits
    wire [2:0]              vc_state;                     // its state (VC_VALID / VC_WBPEND / VC_MOVING)
    wire [LINE_W-1:0]       vc_data;                      // its line, 512 bits
    wire                    vc_al_ok;                     // a slot is free for a victim
    wire [VC_ID_W-1:0]      vc_al_slot;                   // its number, 3 bits
    wire                    vc_wr_valid;                  // the victim cache offers a writeback
    wire [LINE_ADDR_W-1:0]  vc_wr_line;                   // its line address, 58 bits
    wire [VC_ID_W-1:0]      vc_wr_sub;                    // its slot, 3 bits
    wire [LINE_W-1:0]       vc_wr_data;                   // its line, 512 bits
    wire                    vc_empty;                     // all slots are free
    wire [VC_SLOTS-1:0]     vc_freed;                     // one pulse per slot that became free, VC_SLOTS = 8 bits
    // ---- from the flush / clean walker ----
    wire                    mt_valid;                     // a flush/clean token is offered
    wire [INDEX_W-1:0]      mt_set;                       // its set, 6 bits
    wire [WAY_W-1:0]        mt_way;                       // its way, 2 bits
    wire                    mt_op;                        // 0 = clean, 1 = flush
    wire                    vc_drain;                     // the victim cache must write back its dirty lines now
    // ---- cache_v5 C-20: vector store data places and the vector line answer ----
    wire                    vec_slot_free;                // a vector data place is free (internal copy of the output)
    wire [VEC_SLOT_W-1:0]   vec_write_slot;               // the lowest free place, VEC_SLOT_W = 1 bit
    wire                    vec_write_valid;              // a vector store is allocated now: its line goes into vec_write_slot
    wire [VEC_SLOT_W-1:0]   vec_read_slot;                // the place of the vector store in S1, 1 bit
    wire [LINE_W-1:0]       vec_read_line_data;           // its line, 512 bits
    wire                    vec_release_valid;            // the vector store in S2 wrote its line: free its place
    wire [VEC_SLOT_W-1:0]   vec_release_slot;             // that place, 1 bit
    wire [LINE_W-1:0]       s2_line_answer;               // the line a vector load answers with, 512 bits

    // fields of the CPU request in S2, taken out of its packet for the miss units (wires only)
    wire [ADDR_W-1:0]           s2_addr;                  // byte address, 64 bits
    wire [OFFSET_W-1:0]         s2_off;                   // byte offset in the line, 6 bits
    wire [MEM_SIZE_BITS-1:0]    s2_size;                  // access size, 2 bits
    wire [MASTER_ID_BITS-1:0]   s2_mstid;                 // master id, 2 bits
    wire [IDX_BITS-1:0]         s2_idx;                   // ROB index, 3 bits
    wire [PORT_W-1:0]           s2_port;                  // request port, PORT_W = 2 bits
    wire [WRITE_DATA_WIDTH-1:0] s2_wdata;                 // store data / AMO operand, 64 bits
    assign s2_addr  = s2_req[REQ_ADDR_LSB +: ADDR_W];
    assign s2_off   = s2_addr[OFFSET_W-1:0];
    assign s2_size  = s2_req[REQ_SIZE_LSB +: MEM_SIZE_BITS];
    assign s2_mstid = s2_req[REQ_MID_LSB +: MASTER_ID_BITS];
    assign s2_idx   = s2_req[REQ_IDX_LSB +: IDX_BITS];
    assign s2_port  = s2_req[REQ_PORT_LSB +: PORT_W];
    assign s2_wdata = s2_req[REQ_DATA_LSB +: WRITE_DATA_WIDTH];
    // the shared memory beats and responses, split for the cacheable and the non-cacheable units
    wire axi_read_beat_cacheable;                         // a read beat for the MSHR
    wire axi_read_beat_nc;                                // a read beat for the NC queue
    wire axi_write_resp_nc;                               // a write response for the NC queue
    wire axi_write_resp_cacheable;                        // a write response for the victim cache
    assign axi_read_beat_cacheable  = axi_read_data_valid  && !axi_read_data_non_cacheable;
    assign axi_read_beat_nc         = axi_read_data_valid  &&  axi_read_data_non_cacheable;
    assign axi_write_resp_nc        = axi_write_resp_valid &&  axi_write_resp_non_cacheable;
    assign axi_write_resp_cacheable = axi_write_resp_valid && !axi_write_resp_non_cacheable;

    // ======================================================================
    // cache_v5 C-20: vector store data places (written at the bank door, read in S1, freed in S2) and the line answer
    // ======================================================================
    l1d_vec_store_buf #(`L1D_PARAMS_PASS, .BANK_ID(BANK_ID)) u_vec_buf (
        .clk             (clk),
        .rst             (rst),
        .write_valid     (vec_write_valid),
        .write_line_data (lsu_request_line_data),
        .slot_available  (vec_slot_free),
        .write_slot      (vec_write_slot),
        .read_slot       (vec_read_slot),
        .read_line_data  (vec_read_line_data),
        .release_valid   (vec_release_valid),
        .release_slot    (vec_release_slot)
    );
    assign vec_slot_available = vec_slot_free;
    // the line a vector load answers with leaves beside the response packet (only the VPU port's merger keeps it)
    assign lsu_response_line_data = s2_line_answer;

    // ======================================================================
    // The five stage modules
    // ======================================================================
    l1d_bank_s0 #(`L1D_PARAMS_PASS, .BANK_ID(BANK_ID)) u_s0 (
        .clk                           (clk),
        .rst                           (rst),
        .arrival_request_0             (arrival_request_0),
        .arrival_request_1             (arrival_request_1),
        .arrival_request_2             (arrival_request_2),
        .arrival_request_3             (arrival_request_3),
        .arrival_packet_0              (arrival_packet_0),
        .arrival_packet_1              (arrival_packet_1),
        .arrival_packet_2              (arrival_packet_2),
        .arrival_packet_3              (arrival_packet_3),
        .arrival_pc_0                  (arrival_pc_0),
        .arrival_pc_1                  (arrival_pc_1),
        .arrival_pc_2                  (arrival_pc_2),
        .arrival_pc_3                  (arrival_pc_3),
        .drain_request                 (drain_request),
        .drain_packet                  (drain_packet),
        .drain_pc                      (drain_pc),
        .arrival_allocated_0           (arrival_allocated_0),
        .arrival_allocated_1           (arrival_allocated_1),
        .arrival_allocated_2           (arrival_allocated_2),
        .arrival_allocated_3           (arrival_allocated_3),
        .drain_allocated               (drain_allocated),
        .tracker_free_count_q          (tracker_free_count_q),
        .lsu_response_fifo_free_0      (lsu_response_fifo_free_0),
        .lsu_response_fifo_free_1      (lsu_response_fifo_free_1),
        .lsu_response_fifo_free_2      (lsu_response_fifo_free_2),
        .lsu_response_fifo_free_3      (lsu_response_fifo_free_3),
        .lsu_response_fifo_free_next_0 (lsu_response_fifo_free_next_0),
        .lsu_response_fifo_free_next_1 (lsu_response_fifo_free_next_1),
        .lsu_response_fifo_free_next_2 (lsu_response_fifo_free_next_2),
        .lsu_response_fifo_free_next_3 (lsu_response_fifo_free_next_3),
        .snoop_request_valid           (snoop_request_valid),
        .snoop_request_line_addr       (snoop_request_line_addr),
        .snoop_request_invalidate      (snoop_request_invalidate),
        .snoop_request_ready           (snoop_request_ready),
        .snoop_response_fifo_free      (snoop_response_fifo_free),
        .prefetch_request_valid        (prefetch_request_valid),
        .prefetch_request_line_addr    (prefetch_request_line_addr),
        .prefetch_request_unique       (prefetch_request_unique),
        .prefetch_request_ready        (prefetch_request_ready),
        .refill_handover_busy          (refill_handover_busy),
        .s1_valid (s1_valid), .s1_kind (s1_kind), .s1_req (s1_req),
        .s2_valid (s2_valid), .s2_kind (s2_kind), .s2_req (s2_req), .s2_hit (s2_hit),
        .s2_sid (s2_sid), .s2_line (s2_line), .s2_seat (s2_seat), .s2_kill (s2_kill), .s2_answered (s3_load),
        .act_valid (act_valid), .act_code (act_code), .act_id (act_id),
        .snp_act_valid (snp_act_valid), .snp_act_code (snp_act_code), .snp_act_id (snp_act_id),
        .s2_train_sent (s2_train_sent), .s2_seat_trained (s2_seat_trained), .s2_seat_pc (s2_seat_pc),
        .s3_amo_v (s3_amo_v),
        .lock_valid (lock_valid), .lock_snp (lock_snp), .lock_line (lock_line), .ev_amor (ev_amor),
        .tok_valid (tok_valid), .tok_id (tok_id), .tok_line (tok_line),
        .mt_valid (mt_valid), .mt_set (mt_set), .mt_way (mt_way), .mt_op (mt_op),
        .m_resp_valid (m_resp_valid), .n_resp_valid (n_resp_valid),
        .mshr_freed (mshr_freed), .vc_freed (vc_freed), .nc_freed (nc_freed),
        .any_gnt (any_gnt), .g_set (g_set), .gnt_refill (gnt_refill), .gnt_maint (gnt_maint),
        .r_valid (r_valid), .r_kind (r_kind), .r_seat (r_seat), .r_req (r_req), .r_fast (r_fast), .r_mid (r_mid),
        .r_line (r_line), .r_mway (r_mway), .r_mop (r_mop), .r_sinv (r_sinv), .r_sid (r_sid), .r_deps (r_deps),
        .tracker_empty (tracker_empty), .miss_ok (miss_ok), .own_response_push_port (own_response_push_port),
        .vec_slot_available (vec_slot_free), .vec_write_slot (vec_write_slot), .vec_write_valid (vec_write_valid)
    );

    l1d_bank_r #(`L1D_PARAMS_PASS, .BANK_ID(BANK_ID)) u_r (
        .clk (clk), .rst (rst),
        .any_gnt (any_gnt), .g_set (g_set),
        .r_valid (r_valid), .r_kind (r_kind), .r_seat (r_seat), .r_req (r_req), .r_fast (r_fast), .r_mid (r_mid),
        .r_line (r_line), .r_mway (r_mway), .r_mop (r_mop), .r_sinv (r_sinv), .r_sid (r_sid),
        .r_deps (r_deps),
        .s2_refill (s2_refill), .s2_st_wr (s2_st_wr), .s2_tag_fix_wr (s2_tag_fix_wr), .s2_way (s2_write_way),
        .s2_way_onehot (s2_write_way_onehot),                                                 // CB-3
        .s2_line (s2_line), .s2_write_line (s2_write_line), .s2_write_tag (s2_write_tag),
        .s3_amo_v (s3_amo_v), .s3_way (s3_way), .s3_line (s3_line), .s3_merged (s3_merged),
        .s1_valid (s1_valid), .s1_kind (s1_kind), .s1_seat (s1_seat), .s1_req (s1_req), .s1_mid (s1_mid),
        .s1_line (s1_line), .s1_mway (s1_mway), .s1_mop (s1_mop), .s1_sinv (s1_sinv), .s1_sid (s1_sid),
        .s1_kill (s1_kill), .tag_words_q (tag_words_q), .s1_words (s1_words)
    );

    l1d_bank_s1 #(`L1D_PARAMS_PASS, .BANK_ID(BANK_ID)) u_s1 (
        .clk (clk), .rst (rst),
        .s1_valid (s1_valid), .s1_kind (s1_kind), .s1_seat (s1_seat), .s1_req (s1_req), .s1_mid (s1_mid),
        .s1_line (s1_line), .s1_mway (s1_mway), .s1_mop (s1_mop), .s1_sinv (s1_sinv), .s1_sid (s1_sid),
        .s1_kill (s1_kill), .tag_words_q (tag_words_q), .s1_words (s1_words),
        .lock_valid (lock_valid), .lock_line (lock_line),
        .q1_state (q1_state), .q1_swap (q1_swap), .q1_slot (q1_slot), .q1_is_prefetch (q1_is_prefetch),
        .vc_al_ok (vc_al_ok), .vc_al_slot (vc_al_slot),
        .rsv_sc_match (rsv_sc_match), .reservation_line_addr (reservation_line_addr),          // CB-3
        .r_held (r_held), .rsv_clear_valid (rsv_clear_valid), .rsv_clear_line (rsv_clear_line),
        .clear_eq_s2_line (clear_eq_s2_line), .clear_eq_rsv_line (clear_eq_rsv_line),          // CB-3
        .vec_read_slot (vec_read_slot), .vec_read_line_data (vec_read_line_data),
        .s2_valid (s2_valid), .s2_kind (s2_kind), .s2_seat (s2_seat), .s2_req (s2_req), .s2_mid (s2_mid),
        .s2_line (s2_line), .s2_sinv (s2_sinv), .s2_sid (s2_sid),
        .s2_hit (s2_hit), .s2_scfail (s2_scfail), .s2_snp_hit (s2_snp_hit), .s2_hand (s2_hand), .s2_way (s2_way),
        .s2_held (s2_held), .s2_swap (s2_swap), .s2_kill (s2_kill),
        .s2_words (s2_words), .s2_lines (s2_lines), .s2_st_be (s2_st_be), .s2_st_din (s2_st_din),
        .s2_wbpend (s2_wbpend), .s2_vvalid (s2_vvalid), .s2_vdirty (s2_vdirty), .s2_vshared (s2_vshared),
        .s2_vline (s2_vline), .s2_slot (s2_slot),
        .s2_raw_hit (s2_raw_hit), .s2_pf_hit (s2_pf_hit),
        .s2_data_synd (s2_data_synd), .s2_data_fix (s2_data_fix), .s2_data_ce (s2_data_ce), .s2_data_ue (s2_data_ue),
        .s2_tag_synd (s2_tag_synd), .s2_tag_fix (), .s2_tag_ce (s2_tag_ce), .s2_tag_ue (s2_tag_ue),
        .s2_tag_blocked (s2_tag_blocked), .s2_tags (s2_tags),
        .s2_way_onehot_copy (s2_way_onehot_copy), .s2_mid_onehot_copy (s2_mid_onehot_copy)   // CB-3
    );

    l1d_bank_s2 #(`L1D_PARAMS_PASS, .BANK_ID(BANK_ID)) u_s2 (
        .clk (clk), .rst (rst),
        .s2_valid (s2_valid), .s2_kind (s2_kind), .s2_seat (s2_seat), .s2_req (s2_req), .s2_mid (s2_mid),
        .s2_line (s2_line), .s2_sinv (s2_sinv),
        .s2_hit (s2_hit), .s2_scfail (s2_scfail), .s2_snp_hit (s2_snp_hit), .s2_hand (s2_hand), .s2_way (s2_way),
        .s2_held (s2_held), .s2_swap (s2_swap), .s2_kill (s2_kill),
        .s2_way_onehot_copy (s2_way_onehot_copy),                                             // CB-3
        .s2_words (s2_words), .s2_lines (s2_lines), .s2_tags (s2_tags), .s2_st_be (s2_st_be), .s2_st_din (s2_st_din),
        .s2_wbpend (s2_wbpend), .s2_vvalid (s2_vvalid), .s2_vdirty (s2_vdirty), .s2_vshared (s2_vshared),
        .s2_vline (s2_vline), .s2_slot (s2_slot),
        .s2_raw_hit (s2_raw_hit), .s2_pf_hit (s2_pf_hit), .s2_tag_blocked (s2_tag_blocked),
        .s2_data_synd (s2_data_synd), .s2_data_fix (s2_data_fix), .s2_data_ce (s2_data_ce), .s2_data_ue (s2_data_ue),
        .s2_tag_synd (s2_tag_synd), .s2_tag_ce (s2_tag_ce), .s2_tag_ue (s2_tag_ue),
        .s2_seat_trained (s2_seat_trained), .s2_seat_pc (s2_seat_pc),
        .q2_data (q2_data),
        .mshr_hit (mshr_hit), .mshr_hid (mshr_hid), .mshr_hasd (mshr_hasd), .mshr_can (mshr_can), .mshr_aid (mshr_aid),
        .prefetch_can_allocate (pf_can_alloc),
        .vc_hit (vc_hit), .vc_slot (vc_slot), .vc_state (vc_state), .vc_data (vc_data), .vc_dirty (vc_dirty),
        .nc_can (nc_can),
        .s2_refill (s2_refill), .s2_maint (s2_maint), .s2_st_wr (s2_st_wr), .s2_tag_fix_wr (s2_tag_fix_wr),
        .s2_write_way (s2_write_way), .s2_write_way_onehot (s2_write_way_onehot),            // CB-3
        .s2_write_line (s2_write_line), .s2_write_tag (s2_write_tag),
        .s2_uniq (s2_uniq), .s2_amo1 (s2_amo1), .s2_alloc_is_write (s2_alloc_is_write),
        .s2_alloc_no_response (s2_alloc_no_response), .s2_alloc_is_prefetch (s2_alloc_is_prefetch),
        .s2_line_answer (s2_line_answer), .vec_release_valid (vec_release_valid), .vec_release_slot (vec_release_slot),
        .line_q (line_q), .st_merged (st_merged),
        .act_valid (act_valid), .act_code (act_code), .act_id (act_id),
        .s3_load (s3_load), .s2_pkt (s2_pkt),
        .m_alloc (m_alloc), .m_swap (m_swap), .vc_swap (vc_swap), .n_alloc (n_alloc),
        .ev_lr (ev_lr), .ev_sc (ev_sc), .ev_wr (ev_wr), .ev_amor (ev_amor), .ev_amow (ev_amow),
        .rsv_event_pulse (rsv_event_pulse),
        .snoop_response_valid (snoop_response_valid), .snoop_response_packet (snoop_response_packet),
        .snp_act_valid (snp_act_valid), .snp_act_code (snp_act_code), .snp_act_id (snp_act_id),
        .vc_snp (vc_snp), .vc_ins (vc_ins), .vc_rel (vc_rel),
        .s3_amo_v (s3_amo_v), .s3_line (s3_line), .s3_way (s3_way), .s3_old (s3_old),
        .s3_be (s3_be), .s3_off (s3_off), .s3_new (s3_new),
        .s2_train_sent (s2_train_sent), .train_valid (train_valid), .train_port (train_port), .train_pc (train_pc),
        .train_addr (train_addr), .train_is_store (train_is_store), .train_miss (train_miss),
        .train_pf_hit (train_pf_hit), .train_no_train (train_no_train),
        .evict_valid (evict_valid), .evict_line_addr (evict_line_addr),
        .pf_fb_valid (pf_fb_valid), .pf_fb_line_addr (pf_fb_line_addr), .pf_fb_result (pf_fb_result),
        .ecc_correctable (ecc_correctable), .ecc_uncorrectable (ecc_uncorrectable)
    );

    l1d_bank_s3 #(`L1D_PARAMS_PASS, .BANK_ID(BANK_ID)) u_s3 (
        .clk (clk), .rst (rst),
        .s3_old (s3_old), .s3_be (s3_be), .s3_off (s3_off), .s3_new (s3_new), .s3_merged (s3_merged),
        .s3_load (s3_load), .s2_pkt (s2_pkt), .s2_seat (s2_seat),
        .m_resp_valid (m_resp_valid), .m_resp_pkt (m_resp_pkt), .m_resp_seat (m_resp_seat), .m_resp_ack (m_resp_ack),
        .n_resp_valid (n_resp_valid), .n_resp_pkt (n_resp_pkt), .n_resp_seat (n_resp_seat), .n_resp_ack (n_resp_ack),
        .miss_ok (miss_ok), .lsu_response_fifo_free_0 (lsu_response_fifo_free_0), .lsu_response_fifo_free_1 (lsu_response_fifo_free_1),
        .lsu_response_fifo_free_2 (lsu_response_fifo_free_2), .lsu_response_fifo_free_3 (lsu_response_fifo_free_3),
        .lsu_response_valid (lsu_response_valid), .lsu_response_packet (lsu_response_packet), .own_response_push_port (own_response_push_port),
        .done_valid (done_valid), .done_seat (done_seat),
        .m_rd_valid (m_rd_valid), .m_rd_ready (m_rd_ready), .m_rd_line (m_rd_line), .m_rd_sub (m_rd_sub), .m_rd_unique (m_rd_unique),
        .n_rd_valid (n_rd_valid), .n_rd_ready (n_rd_ready), .n_rd_addr (n_rd_addr), .n_rd_size (n_rd_size),
        .vc_wr_valid (vc_wr_valid), .vc_wr_ready (vc_wr_ready), .vc_wr_line (vc_wr_line), .vc_wr_sub (vc_wr_sub), .vc_wr_data (vc_wr_data),
        .n_wr_valid (n_wr_valid), .n_wr_ready (n_wr_ready), .n_wr_addr (n_wr_addr), .n_wr_size (n_wr_size),
        .n_wr_data (n_wr_data), .n_wr_strb (n_wr_strb),
        .axi_read_req_valid (axi_read_req_valid), .axi_read_req_ready (axi_read_req_ready),
        .axi_read_req_addr (axi_read_req_addr), .axi_read_req_burst_len (axi_read_req_burst_len),
        .axi_read_req_size (axi_read_req_size), .axi_read_req_sub_id (axi_read_req_sub_id),
        .axi_read_req_non_cacheable (axi_read_req_non_cacheable), .axi_read_req_unique (axi_read_req_unique),
        .axi_write_req_valid (axi_write_req_valid), .axi_write_req_ready (axi_write_req_ready),
        .axi_write_req_addr (axi_write_req_addr), .axi_write_req_burst_len (axi_write_req_burst_len),
        .axi_write_req_size (axi_write_req_size), .axi_write_req_sub_id (axi_write_req_sub_id),
        .axi_write_req_non_cacheable (axi_write_req_non_cacheable), .axi_write_req_data (axi_write_req_data),
        .axi_write_req_strobe (axi_write_req_strobe)
    );

    // ======================================================================
    // Reservation (LR / SC) and AMO lock
    // ======================================================================
    // ---- RESERVATION (LR / SC) and AMO LOCK: l1d_reservation_unit. Evicting the reserved line clears the reservation (when enabled). ----
    l1d_reservation_unit #(`L1D_PARAMS_PASS) u_rsv (
        .clk                    (clk),
        .rst                    (rst),
        .sc_check_line_addr     (s1_line),
        .sc_reservation_matches (rsv_sc_match),
        .s2_line_addr           (s2_line),
        .s2_lr_completed        (ev_lr),
        .s2_sc_completed        (ev_sc),
        .s2_write_completed     (ev_wr),
        .s2_amo_read_completed  (ev_amor),
        .s2_amo_read_was_hit    (s2_hit),
        .s2_amo_write_completed (ev_amow),
        .s2_install_valid       (s2_refill),
        .s2_install_line_addr   (s2_line),
        .s1_clear_valid         (rsv_clear_valid),        // a snoop in S1, or the eviction of a line (computed in u_s1)
        .s1_clear_line_addr     (rsv_clear_line),         // CB-3: debug print and check only
        .clear_eq_s2_line       (clear_eq_s2_line),       // CB-3: the two compares of the clear line, made in u_s1
        .clear_eq_rsv_line      (clear_eq_rsv_line),
        .maint_clear            (maint_start),
        .other_bank_rsv_clear   (other_bank_rsv_event),   // A2: an LR/SC in another bank ends this reservation
        .reservation_valid      (),
        .reservation_line_addr  (reservation_line_addr),  // CB-3: to u_s1 (per-way clear compares)
        .amo_lock_valid         (lock_valid),
        .amo_lock_snoop_hold    (lock_snp),
        .amo_lock_line          (lock_line),
        .amo_lock_timeout_pulse (amo_lock_timeout_pulse)
    );

    // ======================================================================
    // Miss units: MSHR, NC entries, victim cache, flush / clean
    // ======================================================================
    // ---- MISS UNITS: cacheable MSHR, non-cacheable queue, victim cache, flush/clean walker ----
    l1d_mshr #(`L1D_PARAMS_PASS) u_mshr (
        .clk                      (clk),
        .rst                      (rst),
        .lookup_line_addr_next    (s1_line),              // TIMING OPT (CB-4): s1_line becomes s2_line at the next edge (no stall)
        .lookup_line_addr         (s2_line),              // CB-4: read only by the simulation check in l1d_mshr
        .lookup_hit               (mshr_hit),
        .lookup_entry_id          (mshr_hid),
        .lookup_entry_has_data    (mshr_hasd),
        .allocate_valid           (m_alloc),
        .allocate_line_addr       (s2_line),
        .allocate_is_swap         (m_swap),
        .allocate_swap_line_data  (vc_data),
        .allocate_swap_dirty      (vc_dirty),
        .allocate_swap_shared     (vc_shared),
        .allocate_swap_vc_slot    (vc_slot),
        .allocate_unique_read     (s2_uniq),
        .allocate_seat            (s2_seat),
        .allocate_is_write        (s2_alloc_is_write),                  // an AMO miss is fetched like a read (no operand merge)
        .allocate_no_response     (s2_alloc_no_response),               // an AMO / vector / prefetch miss is not answered by the MSHR
        .allocate_is_prefetch     (s2_alloc_is_prefetch),               // cache_v6: a prefetch entry (at most PF_MSHR_MAX busy)
        .allocate_access_size     (s2_size),
        .allocate_master_id       (s2_mstid),
        .allocate_rob_index       (s2_idx),
        .allocate_port            (s2_port),                            // A2: the refill answer goes back to this port
        .allocate_byte_offset     (s2_off),
        .allocate_store_data      (s2_wdata),
        .can_allocate             (mshr_can),
        .allocate_entry_id        (mshr_aid),
        .prefetch_can_allocate    (pf_can_alloc),
        .axi_read_req_valid       (m_rd_valid),
        .axi_read_req_ready       (m_rd_ready),
        .axi_read_req_line_addr   (m_rd_line),
        .axi_read_req_sub_id      (m_rd_sub),
        .axi_read_req_unique      (m_rd_unique),
        .axi_read_beat_valid      (axi_read_beat_cacheable),
        .axi_read_beat_sub_id     (axi_read_data_sub_id[MSHR_ID_W-1:0]),
        .axi_read_beat_data       (axi_read_data),
        .axi_read_beat_last       (axi_read_data_last),
        .axi_read_beat_error      (axi_read_data_error),
        .axi_read_beat_shared     (axi_read_data_shared),
        .response_valid           (m_resp_valid),
        .response_packet          (m_resp_pkt),
        .response_seat            (m_resp_seat),
        .response_taken           (m_resp_ack),
        .refill_token_valid       (tok_valid),
        .refill_token_entry_id    (tok_id),
        .refill_token_line_addr   (tok_line),
        .refill_token_granted     (gnt_refill),
        .s1_install_entry_id      (s1_mid),
        .s1_install_state         (q1_state),
        .s1_install_is_swap       (q1_swap),
        .s1_install_vc_slot       (q1_slot),
        .s1_install_is_prefetch   (q1_is_prefetch),
        .s2_install_entry_id      (s2_mid),
        .s2_install_entry_onehot  (s2_mid_onehot_copy),           // CB-3: one-hot copies of s2_mid (u_s1 flops)
        .s2_install_line_data     (q2_data),
        .s1_token_held_valid      (r_held),
        .s1_token_held_entry_id   (s1_mid),
        .s2_install_done_valid    (s2_refill),
        .s2_install_done_entry_id (s2_mid),
        .any_entry_busy           (mshr_busy),
        .entry_freed_pulses       (mshr_freed)
    );

    // non-cacheable queue
    l1d_mshr_nc #(`L1D_PARAMS_PASS) u_nc (
        .clk                  (clk),
        .rst                  (rst),
        .allocate_valid       (n_alloc),
        .allocate_seat        (s2_seat),
        .allocate_is_write    (s2_req[REQ_RDWR_BIT]),
        .allocate_access_size (s2_size),
        .allocate_master_id   (s2_mstid),
        .allocate_rob_index   (s2_idx),
        .allocate_port        (s2_port),                    // A2: the NC answer goes back to this port
        .allocate_addr        (s2_addr),
        .allocate_store_data  (s2_wdata),
        .can_allocate         (nc_can),
        .axi_read_req_valid   (n_rd_valid),
        .axi_read_req_ready   (n_rd_ready),
        .axi_read_req_addr    (n_rd_addr),
        .axi_read_req_size    (n_rd_size),
        .axi_read_beat_valid  (axi_read_beat_nc),
        .axi_read_beat_data   (axi_read_data),
        .axi_read_beat_error  (axi_read_data_error),
        .axi_write_req_valid  (n_wr_valid),
        .axi_write_req_ready  (n_wr_ready),
        .axi_write_req_addr   (n_wr_addr),
        .axi_write_req_size   (n_wr_size),
        .axi_write_req_data   (n_wr_data),
        .axi_write_req_strobe (n_wr_strb),
        .axi_write_resp_valid (axi_write_resp_nc),
        .axi_write_resp_error (axi_write_resp_error),
        .response_valid       (n_resp_valid),
        .response_packet      (n_resp_pkt),
        .response_seat        (n_resp_seat),
        .response_taken       (n_resp_ack),
        .queue_empty          (nc_empty),
        .entry_freed_pulse    (nc_freed)
    );

    // victim cache
    // victim insert: a refill's victim (VALID) or a flush/clean line (WBPEND).  vc_rel: a swap that installed into an empty way
    // leaves its MOVING slot without a victim, so the slot is released. Lines enter and leave raw (no ECC bits, PLAN_v6 O-5).
    l1d_victim_cache #(`L1D_PARAMS_PASS) u_vc (
        .clk                         (clk),
        .rst                         (rst),
        .lookup_line_addr_next       (s1_line),             // TIMING OPT (V-5): s1_line becomes s2_line at the next edge (no stall)
        .lookup_line_addr            (s2_line),
        .lookup_hit                  (vc_hit),
        .lookup_slot                 (vc_slot),
        .lookup_slot_state           (vc_state),
        .lookup_line_data            (vc_data),
        .lookup_dirty                (vc_dirty),
        .lookup_shared               (vc_shared),
        .victim_slot_available       (vc_al_ok),
        .victim_slot_index           (vc_al_slot),
        .insert_valid                (vc_ins),
        .insert_slot                 (s2_slot),
        .insert_line_addr            (s2_vline),
        .insert_line_data            (line_q),
        .insert_dirty                (s2_vdirty),
        .insert_shared               (s2_vshared),
        .insert_as_writeback_pending (s2_wbpend),
        .swap_start_valid            (vc_swap),
        .swap_start_slot             (vc_slot),
        .release_valid               (vc_rel),
        .release_slot                (s2_slot),
        .snoop_hit_valid             (vc_snp),
        .snoop_hit_slot              (vc_slot),
        .snoop_invalidate            (s2_sinv),
        .drain_enable                (vc_drain),
        .writeback_req_valid         (vc_wr_valid),
        .writeback_req_ready         (vc_wr_ready),
        .writeback_req_line_addr     (vc_wr_line),
        .writeback_req_slot_id       (vc_wr_sub),
        .writeback_req_data          (vc_wr_data),
        .writeback_done_valid        (axi_write_resp_cacheable),
        .writeback_done_slot_id      (axi_write_resp_sub_id[VC_ID_W-1:0]),
        .all_slots_free              (vc_empty),
        .slot_freed_pulses           (vc_freed)
    );

    // flush / clean walker. The bank is idle when no tracker entry is in use, the pipeline and the miss units are empty and
    // no AMO lock is held (a held lock keeps the bank busy until its AMO write or the time-out).
    wire bank_idle;                                       // nothing in flight in this bank
    assign bank_idle = tracker_empty && !r_valid && !s1_valid && !s2_valid && !s3_amo_v && !mshr_busy && nc_empty && !lock_valid;

    l1d_maint #(`L1D_PARAMS_PASS) u_maint (
        .clk                (clk),
        .rst                (rst),
        .maint_start        (maint_start),
        .maint_op_is_flush  (maint_op_is_flush),
        .bank_idle          (bank_idle),
        .victim_cache_empty (vc_empty),
        .victim_cache_drain (vc_drain),
        .token_valid        (mt_valid),
        .token_set          (mt_set),
        .token_way          (mt_way),
        .token_op_is_flush  (mt_op),
        .token_granted      (gnt_maint),
        .s2_result_valid    (s2_maint),
        .s2_result_held     (s2_held),
        .maint_done         (maint_done),
        .maint_busy         ()
    );

endmodule

`default_nettype wire
