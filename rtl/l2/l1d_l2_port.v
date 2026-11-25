// ============================================================================
// l1d_l2_port.v  -  the cache's one connection to the L2 (packet port, cache_v5)
// ============================================================================
// TIMING OPT L2-1 (same values on every output and flop in every cycle, built differently):
//   Old: round robin -> BINARY bank number -> 4:1 field muxes -> NC mask (adder + compare) / NC beat shift -> packet
//        register (about 25 levels, 547 ps).
//   New: (1) every bank's candidate packet fields (address, sub id, nc, unique, mask, data) are built per bank from that
//        bank's own inputs, in parallel with the round robin; (2) the NC read mask is a decode with no adder: bit i =
//        (offset <= i) and (offset >= i + 1 - 2**size), the second compare against a constant per size code, then picked
//        by the size; the NC write beat and strobes are placed by a beat decode instead of a shift; (3) one AND-OR with
//        the ONE-HOT grants, read / write class folded into the picks, chooses the candidate: the grant is one AND-OR
//        level from the packet register; (4) the lowest free tid is also built as a one-hot (prefix OR), which writes the
//        route table and the busy bit directly (no 6-bit decode) and is encoded for the packet tid field.
//   The binary grant is still used for sel_bank (route only). With no request in a class the binary grant is bank 0, so
//   bank 0's pick also covers "no request" (read_any / write_any = 0): every sel_* keeps its old value in every cycle.
//   The old binary-mux forms of sel_read_size / sel_write_line / sel_write_strobe / nc_* stay as the reference (and for
//   the L1D_DEBUG view); they no longer feed the packet, so synthesis removes them.
// TIMING OPT L2-2 (same values on every output and flop in every cycle, built differently):
//   (1) Lowest free tid in two flat levels: the 64 tids are 8 groups of 8; per group "all busy" and, per tid, "all lower
//       tids of my group busy" are computed in parallel, the group prefix runs over 8 flags only, one final AND gives the
//       one-hot (was a serial prefix-OR chain of 6 gates, 170 ps). The tid number is {lowest free group, place in it},
//       both from the group flags. (2) The route table write enable and the busy set use take_if_tid_free (= req_take
//       without the tid term: a one-hot bit already says a tid is free), so the free-tid one-hot is the last AND.
//   (3) The route table is read by a one-hot decode of the answer tid and a flat AND-OR; the answer-register values that
//       depend only on the route (first beat, last beat, line read) are formed per entry before that AND-OR.
//   (4) The bank ready outputs and the picks use the turn terms read_turn / write_turn directly (a grant bit implies its
//       class has a request), so read_any / write_any / pick_write are no longer in series before the grant AND.
// TIMING OPT L2-3 (same values, built differently):
//   (1) The lowest-free one-hot is built from three radix-4 any-free levels on the free bits (quad of 4, block of 4
//       quads, all): tid i = free and no free tid lower in its quad, its block or below its block; each "lower" term is
//       an OR of at most 3 terms. tid_take_onehot = take_if_tid_free & that one-hot, with the take folded into the
//       early quad term; the route table write and the busy set share it (one net, no extra AND after the prefix).
//       The tid number keeps the L2-2 group form. take_if_requested uses |tid_block_free (= tid_any_free, 3 levels).
//   (2) The candidate select uses the raw one-hot grants per class (4-term AND-OR each), then the class turn:
//       read_turn & read value | write_turn & write value | no_request & bank 0 read. The turn and no-request terms
//       leave the grant fan-out path. cand_pick / read_pick_onehot / write_pick_onehot stay as reference names.
//   Note: ASAP7 has no enable flop; every enable register is a plain flop with a 2-level mux (NAND2 + OA21) at D, and
//   that mux cannot be removed by the RTL style.
// TIMING OPT L2-5 (same values, serial chains made parallel):
//   (1) The two round robins are l1d_arb_rr_flat (same ports, same register mask_r, same grant and next mask for EVERY
//       mask_r value): requester j wins when it asks and no asking requester beats it, all pairs compared at once
//       (2 to 3 gates from mask_r / the requests to the grant; was masked -> prefix OR -> use_masked -> 2:1 mux).
//   (2) The radix-4 free-tid flags (quad, block, lower-in-quad, lower quads, lower blocks, early take term) carry
//       (* keep *): synthesis re-shaped the tree into 6 chained OR gates; kept, the take one-hot is 4 levels from
//       tid_busy (OR4 quad, OR4 block, OR3 lower blocks, one 3-input AND-NOT).
//   (3) The tid number is built from the same radix-4 flags (3 digits: place in the quad, quad in the block, block),
//       each digit an AND-OR of the "first" flags of one level (was the 8 x 8 group form, about 7 levels; now about 5).
//       The group form and its names are removed.
//   (4) req_take uses take_if_requested (= ~req_reg_valid & |tid_block_free, the kept 3-level any-free), same value
//       as ~req_reg_valid & tid_any_free.
// TIMING OPT L2-9 (one new register, no new cycle, same value on every output and old flop in every cycle):
//   (1) The quad flags ("this group of TID_QUAD_SIZE = 4 tids has a free tid") are the register tid_quad_free_q
//       (16 flops, reset all ones = every tid free) instead of an OR of the 64 tid_busy flops, so the lowest-free
//       search starts from flop outputs: block flags, "lower quads" and "lower blocks" are ORs of flag flops.
//   (2) Invariant (kept by construction, proven by the equivalence induction step): tid_quad_free_q[q] = OR of the free
//       bits of quad q, in every cycle. Next value, with free'[k] = (free[k] & ~set[k]) | clear[k] (the clear wins):
//         next[q] = clear in quad q | two or more tids of quad q free | (tid_quad_free_q[q] & ~take in quad q)
//       "take in quad q" (tid_quad_take) = take_if_tid_free & quad q is the lowest quad with a free tid (from the flag
//       flops); a take there takes the one first free tid, so the quad stays free only if a second one was free.
//       "two or more free" = some free tid of the quad has a lower free tid in the quad (tid_bits_below_free).
//   (3) The take one-hot is tid_quad_take (take term and quad search, one kept net per quad) AND the first free tid of
//       its quad (from the 4 busy flops of the quad), so the route table write / busy set enable is one AND after it.
//       The old level-1 wire tid_quad_free and the early term tid_take_early are removed.
// ============================================================================
// IN SIMPLE WORDS
//   Every bank wants to read lines from the L2 (refills), write lines back (victim write-backs) and do single
//   non-cacheable (NC) accesses. There is ONE request wire bundle and ONE response wire bundle to the L2. This block
//   shares them, like a post office with one counter:
//     - REQUEST : each bank offers at most one read and one write. A round-robin picks a bank; reads and writes take
//                 turns when both wait. The chosen request gets a free transaction number (tid), is packed into one
//                 packet and held in the request register until the L2 takes it (l2_req_ready).
//     - RESPONSE: the L2 answers each request once, with the same tid. The tid tells which bank and which MSHR entry or
//                 victim slot the answer belongs to (route table). A line read is handed to the bank as AXI_BEATS beats
//                 (one per cycle, the bank's MSHR collects them); a write answer is one cycle.
//   No AXI here (user decision 2026-11-05: no AXI signals inside the D-cache). AXI exists only in the dummy L2
//   (../../dummy_l2/l2_pkt_axi_bridge.v), which turns these packets into AXI for the test memory.
//
// WHERE IT SITS
//   Parent: l1d_top (instance u_l2_port). Not a pipeline stage: a miss already left the bank pipeline when it asks here.
//   Built from cache_v1_4's l1d_l2_port (same request / response logic); the bank side keeps cache_v2_bank_pipe's names so
//   the banks connect unchanged; v5 adds the unique / shared bit `us` and the replay-busy output below.
//
// PACKET (packet v2 + 1 bit, MSB first, L2_PKT_W = 643 bits at the defaults)
//   us 1 | address 56 | rw 1 | master id 2 | trans id 6 | nc 1 | byte mask 64 | data 512
//   us in a request : 1 = read the line UNIQUE (store, LR, SC, AMO; was ARUSER[0] in cache_v2_bank_pipe)
//   us in a response: 1 = another cache keeps a copy, install the line SHARED (was RUSER[0])
//   line read : mask all ones, data 0          line write: mask all ones, data = the line
//   NC read   : mask = the 1 / 2 / 4 / 8 bytes NC write  : the same mask; data = the NC beat at its place in the line
//
// WHY refill_replay_busy (new in v5, README section 5)
//   The L2 sends a whole line in one packet, but the bank's MSHR takes it as AXI_BEATS beats over AXI_BEATS cycles. A snoop
//   to that line, handled in the bank while the beats are still coming, would see the MSHR entry "without data" and answer
//   "not present" although the line is about to be installed (a stale copy). The L2 contract (no response for a line while
//   a snoop on it is open) cannot cover this, because the beats are inside the cache. So while this block is handing a
//   line to bank b, refill_replay_busy[b] = 1 and bank b does not start a snoop (same hold as an AMO read in flight).
//   The hold lasts at most AXI_BEATS cycles (the bank always takes a beat), so it is bounded (rule S-1).
//
// PARAMETERS (defaults in brackets)
//   the cache parameters `L1D_PARAMS (NUM_BANKS 4, LINE_BYTES 64, AXI_DATA_W 128 = beat width inside the cache, ...)
//   L2_ADDR_W    [56] packet address bits (Rev K physical address)
//   L2_MID_W     [2]  packet master id bits
//   L2_TID_W     [6]  packet transaction id bits (64 tids)
//   L2_MASTER_ID [0]  master id this cache puts in every request
//
// PORTS (direction, width at the defaults, meaning)
//   clk, rst                          in  1     clock; synchronous reset, active high
//   bank_read_req_valid               in  4     one bit per bank: it offers a read
//   bank_read_req_ready               out 4     one bit per bank: its read was taken this cycle
//   bank_read_req_addr                in  4x64  read address of each bank
//   bank_read_req_burst_len           in  4x8   beats - 1 of each read (checked only, see EXCEPTIONS)
//   bank_read_req_size                in  4x3   bytes of an NC read as a power of two
//   bank_read_req_sub_id              in  4x2   MSHR entry / NC queue place of each read
//   bank_read_req_non_cacheable       in  4     NC flag of each read
//   bank_read_req_unique              in  4     the read wants the line unique
//   bank_read_data_valid              out 4     one bit per bank: a read beat for it this cycle
//   bank_read_data_sub_id, _non_cacheable, bank_read_data (128), _last, _error, _shared   out  the beat and its route
//   bank_write_req_valid / _ready / _addr / _burst_len / _size / _sub_id / _non_cacheable / _data (4x512) / _strobe (4x16)
//   bank_write_resp_valid (4) / _sub_id / _non_cacheable / _error                          out  the write answer
//   bank_refill_replay_busy           out 4     a line is being handed to the bank (hold snoops)
//   l2_error_pulse                    out 1     an answer came with l2_resp_err = 1
//   l2_req_valid / l2_req_ready / l2_req_pkt (643)     request packet, valid / ready
//   l2_resp_valid / l2_resp_ready / l2_resp_pkt (643) / l2_resp_err   response packet, valid / ready, error flag
//
// WORKED EXAMPLE
//   Bank 2, MSHR entry 1, misses on line 0x...1080 for a store. Cycle t: bank_read_req_valid[2] = 1, unique = 1; tid 0 is
//   free, so the packet {us 1, addr 0x..1080, rw 0, mid 0, tid 0, nc 0, mask all ones, data 0} goes into the request
//   register and route[0] = {read, line, bank 2, sub 1, beat 0}. The L2 answers tid 0 with the line and us = 0 at cycle u.
//   Cycles u+1 .. u+4: bank_read_data_valid[2] = 1 with beats 0..3, last on beat 3, sub id 1; bank_refill_replay_busy[2] = 1.
//
// WAIT BOUNDS (DESIGN_RULES S-1)
//   a bank waits at most NUM_BANKS - 1 grants of the same class (round robin); a read waits at most 1 write grant and a
//   write 1 read grant (turns); a request without a free tid waits until one answer comes back (the L2 always answers;
//   at the defaults up to NUM_BANKS * (MSHR_ENTRIES + VC_SLOTS + MSHR_NC_ENTRIES) = 4 * 18 = 72 requests could be out, more
//   than the 64 tids, so this wait can happen but is bounded); the request register waits for l2_req_ready (the L2's bound).
//
// ASSUMPTIONS (V-9.2)
//   - A bank holds its read / write request until it sees *_ready (as for cache_v2_bank_pipe's AXI master).
//   - The L2 answers every request exactly once, with the request's tid, and keeps the snoop contract (README section 5).
//   - At most 2**L2_TID_W requests are out at once; with fewer free tids a request simply waits for one (no loss).
//   - PA[AXI_ADDR_W-1:L2_ADDR_W] of every request is 0 (Rev K PA is 56 bits); reported under L1D_DEBUG.
//   - A bank always takes a beat (the old RREADY = 1).
//
// EXCEPTIONS (written reason, CODING_RULES V-5.23)
//   bank_read_req_burst_len, bank_write_req_burst_len and bank_write_req_size are not used for the packet (a line is always
//   AXI_BEATS beats, an NC write is described by its strobes). They are kept so the bank side is the same as
//   cache_v2_bank_pipe's and the banks connect unchanged; they are checked under L1D_DEBUG.
//
// TIMING [Estimate, not timed]: request: bank request -> round robin (4) -> 4:1 field mux -> NC mask / beat shift ->
//   request register: about 10 to 12 levels. Response: route table read (64:1 of 8 bits) -> answer register: about 7.
//   Free tid: 64-bit lowest-free encoder, about 6 levels. Off every promised-cycle path (rule GR-1 applies to the hit path).
// ============================================================================

`ifndef L1D_L2_PORT_V
`define L1D_L2_PORT_V

`default_nettype none
`include "l1d_defines.vh"

module l1d_l2_port #(
    `L1D_PARAMS,                       // the cache parameters (l1d_defines.vh), defaults: 4 banks, 64 B lines, 128-bit beats
    parameter L2_ADDR_W    = 56,       // default 56: packet address bits (Rev K physical address)
    parameter L2_MID_W     = 2,        // default 2:  packet master id bits
    parameter L2_TID_W     = 6,        // default 6:  packet transaction id bits (64 tids)
    parameter L2_MASTER_ID = 0         // default 0:  master id this cache puts in every request
) (
    clk, rst,
    bank_read_req_valid, bank_read_req_ready, bank_read_req_addr, bank_read_req_burst_len, bank_read_req_size,
    bank_read_req_sub_id, bank_read_req_non_cacheable, bank_read_req_unique,
    bank_read_data_valid, bank_read_data_sub_id, bank_read_data_non_cacheable, bank_read_data, bank_read_data_last,
    bank_read_data_error, bank_read_data_shared,
    bank_write_req_valid, bank_write_req_ready, bank_write_req_addr, bank_write_req_burst_len, bank_write_req_size,
    bank_write_req_sub_id, bank_write_req_non_cacheable, bank_write_req_data, bank_write_req_strobe,
    bank_write_resp_valid, bank_write_resp_sub_id, bank_write_resp_non_cacheable, bank_write_resp_error,
    bank_refill_replay_busy,
    l2_error_pulse,
    l2_req_valid, l2_req_ready, l2_req_pkt,
    l2_resp_valid, l2_resp_ready, l2_resp_pkt, l2_resp_err
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // ---------------- sizes and packet field positions ----------------
    localparam NB        = NUM_BANKS;                                                       // 4 banks
    localparam MASK_W    = LINE_BYTES;                                                      // 64 bits, one per byte of the line
    localparam L2_PKT_W  = 1 + L2_ADDR_W + 1 + L2_MID_W + L2_TID_W + 1 + MASK_W + LINE_W;  // 1+56+1+2+6+1+64+512 = 643 bits
    localparam MASK_LO   = LINE_W;                                                          // 512: byte mask starts here
    localparam NC_LO     = MASK_LO + MASK_W;                                                // 576: nc bit
    localparam TID_LO    = NC_LO + 1;                                                       // 577: trans id starts here
    localparam MID_LO    = TID_LO + L2_TID_W;                                               // 583: master id starts here
    localparam RW_LO     = MID_LO + L2_MID_W;                                               // 585: rw bit
    localparam ADDR_LO   = RW_LO + 1;                                                       // 586: address starts here
    localparam US_BIT    = ADDR_LO + L2_ADDR_W;                                             // 642: unique / shared bit (MSB)
    localparam NTID      = 1 << L2_TID_W;                                                   // 64 tids
    localparam APAD_W    = AXI_ADDR_W - L2_ADDR_W;                                          // 8 address bits above the packet address
    localparam [BEAT_W-1:0]   LAST_BEAT = AXI_BEATS - 1;                                    // 3: last beat of a line
    localparam [7:0]          LINE_LEN  = AXI_BEATS - 1;                                    // 3: beats - 1 of a line (debug check only)
    localparam [L2_MID_W-1:0] MASTER_ID_C = L2_MASTER_ID;                                   // master id, 2 bits
    // route of a request, written at its tid: {is_write, is_nc, bank, sub id, beat}
    localparam ROUTE_W   = 2 + BANK_W + AXI_SUBID_W + BEAT_W;                               // 2+2+2+2 = 8 bits
    // TIMING OPT L2-1: candidate requests and the fields of one candidate (low to high)
    localparam NC_SIZE_W       = 3;                                                         // 3 bits: NC read size code (bytes = 2**code)
    localparam NC_SIZE_CODES   = 1 << NC_SIZE_W;                                            // 8 size codes (0..7)
    localparam CANDIDATES      = 2 * NB;                                                    // 8 candidates: NB reads (low), NB writes (high)
    localparam CAND_ADDR_LO    = 0;                                                         // 0: address, AXI_ADDR_W = 64 bits
    localparam CAND_SUB_LO     = CAND_ADDR_LO + AXI_ADDR_W;                                 // 64: sub id, AXI_SUBID_W = 2 bits
    localparam CAND_NC_BIT     = CAND_SUB_LO + AXI_SUBID_W;                                 // 66: nc bit
    localparam CAND_UNIQUE_BIT = CAND_NC_BIT + 1;                                           // 67: packet bit us
    localparam CAND_MASK_LO    = CAND_UNIQUE_BIT + 1;                                       // 68: packet mask, MASK_W = 64 bits
    localparam CAND_DATA_LO    = CAND_MASK_LO + MASK_W;                                     // 132: packet data, LINE_W = 512 bits
    localparam CAND_W          = CAND_DATA_LO + LINE_W;                                     // 644 bits per candidate
    // TIMING OPT L2-3: radix-4 levels for the lowest-free one-hot (quad = 4 tids, block = 4 quads)
    localparam TID_DIGIT_W     = L2_TID_W / 3;                                              // 2 bits: one radix level of the tid number
    localparam TID_QUAD_SIZE   = 1 << TID_DIGIT_W;                                          // 4 tids per quad
    localparam TID_QUADS       = NTID / TID_QUAD_SIZE;                                      // 16 quads
    localparam TID_BLOCK_SIZE  = TID_QUAD_SIZE * TID_QUAD_SIZE;                             // 16 tids per block
    localparam TID_BLOCKS      = NTID / TID_BLOCK_SIZE;                                     // 4 blocks
    // TIMING OPT L2-5: the three digits of the tid number (place in the quad, quad in the block, block), low to high
    localparam TID_QUAD_DIGIT_LO  = TID_DIGIT_W;                                            // 2: tid bits [3:2] = quad in the block
    localparam TID_BLOCK_DIGIT_LO = 2 * TID_DIGIT_W;                                        // 4: tid bits [5:4] = block

    // ---------------- ports ----------------
    input  wire                        clk;                            // clock: all registers change on its rising edge
    input  wire                        rst;                            // synchronous reset, active high
    // banks: read requests
    input  wire [NB-1:0]               bank_read_req_valid;            // NB = 4 bits: bank offers a read
    output wire [NB-1:0]               bank_read_req_ready;            // 4 bits: its read was taken this cycle
    input  wire [NB*AXI_ADDR_W-1:0]    bank_read_req_addr;             // NB*AXI_ADDR_W = 256 bits: read address per bank
    input  wire [NB*8-1:0]             bank_read_req_burst_len;        // 32 bits: beats - 1 per bank (debug check only)
    input  wire [NB*3-1:0]             bank_read_req_size;             // 12 bits: NC read size (power of two bytes)
    input  wire [NB*AXI_SUBID_W-1:0]   bank_read_req_sub_id;           // NB*AXI_SUBID_W = 8 bits: MSHR entry / NC place
    input  wire [NB-1:0]               bank_read_req_non_cacheable;    // 4 bits: NC read
    input  wire [NB-1:0]               bank_read_req_unique;           // 4 bits: read the line unique (packet bit us)
    // banks: read data (shared beat bus, one valid per bank)
    output wire [NB-1:0]               bank_read_data_valid;           // 4 bits: a beat for this bank
    output wire [AXI_SUBID_W-1:0]      bank_read_data_sub_id;          // AXI_SUBID_W = 2 bits: MSHR entry / NC place
    output wire                        bank_read_data_non_cacheable;   // the beat answers an NC read
    output wire [AXI_DATA_W-1:0]       bank_read_data;                 // AXI_DATA_W = 128 bits: the beat
    output wire                        bank_read_data_last;            // last beat of the answer
    output wire                        bank_read_data_error;           // the answer came with an error
    output wire                        bank_read_data_shared;          // another cache keeps the line (install S)
    // banks: write requests
    input  wire [NB-1:0]               bank_write_req_valid;           // 4 bits: bank offers a write
    output wire [NB-1:0]               bank_write_req_ready;           // 4 bits: its write was taken this cycle
    input  wire [NB*AXI_ADDR_W-1:0]    bank_write_req_addr;            // 256 bits: write address per bank
    input  wire [NB*8-1:0]             bank_write_req_burst_len;       // 32 bits: not used (debug check only)
    input  wire [NB*3-1:0]             bank_write_req_size;            // 12 bits: not used (debug check only)
    input  wire [NB*AXI_SUBID_W-1:0]   bank_write_req_sub_id;          // 8 bits: victim slot / NC place
    input  wire [NB-1:0]               bank_write_req_non_cacheable;   // 4 bits: NC write
    input  wire [NB*LINE_W-1:0]        bank_write_req_data;            // NB*LINE_W = 2048 bits: the line (NC: beat in the low bits)
    input  wire [NB*AXI_STRB_W-1:0]    bank_write_req_strobe;          // NB*AXI_STRB_W = 64 bits: NC write byte strobes
    // banks: write answers
    output wire [NB-1:0]               bank_write_resp_valid;          // 4 bits: a write answer for this bank
    output wire [AXI_SUBID_W-1:0]      bank_write_resp_sub_id;         // 2 bits: victim slot / NC place
    output wire                        bank_write_resp_non_cacheable;  // the answer is for an NC write
    output wire                        bank_write_resp_error;          // the answer came with an error
    // banks: snoop hold while a line is handed over (see header)
    output wire [NB-1:0]               bank_refill_replay_busy;        // 4 bits: a line is being handed to this bank
    // status
    output wire                        l2_error_pulse;                 // one cycle: an answer came with l2_resp_err = 1
    // L2 packet port
    output wire                        l2_req_valid;                   // a request packet is offered
    input  wire                        l2_req_ready;                   // the L2 takes it
    output wire [L2_PKT_W-1:0]         l2_req_pkt;                     // L2_PKT_W = 643 bits
    input  wire                        l2_resp_valid;                  // a response packet is offered
    output wire                        l2_resp_ready;                  // this block takes it
    input  wire [L2_PKT_W-1:0]         l2_resp_pkt;                    // 643 bits
    input  wire                        l2_resp_err;                    // the response is an error

    // ======================================================================
    // Declarations
    // ======================================================================
    // round-robin choice among the banks
    wire [NB-1:0]           read_grant_onehot;     // 4 bits: bank whose read is next
    wire [BANK_W-1:0]       read_grant_bank;       // BANK_W = 2 bits: its number
    wire                    read_any;              // some bank offers a read
    wire [NB-1:0]           write_grant_onehot;    // 4 bits: bank whose write is next
    wire [BANK_W-1:0]       write_grant_bank;      // 2 bits: its number
    wire                    write_any;             // some bank offers a write
    wire                    pick_write;            // this cycle's request is a write
    wire                    req_take;              // a request is taken into the request register this cycle
    wire                    read_take;             // ... and it is a read
    wire                    write_take;            // ... and it is a write
    reg                     last_was_write;        // the last request taken was a write (reads and writes take turns)
    // transaction ids
    reg  [NTID-1:0]         tid_busy;              // NTID = 64 bits: tid in use
    wire [NTID-1:0]         tid_free;              // 64 bits: tid free
    wire [L2_TID_W-1:0]     tid_new;               // 6 bits: lowest free tid
    wire                    tid_any_free;          // a tid is free
    // fields of the chosen request
    wire [BANK_W-1:0]       sel_bank;              // 2 bits
    wire [AXI_ADDR_W-1:0]   sel_addr;              // 64 bits
    wire                    sel_nc;                // non-cacheable
    wire                    sel_unique;            // line read wanted unique
    wire [AXI_SUBID_W-1:0]  sel_sub;               // 2 bits: MSHR entry / victim slot / NC place
    wire [2:0]              sel_read_size;         // NC read size
    wire [LINE_W-1:0]       sel_write_line;        // 512 bits: write line (NC: beat in the low bits)
    wire [AXI_STRB_W-1:0]   sel_write_strobe;      // 16 bits: NC write strobes
    wire [OFFSET_W-1:0]     sel_offset;            // 6 bits: byte offset in the line
    wire [BEAT_W-1:0]       sel_beat;              // 2 bits: beat of the address
    wire [LINE_W-1:0]       nc_write_line;         // 512 bits: NC write beat at its place in the line
    wire [MASK_W-1:0]       nc_write_mask;         // 64 bits: NC write mask
    reg  [MASK_W-1:0]       nc_read_mask;          // 64 bits: NC read mask
    wire [MASK_W-1:0]       sel_mask;              // 64 bits: packet mask
    wire [LINE_W-1:0]       sel_data;              // 512 bits: packet data
    wire [L2_PKT_W-1:0]     sel_pkt;               // 643 bits: packet
    wire [ROUTE_W-1:0]      sel_route;             // 8 bits: route of the new request
    integer                 nc_read_bytes;         // NC read size in bytes
    integer                 mask_bit;              // loop index over the mask bits
    // request register
    reg                     req_reg_valid;         // the request register holds a packet
    reg  [L2_PKT_W-1:0]     req_reg_pkt;           // 643 bits
    // route table: written at the request, read at the answer
    reg  [ROUTE_W-1:0]      route_table [0:NTID-1]; // 64 x 8 bits
    wire [L2_TID_W-1:0]     resp_tid;              // 6 bits: tid of the answer
    wire [ROUTE_W-1:0]      resp_route;            // 8 bits: its route
    wire                    resp_take;             // an answer is taken this cycle
    // answer register (hands the answer to the bank)
    reg                     ans_valid;             // an answer is being given to a bank
    reg                     ans_is_write;          // 1 = write answer (one cycle)
    reg                     ans_nc;                // NC answer
    reg                     ans_shared;            // line read: another cache keeps the line
    reg  [BANK_W-1:0]       ans_bank;              // 2 bits
    reg  [AXI_SUBID_W-1:0]  ans_sub;               // 2 bits
    reg                     ans_err;               // error flag
    reg  [LINE_W-1:0]       ans_line;              // 512 bits: answer data
    reg  [BEAT_W-1:0]       ans_beat;              // 2 bits: beat given now
    reg  [BEAT_W-1:0]       ans_end_beat;          // 2 bits: last beat
    wire                    ans_last;              // this cycle ends the answer
    wire                    resp_ready_int;        // internal copy of l2_resp_ready
    reg                     err_pulse_q;           // error pulse register
    genvar                  bank_idx;
    // TIMING OPT L2-1: per-bank candidates, one-hot pick, one-hot free tid
    wire [NB-1:0]                read_pick_onehot;     // 4 bits: this bank's read is the request of this cycle (bank 0 also when no read waits)
    wire [NB-1:0]                write_pick_onehot;    // 4 bits: this bank's write is the request of this cycle (bank 0 also when no write waits)
    wire [CANDIDATES-1:0]        cand_pick;            // CANDIDATES = 8 bits: {write picks, read picks}, exactly one is 1
    wire [NB*MASK_W-1:0]         read_nc_mask_all;     // NB*MASK_W = 256 bits: NC read mask of every bank (own offset and size)
    wire [NB*MASK_W-1:0]         write_nc_mask_all;    // 256 bits: NC write strobes of every bank at its own beat
    wire [NB*LINE_W-1:0]         write_nc_line_all;    // NB*LINE_W = 2048 bits: NC write beat of every bank at its own beat
    wire [CANDIDATES*CAND_W-1:0] cand_fields;          // CANDIDATES*CAND_W = 5152 bits: fields of every candidate
    wire [CAND_W*CANDIDATES-1:0] cand_columns;         // 5152 bits: the same, bit-major (the CANDIDATES copies of each field bit together)
    wire [CAND_W-1:0]            sel_cand;             // CAND_W = 644 bits: fields of the picked candidate
    wire [NTID-1:0]              tid_free_below;       // NTID = 64 bits: some lower tid is free
    wire [NTID-1:0]              tid_new_onehot;       // 64 bits: lowest free tid, one-hot (zero when none is free)
    wire [L2_TID_W*NTID-1:0]     tid_index_terms;      // L2_TID_W*NTID = 384 bits: one-hot bits whose tid has this index bit set
    wire [L2_TID_W-1:0]          tid_new_encoded;      // L2_TID_W = 6 bits: number of the lowest free tid (= tid_new)
    integer                      tid_slot;             // loop index over the tids (route table write)
    genvar                       mask_idx;             // mask bit / byte of the line
    genvar                       size_idx;             // NC read size code
    genvar                       beat_idx;             // beat of the line
    genvar                       cand_idx;             // candidate request
    genvar                       field_bit;            // bit of the candidate fields
    genvar                       tid_idx;              // tid
    genvar                       tid_bit;              // bit of a tid number
    // TIMING OPT L2-2: early take terms, one-hot route read (L2-5: the group form of the free tid is removed)
    wire                           take_if_requested;      // request register empty and a tid free (flops only)
    wire                           take_if_tid_free;       // request register empty and a request waits (= req_take when a tid is free)
    wire                           read_turn;              // a granted read wins the read / write turn
    wire                           write_turn;             // a granted write wins the read / write turn
    wire [NTID-1:0]                resp_tid_onehot;        // 64 bits: decode of the answer tid
    wire [ROUTE_W*NTID-1:0]        route_columns;          // ROUTE_W*NTID = 512 bits: route table bit-major (the 64 copies of a route bit together)
    wire [BEAT_W*NTID-1:0]         route_start_columns;    // BEAT_W*NTID = 128 bits: first beat to hand over, per entry, bit-major
    wire [BEAT_W*NTID-1:0]         route_end_columns;      // 128 bits: last beat to hand over, per entry, bit-major
    wire [NTID-1:0]                route_line_read_column; // 64 bits: entry is a line read (not a write, not NC)
    wire [BEAT_W-1:0]              resp_start_beat;        // BEAT_W = 2 bits: first beat of the answer (NC: its own beat)
    wire [BEAT_W-1:0]              resp_end_beat;          // 2 bits: last beat of the answer
    wire                           resp_line_read;         // the answer is for a line read (may install shared)
    genvar                         place_idx;              // place of a tid in its quad / of a quad in its block (L2-5)
    genvar                         route_bit;              // bit of a route
    genvar                         beat_bit;               // bit of a beat number
    // TIMING OPT L2-3: radix-4 free-tid levels, shared take one-hot, grant-first candidate select
    // TIMING OPT L2-5: (* keep *) fixes each level as its own net, so synthesis keeps the 4-level tree (it re-shaped
    // the unkept tree into 6 chained OR gates, 286 ps in iteration 3)
    // TIMING OPT L2-9: the quad flags are the register tid_quad_free_q (level 0); the levels below count from it
    (* keep *) wire [TID_QUADS-1:0]  tid_quads_below_free; // TID_QUADS = 16 bits: a lower quad of the same block has a free tid (level 1)
    (* keep *) wire [TID_BLOCKS-1:0] tid_block_free;       // TID_BLOCKS = 4 bits: the block has a free tid (level 1)
    (* keep *) wire [TID_BLOCKS-1:0] tid_blocks_below_free; // 4 bits: a lower block has a free tid (level 2)
    (* keep *) wire [NTID-1:0]       tid_bits_below_free;  // NTID = 64 bits: a lower tid of the same quad is free (level 1 from tid_busy)
    wire [NTID-1:0]                tid_take_onehot;        // 64 bits: = take_if_tid_free & tid_new_onehot (route write, busy set; level 4)
    // TIMING OPT L2-9: registered quad flags and their next value (invariant: flag = OR of the quad's free bits)
    reg  [TID_QUADS-1:0]           tid_quad_free_q;        // TID_QUADS = 16 bits: the quad has a free tid (register, reset all ones)
    (* keep *) wire [TID_QUADS-1:0]  tid_quad_take;        // 16 bits: take_if_tid_free and this is the lowest quad with a free tid (level 3)
    wire [TID_QUADS-1:0]           tid_quad_two_free;      // 16 bits: two or more tids of the quad are free (from tid_busy)
    wire [TID_QUADS-1:0]           tid_quad_clear;         // 16 bits: the answer taken this cycle frees a tid of this quad
    wire [TID_QUADS-1:0]           tid_quad_free_next;     // 16 bits: next value of tid_quad_free_q (= OR of the next free bits)
    wire                           no_request;             // no bank offers a read or a write
    wire [CAND_W-1:0]              read_grant_value;       // CAND_W = 644 bits: fields of the granted read (zero without a read)
    wire [CAND_W-1:0]              write_grant_value;      // 644 bits: fields of the granted write (zero without a write)
    genvar                         quad_idx;               // tid quad
    genvar                         block_idx;              // tid block
    // TIMING OPT L2-5: tid number from the radix-4 flags (three digits, each one AND-OR of "first" flags)
    wire [NTID-1:0]                tid_in_quad_first;      // NTID = 64 bits: free and no lower free tid in its quad
    wire [TID_QUADS*TID_DIGIT_W-1:0]  tid_quad_place;      // TID_QUADS*TID_DIGIT_W = 32 bits: place of the quad's first free tid
    wire [TID_QUADS-1:0]           tid_quad_first_in_block; // TID_QUADS = 16 bits: lowest quad with a free tid in its block
    wire [TID_BLOCKS*TID_DIGIT_W-1:0] tid_block_place;     // TID_BLOCKS*TID_DIGIT_W = 8 bits: place in its quad of the block's first free tid
    wire [TID_BLOCKS*TID_DIGIT_W-1:0] tid_block_quad;      // 8 bits: quad (in the block) of the block's first free tid
    wire [TID_BLOCKS-1:0]          tid_block_first;        // TID_BLOCKS = 4 bits: lowest block with a free tid, one-hot (zero when none)
    genvar                         digit_bit;              // bit of one tid digit

    // ======================================================================
    // Request: round robin per class, read / write turn, free tid
    // ======================================================================
    l1d_arb_rr_flat #(.N(NB), .IDX_W(BANK_W)) u_read_rr (   // which bank's read goes next (L2-5: flat grant)
        .clk          (clk),
        .rst          (rst),
        .request_bits (bank_read_req_valid),
        .grant_taken  (read_take),
        .grant_onehot (read_grant_onehot),
        .grant_index  (read_grant_bank),
        .any_request  (read_any)
    );

    l1d_arb_rr_flat #(.N(NB), .IDX_W(BANK_W)) u_write_rr (  // which bank's write goes next (L2-5: flat grant)
        .clk          (clk),
        .rst          (rst),
        .request_bits (bank_write_req_valid),
        .grant_taken  (write_take),
        .grant_onehot (write_grant_onehot),
        .grant_index  (write_grant_bank),
        .any_request  (write_any)
    );

    assign tid_free = ~tid_busy;

    l1d_pri_enc #(.N(NTID), .IDX_W(L2_TID_W)) u_free_tid (   // lowest free tid (many may be free: priority is needed)
        .request_bits     (tid_free),
        .lowest_set_index (tid_new),
        .any_set          (tid_any_free)
    );

    // TIMING OPT L2-1: the same lowest free tid as a one-hot; with no free tid it is zero, and its encoded number is then 0
    // like tid_new.
    // TIMING OPT L2-3 / L2-5: built from three radix-4 any-free levels on the free bits (quad of 4 tids, block of 4
    // quads, all 4 blocks), each level a kept net (* keep *): tid i is the lowest free tid when it is free and no tid is
    // free lower in its quad, in a lower quad of its block or in a lower block. Every "lower" term is an OR of at most
    // TID_QUAD_SIZE - 1 = 3 flags of one level, so tid_busy -> take one-hot is 4 gate levels.
    // TIMING OPT L2-9: the quad level is the register tid_quad_free_q, so the quad search (lower quads, block, lower
    // blocks, tid_quad_take) is 3 levels from flops, and the in-quad search (2 levels from tid_busy) runs beside it.
    generate
        // level 0 (quad flag register) and level 1 (lower quads of the same block)
        for (quad_idx = 0; quad_idx < TID_QUADS; quad_idx = quad_idx + 1) begin : g_tid_quad
            if ((quad_idx % TID_QUAD_SIZE) == 0) begin : g_block_first
                assign tid_quads_below_free[quad_idx] = 1'b0;                              // first quad of its block
            end else begin : g_block_rest
                assign tid_quads_below_free[quad_idx] =                                    // a lower quad of the same block has a free tid
                    |tid_quad_free_q[quad_idx - (quad_idx % TID_QUAD_SIZE) +: (quad_idx % TID_QUAD_SIZE)];
            end
            // L2-5: lowest quad with a free tid inside its block (one AND-NOT of two flags)
            assign tid_quad_first_in_block[quad_idx] = tid_quad_free_q[quad_idx] & ~tid_quads_below_free[quad_idx];
            // L2-9: a take goes to this quad: request waiting, register empty, lowest quad with a free tid (kept, level 3;
            // zero for every quad when no tid is free, so no tid_any_free term is needed)
            assign tid_quad_take[quad_idx] = take_if_tid_free & tid_quad_first_in_block[quad_idx]
                                           & ~tid_blocks_below_free[quad_idx / TID_QUAD_SIZE];
            // L2-9: two or more free tids in the quad = some free tid has a lower free tid in the same quad
            assign tid_quad_two_free[quad_idx] = |(tid_free[quad_idx*TID_QUAD_SIZE +: TID_QUAD_SIZE]          // TID_QUAD_SIZE = 4 tids
                                                 & tid_bits_below_free[quad_idx*TID_QUAD_SIZE +: TID_QUAD_SIZE]);
            // L2-9: the answer taken this cycle frees a tid of this quad (quad number = resp_tid bits above the place digit)
            assign tid_quad_clear[quad_idx] = resp_take & (resp_tid[L2_TID_W-1:TID_DIGIT_W] == quad_idx);   // 4-bit quad number
            // L2-9: next flag = OR of the next free bits, free'[k] = (free[k] & ~set[k]) | clear[k]: a clear in the quad
            // makes it free; else a take in the quad leaves it free only with a second free tid; else it stays as it is
            assign tid_quad_free_next[quad_idx] = tid_quad_clear[quad_idx]                  // late: resp_take (last gate)
                                                | tid_quad_two_free[quad_idx]                // early: tid_busy only
                                                | (tid_quad_free_q[quad_idx] & ~tid_quad_take[quad_idx]);   // late: take
        end
        // level 1 (block) and level 2 (lower blocks)
        for (block_idx = 0; block_idx < TID_BLOCKS; block_idx = block_idx + 1) begin : g_tid_block
            assign tid_block_free[block_idx] = |tid_quad_free_q[block_idx*TID_QUAD_SIZE +: TID_QUAD_SIZE];   // 4 quad flags = 16 tids
            if (block_idx == 0) begin : g_lowest_block
                assign tid_blocks_below_free[block_idx] = 1'b0;                            // no block below block 0
            end else begin : g_upper_block
                assign tid_blocks_below_free[block_idx] = |tid_block_free[block_idx-1:0];  // a lower block has a free tid
            end
            // L2-5: lowest block with a free tid, one-hot (zero when no tid is free)
            assign tid_block_first[block_idx] = tid_block_free[block_idx] & ~tid_blocks_below_free[block_idx];
        end
        // per tid: lower-in-quad flag (level 1), first free of its quad (level 2), take one-hot (level 4)
        for (tid_idx = 0; tid_idx < NTID; tid_idx = tid_idx + 1) begin : g_tid_onehot
            if ((tid_idx % TID_QUAD_SIZE) == 0) begin : g_quad_first
                assign tid_bits_below_free[tid_idx] = 1'b0;                                // first tid of its quad
            end else begin : g_quad_rest
                assign tid_bits_below_free[tid_idx] =                                      // a lower tid of the same quad is free
                    |tid_free[tid_idx - (tid_idx % TID_QUAD_SIZE) +: (tid_idx % TID_QUAD_SIZE)];
            end
            assign tid_in_quad_first[tid_idx] = tid_free[tid_idx] & ~tid_bits_below_free[tid_idx];   // L2-5: first free of its quad
            assign tid_free_below[tid_idx]  = tid_bits_below_free[tid_idx]                 // some lower tid is free
                                            | tid_quads_below_free[tid_idx / TID_QUAD_SIZE]
                                            | tid_blocks_below_free[tid_idx / TID_BLOCK_SIZE];
            assign tid_new_onehot[tid_idx]  = tid_free[tid_idx] & ~tid_free_below[tid_idx];   // free and lowest free (reference)
            // L2-9: = take_if_tid_free & tid_new_onehot: the take quad (kept, from the flag flops) AND the first free tid
            // of that quad (from its 4 busy flops); one AND2 before the route table / busy fan-out
            assign tid_take_onehot[tid_idx] = tid_quad_take[tid_idx / TID_QUAD_SIZE] & tid_in_quad_first[tid_idx];
            // reference terms of L2-1 (one-hot bits whose tid has this index bit set); no longer used, synthesis removes them
            for (tid_bit = 0; tid_bit < L2_TID_W; tid_bit = tid_bit + 1) begin : g_tid_term
                if ((tid_idx >> tid_bit) & 1) begin : g_set
                    assign tid_index_terms[tid_bit*NTID + tid_idx] = tid_new_onehot[tid_idx];   // tid number has this bit set
                end else begin : g_clear
                    assign tid_index_terms[tid_bit*NTID + tid_idx] = 1'b0;                      // tid number has this bit clear
                end
            end
        end
        // TIMING OPT L2-5: tid number = {block, quad in the block, place in the quad} of the lowest free tid, three digits
        // of TID_DIGIT_W = 2 bits. Place of the first free tid of every quad (from the in-quad one-hot, level 2):
        for (quad_idx = 0; quad_idx < TID_QUADS; quad_idx = quad_idx + 1) begin : g_quad_place
            for (digit_bit = 0; digit_bit < TID_DIGIT_W; digit_bit = digit_bit + 1) begin : g_quad_place_bit
                wire [TID_QUAD_SIZE-1:0] place_terms;          // TID_QUAD_SIZE = 4 bits: in-quad one-hot bits with this place bit set
                for (place_idx = 0; place_idx < TID_QUAD_SIZE; place_idx = place_idx + 1) begin : g_quad_place_term
                    if ((place_idx >> digit_bit) & 1) begin : g_place_set
                        assign place_terms[place_idx] = tid_in_quad_first[quad_idx*TID_QUAD_SIZE + place_idx];   // place has the bit
                    end else begin : g_place_clear
                        assign place_terms[place_idx] = 1'b0;                                                    // place lacks the bit
                    end
                end
                assign tid_quad_place[quad_idx*TID_DIGIT_W + digit_bit] = |place_terms;   // OR of the matching places
            end
        end
        // per block: place and quad number of the block's first free tid (AND-OR with the in-block quad one-hot)
        for (block_idx = 0; block_idx < TID_BLOCKS; block_idx = block_idx + 1) begin : g_block_digits
            for (digit_bit = 0; digit_bit < TID_DIGIT_W; digit_bit = digit_bit + 1) begin : g_block_digit_bit
                wire [TID_QUAD_SIZE-1:0] place_terms;          // 4 bits: first quad of the block AND its place bit
                wire [TID_QUAD_SIZE-1:0] quad_terms;           // 4 bits: first quad of the block, if its number has the bit
                for (place_idx = 0; place_idx < TID_QUAD_SIZE; place_idx = place_idx + 1) begin : g_block_digit_term
                    assign place_terms[place_idx] = tid_quad_first_in_block[block_idx*TID_QUAD_SIZE + place_idx]
                                                  & tid_quad_place[(block_idx*TID_QUAD_SIZE + place_idx)*TID_DIGIT_W + digit_bit];
                    if ((place_idx >> digit_bit) & 1) begin : g_quad_set
                        assign quad_terms[place_idx] = tid_quad_first_in_block[block_idx*TID_QUAD_SIZE + place_idx];   // number has the bit
                    end else begin : g_quad_clear
                        assign quad_terms[place_idx] = 1'b0;                                                         // number lacks the bit
                    end
                end
                assign tid_block_place[block_idx*TID_DIGIT_W + digit_bit] = |place_terms;   // place digit of the block
                assign tid_block_quad[block_idx*TID_DIGIT_W + digit_bit]  = |quad_terms;    // quad digit of the block
            end
        end
        // the three digits: AND-OR of the block one-hot with the per-block digits; zero when no tid is free (= tid_new)
        for (digit_bit = 0; digit_bit < TID_DIGIT_W; digit_bit = digit_bit + 1) begin : g_tid_digit
            wire [TID_BLOCKS-1:0] place_terms;                 // TID_BLOCKS = 4 bits: first block AND its place digit bit
            wire [TID_BLOCKS-1:0] quad_terms;                  // 4 bits: first block AND its quad digit bit
            wire [TID_BLOCKS-1:0] block_terms;                 // 4 bits: first block, if its number has the bit
            for (block_idx = 0; block_idx < TID_BLOCKS; block_idx = block_idx + 1) begin : g_tid_digit_term
                assign place_terms[block_idx] = tid_block_first[block_idx] & tid_block_place[block_idx*TID_DIGIT_W + digit_bit];
                assign quad_terms[block_idx]  = tid_block_first[block_idx] & tid_block_quad[block_idx*TID_DIGIT_W + digit_bit];
                if ((block_idx >> digit_bit) & 1) begin : g_block_set
                    assign block_terms[block_idx] = tid_block_first[block_idx];          // block number has the bit
                end else begin : g_block_clear
                    assign block_terms[block_idx] = 1'b0;                                // block number lacks the bit
                end
            end
            assign tid_new_encoded[digit_bit]                      = |place_terms;   // tid bits [1:0]: place in the quad
            assign tid_new_encoded[TID_QUAD_DIGIT_LO + digit_bit]  = |quad_terms;    // tid bits [3:2]: quad in the block
            assign tid_new_encoded[TID_BLOCK_DIGIT_LO + digit_bit] = |block_terms;   // tid bits [5:4]: block
        end
    endgenerate

    // reads and writes take turns when both wait; one request per cycle at most, only into an empty request register
    assign pick_write = write_any & (~read_any | ~last_was_write);
    // L2-5: = ~req_reg_valid & tid_any_free & (read_any | write_any), with the kept 3-level any-free (take_if_requested)
    assign req_take   = take_if_requested & (read_any | write_any);
    assign read_take  = req_take & ~pick_write;
    assign write_take = req_take &  pick_write;

    // TIMING OPT L2-2: the same values from early terms, the late grant last. A grant bit implies that its class has a
    // request (read_any / write_any = 1), so: read_take & grant = grant & take_if_requested & read_turn, and the same for
    // writes; and a one-hot free tid implies tid_any_free, so req_take & one-hot = take_if_tid_free & one-hot.
    assign take_if_requested = ~req_reg_valid & (|tid_block_free);         // flops only; |tid_block_free = tid_any_free (L2-9: 2 levels from the flag flops)
    assign take_if_tid_free  = ~req_reg_valid & (read_any | write_any);    // no tid term
    assign read_turn         = ~write_any | last_was_write;                // = ~pick_write when read_any = 1
    assign write_turn        = ~read_any | ~last_was_write;                // = pick_write when write_any = 1

    assign bank_read_req_ready  = read_grant_onehot  & {NB{take_if_requested & read_turn}};    // = read_take ? read grant : 0
    assign bank_write_req_ready = write_grant_onehot & {NB{take_if_requested & write_turn}};   // = write_take ? write grant : 0

    always @(posedge clk) begin : turn_proc
        if (rst)
            last_was_write <= 1'b0;
        else if (req_take)
            last_was_write <= pick_write;
    end

    // ======================================================================
    // Request: fields and packet
    // ======================================================================
    // TIMING OPT L2-1: candidate fields per bank, built from the bank's own inputs (no grant in this cone)
    generate
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin : g_cand
            wire [OFFSET_W-1:0]  read_offset;     // OFFSET_W = 6 bits: byte offset of this bank's read
            wire [NC_SIZE_W-1:0] read_size;       // NC_SIZE_W = 3 bits: NC read size code of this bank
            wire [BEAT_W-1:0]    write_beat;      // BEAT_W = 2 bits: beat of this bank's write address
            assign read_offset = bank_read_req_addr[bank_idx*AXI_ADDR_W +: OFFSET_W];       // low address bits
            assign read_size   = bank_read_req_size[bank_idx*NC_SIZE_W +: NC_SIZE_W];       // size code
            assign write_beat  = (AXI_BEATS > 1) ? bank_write_req_addr[bank_idx*AXI_ADDR_W + AXI_OFF_W +: BEAT_W]
                                                 : {BEAT_W{1'b0}};                          // same rule as sel_beat

            // NC read mask, bit i = (i >= offset) and (i < offset + 2**size), without an adder: the second test is
            // offset >= i + 1 - 2**k for each size code k (a compare with a constant, always true when i + 1 <= 2**k)
            for (mask_idx = 0; mask_idx < MASK_W; mask_idx = mask_idx + 1) begin : g_read_mask
                wire [NC_SIZE_CODES-1:0] ends_after;   // NC_SIZE_CODES = 8 bits: byte i is before the end, per size code
                for (size_idx = 0; size_idx < NC_SIZE_CODES; size_idx = size_idx + 1) begin : g_size
                    if (mask_idx + 1 <= (1 << size_idx)) begin : g_always
                        assign ends_after[size_idx] = 1'b1;                                   // offset + 2**k > i for every offset
                    end else begin : g_compare
                        assign ends_after[size_idx] = (read_offset >= mask_idx + 1 - (1 << size_idx));   // constant compare
                    end
                end
                assign read_nc_mask_all[bank_idx*MASK_W + mask_idx] =
                    (read_offset <= mask_idx) & ends_after[read_size];                        // from the offset, up to the end
            end

            // NC write: the beat (low AXI_DATA_W bits) and its strobes go to the address's beat, zero elsewhere
            for (beat_idx = 0; beat_idx < AXI_BEATS; beat_idx = beat_idx + 1) begin : g_write_beat
                assign write_nc_line_all[bank_idx*LINE_W + beat_idx*AXI_DATA_W +: AXI_DATA_W] =      // AXI_DATA_W = 128 bits
                    (write_beat == beat_idx) ? bank_write_req_data[bank_idx*LINE_W +: AXI_DATA_W] : {AXI_DATA_W{1'b0}};
                assign write_nc_mask_all[bank_idx*MASK_W + beat_idx*AXI_STRB_W +: AXI_STRB_W] =      // AXI_STRB_W = 16 bits
                    (write_beat == beat_idx) ? bank_write_req_strobe[bank_idx*AXI_STRB_W +: AXI_STRB_W] : {AXI_STRB_W{1'b0}};
            end

            // read candidate (candidate bank_idx): line read mask all ones and data 0; us only for a line read
            assign cand_fields[bank_idx*CAND_W + CAND_ADDR_LO +: AXI_ADDR_W] =                    // 64 bits: address
                bank_read_req_addr[bank_idx*AXI_ADDR_W +: AXI_ADDR_W];
            assign cand_fields[bank_idx*CAND_W + CAND_SUB_LO +: AXI_SUBID_W] =                   // 2 bits: sub id
                bank_read_req_sub_id[bank_idx*AXI_SUBID_W +: AXI_SUBID_W];
            assign cand_fields[bank_idx*CAND_W + CAND_NC_BIT] = bank_read_req_non_cacheable[bank_idx];   // nc
            assign cand_fields[bank_idx*CAND_W + CAND_UNIQUE_BIT] =                              // us: line read wanted unique
                ~bank_read_req_non_cacheable[bank_idx] & bank_read_req_unique[bank_idx];
            assign cand_fields[bank_idx*CAND_W + CAND_MASK_LO +: MASK_W] =                       // 64 bits: NC bytes or all
                bank_read_req_non_cacheable[bank_idx] ? read_nc_mask_all[bank_idx*MASK_W +: MASK_W] : {MASK_W{1'b1}};
            assign cand_fields[bank_idx*CAND_W + CAND_DATA_LO +: LINE_W] = {LINE_W{1'b0}};       // 512 bits: a read has no data

            // write candidate (candidate NB + bank_idx): never us; NC: placed beat and strobes, line: the line, mask all
            assign cand_fields[(NB+bank_idx)*CAND_W + CAND_ADDR_LO +: AXI_ADDR_W] =               // 64 bits: address
                bank_write_req_addr[bank_idx*AXI_ADDR_W +: AXI_ADDR_W];
            assign cand_fields[(NB+bank_idx)*CAND_W + CAND_SUB_LO +: AXI_SUBID_W] =              // 2 bits: sub id
                bank_write_req_sub_id[bank_idx*AXI_SUBID_W +: AXI_SUBID_W];
            assign cand_fields[(NB+bank_idx)*CAND_W + CAND_NC_BIT] = bank_write_req_non_cacheable[bank_idx];   // nc
            assign cand_fields[(NB+bank_idx)*CAND_W + CAND_UNIQUE_BIT] = 1'b0;                   // a write never sets us
            assign cand_fields[(NB+bank_idx)*CAND_W + CAND_MASK_LO +: MASK_W] =                  // 64 bits: NC strobes or all
                bank_write_req_non_cacheable[bank_idx] ? write_nc_mask_all[bank_idx*MASK_W +: MASK_W] : {MASK_W{1'b1}};
            assign cand_fields[(NB+bank_idx)*CAND_W + CAND_DATA_LO +: LINE_W] =                  // 512 bits: NC beat or line
                bank_write_req_non_cacheable[bank_idx] ? write_nc_line_all[bank_idx*LINE_W +: LINE_W]
                                                       : bank_write_req_data[bank_idx*LINE_W +: LINE_W];

            // picks: the one-hot grant, gated by the class; bank 0 also stands for "no request" (the binary grant is 0
            // then), so exactly one candidate is picked in every cycle, as the old binary muxes did
            // TIMING OPT L2-2: same values with the turn terms (a grant bit implies its class has a request):
            //   ~pick_write & read grant = read grant & read_turn;  pick_write & write grant = write grant & write_turn;
            //   ~pick_write & ~read_any = ~read_any & ~write_any;   pick_write & ~write_any = 0
            if (bank_idx == 0) begin : g_pick_first
                assign read_pick_onehot[bank_idx]  = (read_grant_onehot[bank_idx] & read_turn) | (~read_any & ~write_any);
                assign write_pick_onehot[bank_idx] =  write_grant_onehot[bank_idx] & write_turn;
            end else begin : g_pick_rest
                assign read_pick_onehot[bank_idx]  = read_grant_onehot[bank_idx]  & read_turn;     // read of this bank
                assign write_pick_onehot[bank_idx] = write_grant_onehot[bank_idx] & write_turn;    // write of this bank
            end
        end
    endgenerate

    assign cand_pick = {write_pick_onehot, read_pick_onehot};   // candidate order: reads 0..NB-1, writes NB..2*NB-1 (L2-3: reference only)

    // one AND-OR per field bit: the picked candidate's value (bit-major copy first, so each bit is one reduction)
    // TIMING OPT L2-3: the raw one-hot grants (straight from the round robins) select within each class, and the class
    // turn is applied after that select, so the turn / no-request terms are no longer in front of the grant fan-out:
    //   sel_cand = read_turn & (read grant AND-OR) | write_turn & (write grant AND-OR) | no_request & (bank 0 read).
    // Same value as the cand_pick AND-OR in every cycle: a grant bit implies its class has a request, read_turn and
    // write_turn are both 1 only when at most one class has a request (then the other class's AND-OR is 0), and with no
    // request both grant AND-ORs are 0 and bank 0's read candidate is chosen, as before.
    assign no_request = ~read_any & ~write_any;                           // no bank offers a read or a write
    generate
        for (field_bit = 0; field_bit < CAND_W; field_bit = field_bit + 1) begin : g_pick_bit
            for (cand_idx = 0; cand_idx < CANDIDATES; cand_idx = cand_idx + 1) begin : g_column
                assign cand_columns[field_bit*CANDIDATES + cand_idx] = cand_fields[cand_idx*CAND_W + field_bit];   // regroup
            end
            assign read_grant_value[field_bit]  = |(read_grant_onehot  & cand_columns[field_bit*CANDIDATES      +: NB]);   // NB = 4 read candidates
            assign write_grant_value[field_bit] = |(write_grant_onehot & cand_columns[field_bit*CANDIDATES + NB +: NB]);   // 4 write candidates
            assign sel_cand[field_bit] = (read_turn  & read_grant_value[field_bit])           // granted read, if reads have the turn
                                       | (write_turn & write_grant_value[field_bit])          // granted write, if writes have the turn
                                       | (no_request & cand_columns[field_bit*CANDIDATES]);   // no request: bank 0's read
        end
    endgenerate

    assign sel_bank         = pick_write ? write_grant_bank : read_grant_bank;
    assign sel_addr         = sel_cand[CAND_ADDR_LO +: AXI_ADDR_W];      // = old 4:1 mux of the class's granted bank
    assign sel_nc           = sel_cand[CAND_NC_BIT];                      // = old mux of the nc bits
    assign sel_unique       = sel_cand[CAND_UNIQUE_BIT];                  // = ~pick_write & ~sel_nc & unique of the read bank
    assign sel_sub          = sel_cand[CAND_SUB_LO +: AXI_SUBID_W];      // = old mux of the sub ids
    assign sel_beat         = (AXI_BEATS > 1) ? sel_addr[AXI_OFF_W +: BEAT_W] : {BEAT_W{1'b0}};   // route beat
    // TIMING OPT L2-1: reference forms below (binary muxes, old NC mask and shifts): same values as before, but no longer
    // in the packet cone (sel_mask / sel_data come from sel_cand), so synthesis removes them; kept for the names
    assign sel_read_size    = bank_read_req_size[read_grant_bank*NC_SIZE_W +: NC_SIZE_W];
    assign sel_write_line   = bank_write_req_data[write_grant_bank*LINE_W +: LINE_W];
    assign sel_write_strobe = bank_write_req_strobe[write_grant_bank*AXI_STRB_W +: AXI_STRB_W];
    assign sel_offset       = sel_addr[OFFSET_W-1:0];

    // NC write: the bank gives the beat in the low AXI_DATA_W bits and its strobes (already on the byte lanes of the
    // address); put both at the address's beat of the line
    assign nc_write_line = {{(LINE_W - AXI_DATA_W){1'b0}}, sel_write_line[AXI_DATA_W-1:0]} << (sel_beat * AXI_DATA_W);
    assign nc_write_mask = {{(MASK_W - AXI_STRB_W){1'b0}}, sel_write_strobe} << (sel_beat * AXI_STRB_W);

    // NC read: the 1 / 2 / 4 / 8 bytes from the offset
    always @* begin : nc_read_mask_proc
        nc_read_bytes = 1 << sel_read_size;
        for (mask_bit = 0; mask_bit < MASK_W; mask_bit = mask_bit + 1)
            nc_read_mask[mask_bit] = (mask_bit >= sel_offset) && (mask_bit < sel_offset + nc_read_bytes);
    end

    // TIMING OPT L2-1: mask and data come from the picked candidate; same values as
    //   sel_mask = ~sel_nc ? all ones : (pick_write ? nc_write_mask : nc_read_mask)
    //   sel_data = ~pick_write ? 0 : (sel_nc ? nc_write_line : sel_write_line)
    assign sel_mask  = sel_cand[CAND_MASK_LO +: MASK_W];                  // 64 bits: packet mask
    assign sel_data  = sel_cand[CAND_DATA_LO +: LINE_W];                  // 512 bits: packet data
    assign sel_pkt   = {sel_unique, sel_addr[L2_ADDR_W-1:0], pick_write, MASTER_ID_C, tid_new_encoded, sel_nc, sel_mask, sel_data};
    assign sel_route = {pick_write, sel_nc, sel_bank, sel_sub, sel_beat};

    // request register: holds the packet until the L2 takes it
    always @(posedge clk) begin : req_reg_valid_proc
        if (rst)
            req_reg_valid <= 1'b0;
        else if (req_take)
            req_reg_valid <= 1'b1;
        else if (req_reg_valid && l2_req_ready)
            req_reg_valid <= 1'b0;
    end

    always @(posedge clk) begin : req_reg_data_proc     // no reset: read only while req_reg_valid = 1
        if (req_take)
            req_reg_pkt <= sel_pkt;
    end

    // route table (no reset: written at allocation, read only while the tid is busy)
    // TIMING OPT L2-1: written through the one-hot free tid (req_take implies a free tid, so it is the decode of tid_new)
    // TIMING OPT L2-2: take_if_tid_free instead of req_take (a one-hot bit implies tid_any_free); the one-hot is last
    always @(posedge clk) begin : route_write_proc
        for (tid_slot = 0; tid_slot < NTID; tid_slot = tid_slot + 1)
            if (tid_take_onehot[tid_slot])                                  // L2-3: one shared take one-hot
                route_table[tid_slot] <= sel_route;
    end

    // tid busy bits: set at allocation, cleared when the answer is taken (the clear wins, as before)
    // TIMING OPT L2-2: the clear is the one-hot decode of the answer tid, gated by resp_take
    always @(posedge clk) begin : tid_busy_proc
        if (rst)
            tid_busy <= {NTID{1'b0}};
        else
            tid_busy <= (tid_busy | tid_take_onehot)                              // set the new tid (zero without a take)
                      & ~({NTID{resp_take}} & resp_tid_onehot);                   // clear the answered tid
    end

    // TIMING OPT L2-9: quad flags, kept equal to the OR of each quad's free bits (reset: all tids free, all flags 1)
    always @(posedge clk) begin : tid_quad_free_proc
        if (rst)
            tid_quad_free_q <= {TID_QUADS{1'b1}};                                // TID_QUADS = 16 flags: every quad free
        else
            tid_quad_free_q <= tid_quad_free_next;                               // set / clear of this cycle applied
    end

    assign l2_req_valid = req_reg_valid;
    assign l2_req_pkt   = req_reg_pkt;

    // ======================================================================
    // Response: route lookup, then hand the answer to the bank
    // ======================================================================
    assign resp_tid       = l2_resp_pkt[TID_LO +: L2_TID_W];

    // TIMING OPT L2-2: route table read = one-hot decode of the answer tid and one flat AND-OR per bit (was a 64:1 mux);
    // the route-only answer values (first beat, last beat, line read) are formed per entry before the AND-OR, so they
    // come out of it at the same time as the route bits
    generate
        for (tid_idx = 0; tid_idx < NTID; tid_idx = tid_idx + 1) begin : g_route_entry
            assign resp_tid_onehot[tid_idx] = (resp_tid == tid_idx);                       // decode of the answer tid
            for (route_bit = 0; route_bit < ROUTE_W; route_bit = route_bit + 1) begin : g_route_column
                assign route_columns[route_bit*NTID + tid_idx] = route_table[tid_idx][route_bit];   // regroup bit-major
            end
            for (beat_bit = 0; beat_bit < BEAT_W; beat_bit = beat_bit + 1) begin : g_beat_column
                assign route_start_columns[beat_bit*NTID + tid_idx] =                     // NC: its own beat, line: beat 0
                    route_table[tid_idx][ROUTE_W-2] & route_table[tid_idx][beat_bit];
                assign route_end_columns[beat_bit*NTID + tid_idx] =                       // NC: its own beat, line: LAST_BEAT
                    route_table[tid_idx][ROUTE_W-2] ? route_table[tid_idx][beat_bit] : LAST_BEAT[beat_bit];
            end
            assign route_line_read_column[tid_idx] =                                      // not a write and not NC
                ~route_table[tid_idx][ROUTE_W-1] & ~route_table[tid_idx][ROUTE_W-2];
        end
        for (route_bit = 0; route_bit < ROUTE_W; route_bit = route_bit + 1) begin : g_route_read
            assign resp_route[route_bit] = |(resp_tid_onehot & route_columns[route_bit*NTID +: NTID]);   // = route_table[resp_tid]
        end
        for (beat_bit = 0; beat_bit < BEAT_W; beat_bit = beat_bit + 1) begin : g_beat_read
            assign resp_start_beat[beat_bit] = |(resp_tid_onehot & route_start_columns[beat_bit*NTID +: NTID]);   // AND-OR
            assign resp_end_beat[beat_bit]   = |(resp_tid_onehot & route_end_columns[beat_bit*NTID +: NTID]);     // AND-OR
        end
    endgenerate
    assign resp_line_read = |(resp_tid_onehot & route_line_read_column);   // AND-OR

    assign ans_last      = ans_valid & (ans_is_write | (ans_beat == ans_end_beat));
    assign resp_ready_int = ~ans_valid | ans_last;          // the next answer is taken in the cycle the current one ends
    assign resp_take      = l2_resp_valid & resp_ready_int;

    always @(posedge clk) begin : ans_ctrl_proc
        if (rst) begin
            ans_valid   <= 1'b0;
            err_pulse_q <= 1'b0;
        end else begin
            err_pulse_q <= resp_take & l2_resp_err;
            if (resp_take)
                ans_valid <= 1'b1;
            else if (ans_last)
                ans_valid <= 1'b0;
        end
    end

    always @(posedge clk) begin : ans_data_proc      // no reset: read only while ans_valid = 1
        if (resp_take) begin
            ans_is_write <= resp_route[ROUTE_W-1];
            ans_nc       <= resp_route[ROUTE_W-2];
            ans_shared   <= resp_line_read & l2_resp_pkt[US_BIT];          // = ~is_write & ~nc & us (L2-2: per entry)
            ans_bank     <= resp_route[AXI_SUBID_W + BEAT_W +: BANK_W];
            ans_sub      <= resp_route[BEAT_W +: AXI_SUBID_W];
            ans_err      <= l2_resp_err;
            ans_line     <= l2_resp_pkt[LINE_W-1:0];
            ans_beat     <= resp_start_beat;     // = nc ? route beat : 0, NC: only its own beat (L2-2: per entry)
            ans_end_beat <= resp_end_beat;       // = nc ? route beat : LAST_BEAT (L2-2: per entry)
        end else if (ans_valid && !ans_last) begin
            ans_beat <= ans_beat + 1'b1;
        end
    end

    // steer the answer to its bank; the line hand-over holds that bank's snoops (header)
    generate
        for (bank_idx = 0; bank_idx < NB; bank_idx = bank_idx + 1) begin : g_steer
            assign bank_read_data_valid[bank_idx]    = ans_valid & ~ans_is_write & (ans_bank == bank_idx);
            assign bank_write_resp_valid[bank_idx]   = ans_valid &  ans_is_write & (ans_bank == bank_idx);
            assign bank_refill_replay_busy[bank_idx] = ans_valid & ~ans_is_write & ~ans_nc & (ans_bank == bank_idx);
        end
    endgenerate

    assign bank_read_data_sub_id         = ans_sub;
    assign bank_read_data_non_cacheable  = ans_nc;
    assign bank_read_data                = ans_line[ans_beat*AXI_DATA_W +: AXI_DATA_W];
    assign bank_read_data_last           = (ans_beat == ans_end_beat);
    assign bank_read_data_error          = ans_err;
    assign bank_read_data_shared         = ans_shared;
    assign bank_write_resp_sub_id        = ans_sub;
    assign bank_write_resp_non_cacheable = ans_nc;
    assign bank_write_resp_error         = ans_err;
    assign l2_resp_ready                 = resp_ready_int;
    assign l2_error_pulse                = err_pulse_q;

`ifdef L1D_DEBUG
    // ======================================================================
    // Debug trace (simulation only): requests, answers, checks of the L2 side, watchdogs
    // ======================================================================
    integer dbg_req_wait, dbg_ans_wait, dbg_k, dbg_busy_count;
    initial begin
        dbg_req_wait = 0;
        dbg_ans_wait = 0;
    end
    always @(posedge clk) begin : dbg_proc
        if (rst) begin
            dbg_req_wait = 0;
            dbg_ans_wait = 0;
        end else begin
            if (req_take) begin
                $display("[%0t] L1D L2P  REQ %s %s bank %0d sub %0d tid %0d us %b addr %h", $time, pick_write ? "WR" : "RD",
                         sel_nc ? "NC  " : "line", sel_bank, sel_sub, tid_new, sel_unique, sel_addr);
                if ((APAD_W > 0) && (sel_addr[AXI_ADDR_W-1:L2_ADDR_W] != {APAD_W{1'b0}}))
                    $display("[%0t] L1D L2P  ERROR address %h has bits above the %0d-bit packet address", $time, sel_addr, L2_ADDR_W);
                if (!pick_write && (bank_read_req_burst_len[read_grant_bank*8 +: 8] != (sel_nc ? 8'd0 : LINE_LEN)))
                    $display("[%0t] L1D L2P  ERROR bank %0d read len %0d", $time, read_grant_bank, bank_read_req_burst_len[read_grant_bank*8 +: 8]);
                if (pick_write && (bank_write_req_burst_len[write_grant_bank*8 +: 8] != (sel_nc ? 8'd0 : LINE_LEN)))
                    $display("[%0t] L1D L2P  ERROR bank %0d write len %0d", $time, write_grant_bank, bank_write_req_burst_len[write_grant_bank*8 +: 8]);
            end
            if (resp_take) begin
                $display("[%0t] L1D L2P  RESP tid %0d %s bank %0d sub %0d us %b err %b", $time, resp_tid,
                         resp_route[ROUTE_W-1] ? "WR" : "RD", resp_route[AXI_SUBID_W + BEAT_W +: BANK_W],
                         resp_route[BEAT_W +: AXI_SUBID_W], l2_resp_pkt[US_BIT], l2_resp_err);
                if (!tid_busy[resp_tid])
                    $display("[%0t] L1D L2P  ERROR answer with tid %0d, which is not in use", $time, resp_tid);
                if (l2_resp_pkt[MID_LO +: L2_MID_W] != MASTER_ID_C)
                    $display("[%0t] L1D L2P  ERROR answer with master id %0d, expected %0d", $time, l2_resp_pkt[MID_LO +: L2_MID_W], MASTER_ID_C);
                if (l2_resp_pkt[RW_LO] != resp_route[ROUTE_W-1])
                    $display("[%0t] L1D L2P  ERROR answer rw %b for tid %0d does not match the request", $time, l2_resp_pkt[RW_LO], resp_tid);
            end
            if (req_reg_valid && !l2_req_ready) begin
                dbg_req_wait = dbg_req_wait + 1;
                if (dbg_req_wait % `L1D_DBG_WD == 0)
                    $display("[%0t] L1D L2P  STUCK request not taken by the L2 for %0d cycles", $time, dbg_req_wait);
            end else
                dbg_req_wait = 0;
            dbg_busy_count = 0;
            for (dbg_k = 0; dbg_k < NTID; dbg_k = dbg_k + 1) dbg_busy_count = dbg_busy_count + tid_busy[dbg_k];
            if ((dbg_busy_count > 0) && !l2_resp_valid) begin
                dbg_ans_wait = dbg_ans_wait + 1;
                if (dbg_ans_wait % `L1D_DBG_WD == 0)
                    $display("[%0t] L1D L2P  STUCK %0d tid(s) in use, no answer for %0d cycles", $time, dbg_busy_count, dbg_ans_wait);
            end else
                dbg_ans_wait = 0;
        end
    end
`endif

endmodule

`default_nettype wire

`endif // L1D_L2_PORT_V
