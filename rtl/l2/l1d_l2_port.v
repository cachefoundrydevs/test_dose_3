// ============================================================================
// l1d_l2_port.v  -  the cache's one connection to the L2 (packet port, cache_v5)
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

    // ======================================================================
    // Request: round robin per class, read / write turn, free tid
    // ======================================================================
    l1d_arb_rr #(.N(NB), .IDX_W(BANK_W)) u_read_rr (     // which bank's read goes next
        .clk          (clk),
        .rst          (rst),
        .request_bits (bank_read_req_valid),
        .grant_taken  (read_take),
        .grant_onehot (read_grant_onehot),
        .grant_index  (read_grant_bank),
        .any_request  (read_any)
    );

    l1d_arb_rr #(.N(NB), .IDX_W(BANK_W)) u_write_rr (    // which bank's write goes next
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

    // reads and writes take turns when both wait; one request per cycle at most, only into an empty request register
    assign pick_write = write_any & (~read_any | ~last_was_write);
    assign req_take   = ~req_reg_valid & tid_any_free & (read_any | write_any);
    assign read_take  = req_take & ~pick_write;
    assign write_take = req_take &  pick_write;

    assign bank_read_req_ready  = read_take  ? read_grant_onehot  : {NB{1'b0}};
    assign bank_write_req_ready = write_take ? write_grant_onehot : {NB{1'b0}};

    always @(posedge clk) begin : turn_proc
        if (rst)
            last_was_write <= 1'b0;
        else if (req_take)
            last_was_write <= pick_write;
    end

    // ======================================================================
    // Request: fields and packet
    // ======================================================================
    assign sel_bank         = pick_write ? write_grant_bank : read_grant_bank;
    assign sel_addr         = pick_write ? bank_write_req_addr[write_grant_bank*AXI_ADDR_W +: AXI_ADDR_W]
                                         : bank_read_req_addr[read_grant_bank*AXI_ADDR_W +: AXI_ADDR_W];
    assign sel_nc           = pick_write ? bank_write_req_non_cacheable[write_grant_bank] : bank_read_req_non_cacheable[read_grant_bank];
    assign sel_unique       = ~pick_write & ~sel_nc & bank_read_req_unique[read_grant_bank];
    assign sel_sub          = pick_write ? bank_write_req_sub_id[write_grant_bank*AXI_SUBID_W +: AXI_SUBID_W]
                                         : bank_read_req_sub_id[read_grant_bank*AXI_SUBID_W +: AXI_SUBID_W];
    assign sel_read_size    = bank_read_req_size[read_grant_bank*3 +: 3];
    assign sel_write_line   = bank_write_req_data[write_grant_bank*LINE_W +: LINE_W];
    assign sel_write_strobe = bank_write_req_strobe[write_grant_bank*AXI_STRB_W +: AXI_STRB_W];
    assign sel_offset       = sel_addr[OFFSET_W-1:0];
    assign sel_beat         = (AXI_BEATS > 1) ? sel_addr[AXI_OFF_W +: BEAT_W] : {BEAT_W{1'b0}};

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

    assign sel_mask  = ~sel_nc ? {MASK_W{1'b1}} : (pick_write ? nc_write_mask : nc_read_mask);
    assign sel_data  = ~pick_write ? {LINE_W{1'b0}} : (sel_nc ? nc_write_line : sel_write_line);
    assign sel_pkt   = {sel_unique, sel_addr[L2_ADDR_W-1:0], pick_write, MASTER_ID_C, tid_new, sel_nc, sel_mask, sel_data};
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
    always @(posedge clk) begin : route_write_proc
        if (req_take)
            route_table[tid_new] <= sel_route;
    end

    // tid busy bits: set at allocation, cleared when the answer is taken
    always @(posedge clk) begin : tid_busy_proc
        if (rst)
            tid_busy <= {NTID{1'b0}};
        else begin
            if (req_take)  tid_busy[tid_new]  <= 1'b1;
            if (resp_take) tid_busy[resp_tid] <= 1'b0;
        end
    end

    assign l2_req_valid = req_reg_valid;
    assign l2_req_pkt   = req_reg_pkt;

    // ======================================================================
    // Response: route lookup, then hand the answer to the bank
    // ======================================================================
    assign resp_tid       = l2_resp_pkt[TID_LO +: L2_TID_W];
    assign resp_route     = route_table[resp_tid];
    assign ans_last       = ans_valid & (ans_is_write | (ans_beat == ans_end_beat));
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
            ans_shared   <= ~resp_route[ROUTE_W-1] & ~resp_route[ROUTE_W-2] & l2_resp_pkt[US_BIT];
            ans_bank     <= resp_route[AXI_SUBID_W + BEAT_W +: BANK_W];
            ans_sub      <= resp_route[BEAT_W +: AXI_SUBID_W];
            ans_err      <= l2_resp_err;
            ans_line     <= l2_resp_pkt[LINE_W-1:0];
            ans_beat     <= resp_route[ROUTE_W-2] ? resp_route[BEAT_W-1:0] : {BEAT_W{1'b0}};   // NC: only its own beat
            ans_end_beat <= resp_route[ROUTE_W-2] ? resp_route[BEAT_W-1:0] : LAST_BEAT;
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
