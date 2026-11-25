// ============================================================================
// l1d_top.v  -  L1 DATA CACHE cache_v6 (tracker S0, ECC, prefetch): the top level
// ============================================================================
// IN SIMPLE WORDS
//   This is the whole cache. The CPU side has FOUR request ports and four response ports, one per request port. The pin
//   names follow the tested single-port cache_v1_4 (valid_cpu, PC_from_cpu, stop, read_enable_from_lsu,
//   final_r_*_from_scheduler ...), with the port number at the end:
//      _0  port 0: loads and LR (load-reserved)
//      _1  port 1: loads and LR
//      _2  port 2: stores, SC (store-conditional) and AMO (atomic read-modify-write)
//      _3  port 3: VPU, whole-line (64-byte) vector load and vector store
//   On the other side one L2 packet port (requests and answers, no AXI); a third port carries "snoops" from other cores.
//   4 banks inside. Any port can reach any bank.
//
//      port 0..3 pins --> l1d_port_front (bank decode, port skids, one l1d_bank_order per bank) --> l1d_bank x NUM_BANKS (4)
//      l1d_bank x 4 --(answer carries its port)--> l1d_resp_merge u_merge_0 .. u_merge_3 --> final_r_*_from_scheduler_0 .. _3
//      l1d_bank x 4 <--> l1d_l2_port <--> L2 packet port (l2_req_* / l2_resp_*, unique / shared bit us)
//      snoop port (snoop_request_*) --> l1d_snoop_ctrl --> the snoop buffer of the addressed bank
//      l1d_bank x 4 --> l1d_snoop_ctrl --> snoop answers (snoop_response_*)
//      l1d_bank x 4 --(training records, evictions, feedback)--> l1d_prefetch_unit --(one hint per bank)--> l1d_bank x 4
//      flush / clean --> l1d_maint in every bank --> flush_complete / clean_complete
//
// NEW IN cache_v6 (PLAN_v6.md)
//   - every bank has a REQUEST TRACKER (8 entries) that takes up to 4 arrivals per cycle: the port front sends a port's
//     request straight to its bank (bank_arrival_request_N) and keeps a skid only when the bank has no room;
//   - pc_cpu_0 / _1 / _2: the instruction PC of the request on ports 0..2 (the stride and SMS prefetchers train on it; the
//     VPU port has no PC, open item O-8); pf_enable turns the prefetchers on;
//   - SECDED ECC on the data and tag arrays (HAS_ECC): ecc_correctable_pulse = a scrub was written (one cycle),
//     ecc_uncorrectable_sticky = an uncorrectable error was seen (cleared only by reset);
//   - the three prefetchers (HAS_PF) in l1d_prefetch_unit, fed by the banks' training records.
//
// PORT RULES
//   A request is taken in EVERY cycle valid_cpu_N = 1 (exactly one cycle per request). stop_N is a registered stop: when it
//   is 1, port N must not send. It is 1 while port N's skid (2 places, l1d_port_front) holds a request, so the one request
//   sent while the LSU still sees 0 (BUG-7) still fits. The answer comes back on the SAME port number with its idx (ROB
//   index) so the CPU can match it; it is held until read_enable_from_lsu_N = 1. Answers can return OUT OF ORDER across
//   different lines; requests to the SAME line are answered in order. Load data: the requested bytes from bit 0, upper bits
//   zero, no sign extension.
//
// PORT 3: THE VPU PORT (cache_v5 C-20, parameter HAS_VPU, default 1)
//   A vector load answers the whole 64-byte line (final_r_data_from_scheduler_3, 512 bits); a vector store
//   (rd_wr_cpu_3 = 1) writes the whole line from data_from_cpu_3 (no byte mask) and answers a write acknowledge
//   (final_r_hit_rd_wr_from_scheduler_3 = 1). PC_from_cpu_3 must be 64-byte aligned. No atomics on port 3. With
//   HAS_VPU = 0 the port 3 pins are unused (stop_3 = 1, no answers).
//
// ATOMICS
//   LR = is_lr_cpu_0 / is_lr_cpu_1 on ports 0 / 1; SC = is_sc_cpu_2 and AMO = is_amo_cpu_2 (operation amo_op_cpu_2) on
//   port 2. An SC answers in the data word: 0 = success, 1 = fail. There is ONE LR/SC reservation for the whole cache: each
//   bank keeps its reservation unit, and an LR or SC in one bank clears the reservation in the others (other_rsv_event_q).
//   amo_err is set if an AMO lock ever timed out (its AMO write never came).
// SNOOPS
//   snoop_request_valid / _ready / _addr / _invalidate in; snoop_response_* out; one answer per snoop.
//   Contract for the L2: no response packet for a line is sent while a snoop on that line is unanswered.
// MESI
//   request bit us = 1 asks for the line UNIQUE (store, LR, SC, AMO). Response bit us = 1 on a plain read means another cache
//   keeps a copy, so the line is installed as S.
//
// THE BANKS
//   l1d_bank.v is ONE file, instantiated NUM_BANKS times by a generate loop; BANK_ID is the loop index. The bank for an
//   address is address[7:6] (the bits above the 6 line-offset bits). The per-BANK buses (bk_*) keep one field per bank.
//
// PIN NAMES: this version, cache_v5, meaning                                      (N = port number)
//   flush / clean                        flush_request / clean_request      write back all dirty lines (flush also invalidates)
//   valid_cpu_N                          ld0/ld1/st/vpu_req_valid           a request on port N this cycle
//   cacheable_N                          *_req_cacheable                    1 = cacheable, 0 = non-cacheable
//   rd_wr_cpu_3                          vpu_req_is_write                   port 3: 1 = vector store, 0 = vector load
//   mem_size_cpu_N (N = 0, 1, 2)         *_req_size                         0 = 8, 1 = 16, 2 = 32, 3 = 64 bits
//   master_id_from_cpu_N                 *_req_master_id                    0 = LSU, 1 = MMU
//   idx_from_cpu_N                       *_req_rob_index                    ROB index, returned with the answer
//   PC_from_cpu_N                        *_req_addr                         byte address (port 3: line address)
//   data_from_cpu_2 / data_from_cpu_3    st_req_data / vpu_req_wdata        store, SC, AMO operand (64 bits) / vector line (512)
//   is_lr_cpu_0 / is_lr_cpu_1            ld0_req_is_lr / ld1_req_is_lr      the load is an LR
//   is_sc_cpu_2 / is_amo_cpu_2           st_req_is_sc / st_req_is_amo       the store is an SC / an AMO
//   amo_op_cpu_2                         st_req_amo_op                      which AMO (AMO_SWAP .. AMO_MAXU)
//   stop_N                               *_req_stop                         port N must not send (registered)
//   read_enable_from_lsu_N               *_resp_read_enable                 the LSU takes the current answer of port N
//   final_r_cache_hit_from_scheduler_N   *_resp_valid                       an answer is shown on port N
//   final_r_hit_rd_wr_from_scheduler_N   st/vpu_resp_is_write_ack           (N = 2, 3) 1 = write acknowledge
//   final_r_mem_size_from_scheduler_N    *_resp_size                        (N = 0, 1, 2) copy of the request size
//   final_r_master_id_from_scheduler_N   *_resp_master_id                   copy of the request master id
//   final_r_idx_from_scheduler_N         *_resp_rob_index                   copy of the request idx
//   final_r_data_from_scheduler_N        *_resp_data                        load / LR data, SC result, AMO old value, 0 for a
//                                                                           store; port 3: the 512-bit line
//   mem_err                              l2_error_sticky                    sticky: an L2 answer came with an error
//   amo_err                              amo_lock_timeout_sticky            sticky: an AMO lock timed out
//   pc_cpu_N (N = 0, 1, 2)               (cache_v6)                         instruction PC of the request on port N (training)
//   pf_enable                            (cache_v6)                         1 = the prefetchers train and send hints
//   ecc_correctable_pulse                (cache_v6)                         one cycle per ECC scrub written in some bank
//   ecc_uncorrectable_sticky             (cache_v6)                         sticky: an uncorrectable ECC error was seen
//   snoop_* , l2_req_* , l2_resp_* , flush_complete , clean_complete       unchanged
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

module l1d_top #(
    // Address and organisation
    parameter ADDR_W           = `L1D_ADDR_W,
    parameter LINE_BYTES       = `L1D_LINE_BYTES,
    parameter NUM_BANKS        = `L1D_NUM_BANKS,
    parameter SETS             = `L1D_SETS,
    parameter WAYS             = `L1D_WAYS,
    // LSU port
    parameter WRITE_DATA_WIDTH = `L1D_WRITE_DATA_WIDTH,
    parameter MEM_SIZE_BITS    = `L1D_MEM_SIZE_BITS,
    parameter MASTER_ID_BITS   = `L1D_MASTER_ID_BITS,
    parameter IDX_BITS         = `L1D_IDX_BITS,
    parameter PORT_QUEUE_DEPTH = `L1D_PORT_QUEUE_DEPTH,   // default 2: skid places per request port (lean S0, l1d_port_front)
    // Per-bank resources
    parameter LSU_BUF_DEPTH    = `L1D_LSU_BUF_DEPTH,
    parameter TRK_ENTRIES      = `L1D_TRK_ENTRIES,      // reserved (v1: the request buffer seats are the tracker)
    parameter TRK_REFILL_RSV   = `L1D_TRK_REFILL_RSV,   // reserved
    parameter MSHR_ENTRIES     = `L1D_MSHR_ENTRIES,
    parameter MSHR_NC_ENTRIES  = `L1D_MSHR_NC_ENTRIES,
    parameter VC_SLOTS         = `L1D_VC_SLOTS,
    parameter RESP_FIFO_DEPTH  = `L1D_RESP_FIFO_DEPTH,
    parameter REFILL_SKIP      = `L1D_REFILL_SKIP,
    // FIFO stage, snoops and atomics (v2)
    parameter SNP_BUF_DEPTH    = `L1D_SNP_BUF_DEPTH,
    parameter SNP_RESP_DEPTH   = `L1D_SNP_RESP_DEPTH,
    parameter PF_FIFO_DEPTH    = `L1D_PF_FIFO_DEPTH,
    parameter SNP_STARVE       = `L1D_SNP_STARVE,
    parameter LOCK_TIMEOUT     = `L1D_LOCK_TIMEOUT,
    parameter RSV_CLR_ON_EVICT = `L1D_RSV_CLR_ON_EVICT,
    parameter HAS_SNOOP        = `L1D_HAS_SNOOP,
    parameter HAS_AMO          = `L1D_HAS_AMO,
    // Beat width and address width inside the cache (the banks' MSHRs take a line as AXI_DATA_W-bit beats; the names stay
    // from cache_v2_bank_pipe, there is no AXI bus in cache_v5)
    parameter AXI_DATA_W       = `L1D_AXI_DATA_W,   // default 128: beat width between the L2 port and the banks
    parameter AXI_ADDR_W       = `L1D_AXI_ADDR_W,   // default 64: address width of the bank requests
    // L2 packet port (cache_v5)
    parameter L2_ADDR_W        = 56,                // default 56: packet address bits (Rev K physical address)
    parameter L2_MID_W         = 2,                 // default 2: packet master id bits
    parameter L2_TID_W         = 6,                 // default 6: packet transaction id bits
    parameter L2_MASTER_ID     = 0,                 // default 0: master id this cache puts in every request
    parameter L2_PKT_W         = 1 + L2_ADDR_W + 1 + L2_MID_W + L2_TID_W + 1 + LINE_BYTES + LINE_BYTES*8,  // default 643 bits (derived, do not override)
    // Front end and feature switches (A2: LSU_PORTS request ports, default 4; the feature switches are kept for later versions)
    parameter LSU_PORTS        = `L1D_LSU_PORTS,          // default 4: request ports, each with its own response port
    parameter HAS_MISS         = `L1D_HAS_MISS,
    parameter HAS_VC           = `L1D_HAS_VC,
    parameter HAS_NC           = `L1D_HAS_NC,
    // cache_v5 C-20: 1 = request port 3 is the 512-bit VPU port (valid_cpu_3 .. final_r_data_from_scheduler_3, all the
    // _3 pins); 0 = no VPU (port 3 pins unused, stop_3 = 1)
    parameter HAS_VPU          = `L1D_HAS_VPU,            // default 1
    // cache_v6: tracker, prefetch and ECC settings (l1d_defines.vh)
    parameter PC_W             = `L1D_PC_W,               // default 64: instruction PC bits (pc_cpu_N pins, tracker entries)
    parameter TRK_ALLOC_MAX    = `L1D_TRK_ALLOC_MAX,      // default 4: arrivals a bank tracker allocates in one cycle
    parameter PF_MSHR_MAX      = `L1D_PF_MSHR_MAX,        // default 2: MSHR entries per bank a prefetch may hold
    parameter HAS_ECC          = `L1D_HAS_ECC,            // default 1: SECDED on the data and tag arrays
    parameter HAS_PF           = `L1D_HAS_PF              // default 1: the prefetch unit is built
) (
    input  wire                         clk,              // clock: all registers change on its rising edge
    input  wire                         rst,              // synchronous reset, active high (control state goes to its idle value)

    // Cache maintenance (cache_v1_4 names)
    input  wire                         flush,            // write back all dirty lines, then invalidate all
    input  wire                         clean,            // write back all dirty lines, keep them valid
    output reg                          flush_complete,   // one-cycle pulse: a flush has finished in every bank
    output reg                          clean_complete,   // one-cycle pulse: a clean has finished in every bank

    // ---- request port 0: load, LR ----
    input  wire                         valid_cpu_0,            // a load / LR this cycle
    input  wire                         cacheable_0,            // 1 = cacheable, 0 = non-cacheable
    input  wire [MEM_SIZE_BITS-1:0]     mem_size_cpu_0,         // 0 = 8, 1 = 16, 2 = 32, 3 = 64 bits; MEM_SIZE_BITS = 2 bits
    input  wire [MASTER_ID_BITS-1:0]    master_id_from_cpu_0,   // 0 = LSU, 1 = MMU; MASTER_ID_BITS = 2 bits
    input  wire [IDX_BITS-1:0]          idx_from_cpu_0,         // ROB index, returned with the answer; IDX_BITS = 3 bits
    input  wire [ADDR_W-1:0]            PC_from_cpu_0,          // byte address; ADDR_W = 64 bits
    input  wire                         is_lr_cpu_0,            // 1 = LR (load-reserved); at most one LR in flight on ports 0 + 1
    output wire                         stop_0,                 // do not send (registered)
    // ---- request port 1: load, LR (same pins as port 0) ----
    input  wire                         valid_cpu_1,            // a load / LR this cycle
    input  wire                         cacheable_1,            // 1 = cacheable
    input  wire [MEM_SIZE_BITS-1:0]     mem_size_cpu_1,         // access size, 2 bits
    input  wire [MASTER_ID_BITS-1:0]    master_id_from_cpu_1,   // master id, 2 bits
    input  wire [IDX_BITS-1:0]          idx_from_cpu_1,         // ROB index, 3 bits
    input  wire [ADDR_W-1:0]            PC_from_cpu_1,          // byte address, 64 bits
    input  wire                         is_lr_cpu_1,            // 1 = LR
    output wire                         stop_1,                 // do not send (registered)
    // ---- request port 2: store, SC, AMO ----
    input  wire                         valid_cpu_2,            // a store / SC / AMO this cycle
    input  wire                         cacheable_2,            // 1 = cacheable
    input  wire [MEM_SIZE_BITS-1:0]     mem_size_cpu_2,         // access size, 2 bits
    input  wire [MASTER_ID_BITS-1:0]    master_id_from_cpu_2,   // master id, 2 bits
    input  wire [IDX_BITS-1:0]          idx_from_cpu_2,         // ROB index, 3 bits
    input  wire [ADDR_W-1:0]            PC_from_cpu_2,          // byte address, 64 bits
    input  wire [WRITE_DATA_WIDTH-1:0]  data_from_cpu_2,        // store data, SC data, AMO operand (low bits); 64 bits
    input  wire                         is_sc_cpu_2,            // 1 = SC (sent only after its LR was answered)
    input  wire                         is_amo_cpu_2,           // 1 = AMO (one request, computed in the cache)
    input  wire [3:0]                   amo_op_cpu_2,           // which AMO (AMO_SWAP .. AMO_MAXU, l1d_encodings.vh)
    output wire                         stop_2,                 // do not send (registered)
    // ---- request port 3: VPU, whole line (used when HAS_VPU = 1) ----
    input  wire                         valid_cpu_3,            // a vector request this cycle
    input  wire                         rd_wr_cpu_3,            // 1 = vector store, 0 = vector load
    input  wire                         cacheable_3,            // must be 1 in this version (non-cacheable vector access: open item PO-6)
    input  wire [MASTER_ID_BITS-1:0]    master_id_from_cpu_3,   // master id, 2 bits
    input  wire [IDX_BITS-1:0]          idx_from_cpu_3,         // ROB index, 3 bits
    input  wire [ADDR_W-1:0]            PC_from_cpu_3,          // line address (bits [5:0] = 0), 64 bits
    input  wire [LINE_BYTES*8-1:0]      data_from_cpu_3,        // vector store data, byte k at bits [8k +: 8]; 512 bits
    output wire                         stop_3,                 // do not send (registered; always 1 when HAS_VPU = 0)
    // ---- cache_v6: the instruction PC of the request on ports 0..2 (prefetch training) and the prefetch switch ----
    input  wire [PC_W-1:0]              pc_cpu_0,               // PC of the port 0 request, PC_W = 64 bits (valid with valid_cpu_0)
    input  wire [PC_W-1:0]              pc_cpu_1,               // PC of the port 1 request, 64 bits
    input  wire [PC_W-1:0]              pc_cpu_2,               // PC of the port 2 request, 64 bits
    input  wire                         pf_enable,              // 1 = the prefetchers train and send hints

    // ---- response port 0 (an answer is held until read_enable_from_lsu_0 = 1) ----
    input  wire                         read_enable_from_lsu_0,               // the LSU takes the current port 0 answer
    output wire                         final_r_cache_hit_from_scheduler_0,   // a port 0 answer is shown
    output wire [MEM_SIZE_BITS-1:0]     final_r_mem_size_from_scheduler_0,    // copy of the request size, 2 bits
    output wire [MASTER_ID_BITS-1:0]    final_r_master_id_from_scheduler_0,   // copy of the request master id, 2 bits
    output wire [IDX_BITS-1:0]          final_r_idx_from_scheduler_0,         // copy of the request idx, 3 bits
    output wire [WRITE_DATA_WIDTH-1:0]  final_r_data_from_scheduler_0,        // load / LR data from bit 0, no sign extension, 64 bits
    // ---- response port 1 ----
    input  wire                         read_enable_from_lsu_1,               // the LSU takes the current port 1 answer
    output wire                         final_r_cache_hit_from_scheduler_1,   // a port 1 answer is shown
    output wire [MEM_SIZE_BITS-1:0]     final_r_mem_size_from_scheduler_1,    // 2 bits
    output wire [MASTER_ID_BITS-1:0]    final_r_master_id_from_scheduler_1,   // 2 bits
    output wire [IDX_BITS-1:0]          final_r_idx_from_scheduler_1,         // 3 bits
    output wire [WRITE_DATA_WIDTH-1:0]  final_r_data_from_scheduler_1,        // 64 bits
    // ---- response port 2 ----
    input  wire                         read_enable_from_lsu_2,               // the LSU takes the current port 2 answer
    output wire                         final_r_cache_hit_from_scheduler_2,   // a port 2 answer is shown
    output wire                         final_r_hit_rd_wr_from_scheduler_2,   // 1 = store / SC acknowledge, 0 = AMO old value
    output wire [MEM_SIZE_BITS-1:0]     final_r_mem_size_from_scheduler_2,    // 2 bits
    output wire [MASTER_ID_BITS-1:0]    final_r_master_id_from_scheduler_2,   // 2 bits
    output wire [IDX_BITS-1:0]          final_r_idx_from_scheduler_2,         // 3 bits
    output wire [WRITE_DATA_WIDTH-1:0]  final_r_data_from_scheduler_2,        // SC result (0 = success, 1 = fail), AMO old value, else 0
    // ---- response port 3 (VPU) ----
    input  wire                         read_enable_from_lsu_3,               // the VPU takes the current port 3 answer
    output wire                         final_r_cache_hit_from_scheduler_3,   // a port 3 answer is shown
    output wire                         final_r_hit_rd_wr_from_scheduler_3,   // 1 = vector store acknowledge, 0 = vector load data
    output wire [MASTER_ID_BITS-1:0]    final_r_master_id_from_scheduler_3,   // 2 bits
    output wire [IDX_BITS-1:0]          final_r_idx_from_scheduler_3,         // 3 bits
    output wire [LINE_BYTES*8-1:0]      final_r_data_from_scheduler_3,        // vector load data (the whole line), 512 bits

    // Snoop request (another core reads or writes a line)
    input  wire                         snoop_request_valid,  // a snoop request from outside
    output wire                         snoop_request_ready,  // the cache can take it
    input  wire [ADDR_W-1:0]            snoop_request_addr,  // any address inside the line
    input  wire                         snoop_request_invalidate,  // 1 = invalidate, 0 = read
    // Snoop answer (one per snoop; held until snoop_response_ready = 1)
    output wire                         snoop_response_valid,  // a snoop answer is shown
    input  wire                         snoop_response_ready,  // the receiver takes the answer
    output wire [ADDR_W-1:0]            snoop_response_addr,  // line address of the answered snoop
    output wire                         snoop_response_present,  // the cache held the line
    output wire                         snoop_response_dirty,  // snoop_response_data is a dirty line to write to memory
    output wire [LINE_BYTES*8-1:0]      snoop_response_data,  // the line
    // Status (cache_v1_4 names)
    output reg                          mem_err,          // sticky: an L2 answer came with l2_resp_err = 1
    output reg                          amo_err,          // sticky: an AMO lock timed out
    output reg                          ecc_correctable_pulse,    // cache_v6: one cycle per ECC scrub written (some bank)
    output reg                          ecc_uncorrectable_sticky, // cache_v6: sticky: an uncorrectable ECC error was seen

    // L2 packet port (packet layout in rtl/l2/l1d_l2_port.v, L2_PKT_W = 643 bits)
    output wire                         l2_req_valid,     // a request packet is offered to the L2
    input  wire                         l2_req_ready,     // the L2 takes it
    output wire [L2_PKT_W-1:0]          l2_req_pkt,       // request packet: us | addr 56 | rw | mid 2 | tid 6 | nc | mask 64 | data 512
    input  wire                         l2_resp_valid,    // a response packet is offered by the L2
    output wire                         l2_resp_ready,    // the cache takes it
    input  wire [L2_PKT_W-1:0]          l2_resp_pkt,      // response packet, same layout (us = 1: install the line shared)
    input  wire                         l2_resp_err       // the response is an error (sets mem_err)
);

    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    // NB = number of banks
    localparam NB = NUM_BANKS;

    // ======================================================================
    // REQUEST PACKETS: every port builds its own request packet from its pins:
    //   | line | port | amo_op | seq | amo | rd_wr | cacheable | size | master_id | idx | addr | data |   (field positions: l1d_fields.vh)
    // The port number is put into the packet, travels through the bank / MSHR / NC queue with the request and comes back in
    // the answer, where it picks the response port (u_merge_0 .. u_merge_3 below).
    //   ports 0 / 1: amo = is_lr (an LR), never a write, never an AMO;
    //   port 2: always a write (store, SC, AMO: rd_wr = 1), amo = is_sc | is_amo, seq = is_amo (the atomic is an AMO);
    //   port 3: line = 1 (whole line), rd_wr = rd_wr_cpu_3, never atomic, cacheable forced to 1 (PO-6), size field all ones,
    //           data field 0 (the bank puts the number of the vector data place there); its 512-bit line goes beside the packet.
    // amo / seq / amo_op are 0 when atomics are not built (HAS_AMO = 0).
    // Example: port 2 AMO_ADD on 0x1C0 -> req_packet_2 = {1'b0, 2'd2, AMO_ADD, 1, 1, 1, cacheable_2, size, mid, idx, 0x1C0, data}
    // cache_v6: the two top bits (REQ_PFU_BIT, REQ_PF_BIT) are 0 for every pins request (a prefetch op is made inside the bank).
    // ======================================================================
    wire [1:0]        no_pf_bits   = 2'b00;                                             // {REQ_PFU_BIT, REQ_PF_BIT} of a pins request
    wire              lr_0         = (HAS_AMO != 0) && is_lr_cpu_0;                    // port 0 request is an LR (built)
    wire              lr_1         = (HAS_AMO != 0) && is_lr_cpu_1;                    // port 1 request is an LR (built)
    wire              atomic_2     = (HAS_AMO != 0) && (is_sc_cpu_2 || is_amo_cpu_2);  // port 2 request is an SC or AMO (built)
    wire              amo_2        = atomic_2 && is_amo_cpu_2;                         // port 2 request is an AMO
    wire [AMOOP_W-1:0] amo_op_2    = amo_2 ? amo_op_cpu_2[AMOOP_W-1:0] : {AMOOP_W{1'b0}};   // its operation, AMOOP_W = 4 bits
    wire              req_valid_0  = valid_cpu_0;                                      // port 0 request this cycle
    wire              req_valid_1  = valid_cpu_1;                                      // port 1 request this cycle
    wire              req_valid_2  = valid_cpu_2;                                      // port 2 request this cycle
    wire              req_valid_3  = (HAS_VPU != 0) && valid_cpu_3;                    // port 3 request this cycle (only with a VPU)
    wire [REQ_W-1:0]  req_packet_0 = {no_pf_bits, 1'b0, 2'd0, {AMOOP_W{1'b0}}, 1'b0, lr_0, 1'b0, cacheable_0,   // port 0 packet, REQ_W = 148 bits
                                      mem_size_cpu_0, master_id_from_cpu_0, idx_from_cpu_0, PC_from_cpu_0,
                                      {WRITE_DATA_WIDTH{1'b0}}};
    wire [REQ_W-1:0]  req_packet_1 = {no_pf_bits, 1'b0, 2'd1, {AMOOP_W{1'b0}}, 1'b0, lr_1, 1'b0, cacheable_1,   // port 1 packet, 148 bits
                                      mem_size_cpu_1, master_id_from_cpu_1, idx_from_cpu_1, PC_from_cpu_1,
                                      {WRITE_DATA_WIDTH{1'b0}}};
    wire [REQ_W-1:0]  req_packet_2 = {no_pf_bits, 1'b0, 2'd2, amo_op_2, amo_2, atomic_2, 1'b1, cacheable_2,      // port 2 packet, 148 bits
                                      mem_size_cpu_2, master_id_from_cpu_2, idx_from_cpu_2, PC_from_cpu_2,
                                      data_from_cpu_2};
    wire [REQ_W-1:0]  req_packet_3 = {no_pf_bits, 1'b1, 2'd3, {AMOOP_W{1'b0}}, 1'b0, 1'b0, rd_wr_cpu_3, 1'b1,     // port 3 packet, 148 bits
                                      {MEM_SIZE_BITS{1'b1}}, master_id_from_cpu_3, idx_from_cpu_3, PC_from_cpu_3,
                                      {WRITE_DATA_WIDTH{1'b0}}};

    // ---- simulation checks: exactly 4 ports; the port 3 request rules (address aligned; non-cacheable vector not built) ----
`ifndef SYNTHESIS
    initial if (LSU_PORTS != 4)
        $display("L1D TOP  ERROR LSU_PORTS = %0d: the named ports _0 .. _3 need LSU_PORTS = 4", LSU_PORTS);
    always @(posedge clk) begin
        if (!rst && (HAS_VPU != 0) && valid_cpu_3) begin
            if (PC_from_cpu_3[OFFSET_W-1:0] != {OFFSET_W{1'b0}})
                $display("[%0t] L1D TOP  ERROR VPU request idx %0d address %h is not 64-byte aligned", $time, idx_from_cpu_3, PC_from_cpu_3);
            if (!cacheable_3)
                $display("[%0t] L1D TOP  ERROR VPU request idx %0d is non-cacheable: not built (open item PO-6), done as cacheable", $time, idx_from_cpu_3);
        end
    end
`endif

    // ======================================================================
    // PORT FRONT (l1d_port_front, cache_v6, PLAN_v6.md 4.5): decodes the bank of every port's request and sends it to that
    // bank's tracker in the arrival cycle (bank_arrival_request_N: port N's pins request is for bank b and nothing older
    // waits for b). A request the tracker cannot take (no free entry) waits in its port's skid (2 places per port); one
    // l1d_bank_order per bank keeps the arrival order and DRAINS the oldest waiting request into the bank (bank_drain_*)
    // one per cycle. The room rule is computed from the banks' flops (tracker free count, vector data place) on both sides:
    // no grant comes back from a bank to the front (rule T-1).
    // ======================================================================
    wire [NB-1:0]           bk_arrival_req_0;        // per bank: port 0's pins request is for it (ring empty), 4 bits
    wire [NB-1:0]           bk_arrival_req_1;        // ... port 1
    wire [NB-1:0]           bk_arrival_req_2;        // ... port 2
    wire [NB-1:0]           bk_arrival_req_3;        // ... port 3
    wire [NB-1:0]           bk_drain_req;            // per bank: the oldest skidded request for it asks for an entry, 4 bits
    wire [NB*REQ_W-1:0]     bk_drain_pkt;            // per bank: its packet, 4*148 = 592 bits
    wire [NB*PC_W-1:0]      bk_drain_pc;             // per bank: its PC, 4*64 bits
    wire [NB*LINE_W-1:0]    bk_arrival_line;         // per bank: the line of an arriving vector store, 4*512 bits
    wire [NB-1:0]           bk_arrival_alloc_0;      // per bank: the tracker took port 0's arrival (front check), 4 bits
    wire [NB-1:0]           bk_arrival_alloc_1;      // ... port 1
    wire [NB-1:0]           bk_arrival_alloc_2;      // ... port 2
    wire [NB-1:0]           bk_arrival_alloc_3;      // ... port 3
    wire [NB-1:0]           bk_drain_alloc;          // per bank: the tracker took the drained request (front check), 4 bits
    wire [NB*TRK_CNT_W-1:0] bk_tracker_free;         // per bank: free tracker entries (flops), 4*4 = 16 bits
    wire [NB-1:0]           bk_vec_slot_ok;          // per bank: a vector store data place is free, 4 bits
    wire                    port_stop_3;             // port 3 stop from the front (used only with a VPU)
    wire [PC_W-1:0]         pc_none;                 // the VPU port has no PC pin: its PC is 0 (open item O-8), 64 bits
    assign pc_none = {PC_W{1'b0}};

    l1d_port_front #(`L1D_PARAMS_PASS, .PORT_QUEUE_DEPTH(PORT_QUEUE_DEPTH), .LINE_PORT_ON(HAS_VPU)) u_front (
        .clk                      (clk),
        .rst                      (rst),
        .port_request_valid_0     (req_valid_0),
        .port_request_valid_1     (req_valid_1),
        .port_request_valid_2     (req_valid_2),
        .port_request_valid_3     (req_valid_3),
        .port_request_packet_0    (req_packet_0),
        .port_request_packet_1    (req_packet_1),
        .port_request_packet_2    (req_packet_2),
        .port_request_packet_3    (req_packet_3),
        .port_request_pc_0        (pc_cpu_0),
        .port_request_pc_1        (pc_cpu_1),
        .port_request_pc_2        (pc_cpu_2),
        .port_request_pc_3        (pc_none),
        .port_stop_0              (stop_0),
        .port_stop_1              (stop_1),
        .port_stop_2              (stop_2),
        .port_stop_3              (port_stop_3),
        .port_request_line_3      (data_from_cpu_3),    // the VPU line (ignored when HAS_VPU = 0)
        .bank_tracker_free_count  (bk_tracker_free),
        .bank_vec_slot_available  (bk_vec_slot_ok),
        .bank_arrival_request_0   (bk_arrival_req_0),
        .bank_arrival_request_1   (bk_arrival_req_1),
        .bank_arrival_request_2   (bk_arrival_req_2),
        .bank_arrival_request_3   (bk_arrival_req_3),
        .bank_drain_request       (bk_drain_req),
        .bank_drain_packet        (bk_drain_pkt),
        .bank_drain_pc            (bk_drain_pc),
        .bank_arrival_line_data   (bk_arrival_line),
        .bank_arrival_allocated_0 (bk_arrival_alloc_0),
        .bank_arrival_allocated_1 (bk_arrival_alloc_1),
        .bank_arrival_allocated_2 (bk_arrival_alloc_2),
        .bank_arrival_allocated_3 (bk_arrival_alloc_3),
        .bank_drain_allocated     (bk_drain_alloc)
    );
    // port 3 never sends without a VPU: its stop is then always 1
    assign stop_3 = (HAS_VPU != 0) ? port_stop_3 : 1'b1;

    // ======================================================================
    // Banks (one file, NUM_BANKS instances)
    // ======================================================================
    // ======================================================================
    // BANKS (one file, NUM_BANKS instances). The signals below are the buses between the banks and the shared units:
    //   bk_resp_*   each bank's response (carries its port number) and the credit of the bank's FIFO in every port's merger
    //   bk_rsv_event / other_rsv_event_q   A2 global LR/SC reservation: LR/SC done in a bank, and 'done in another bank' back
    //   bk_rd_* bk_r_*   read requests to the AXI master and the read data / id coming back
    //   bk_wr_* bk_b_*   write requests to the AXI master and the write response coming back
    //   mt_*        flush/clean start (to all banks), the operation, and the done pulse of each bank
    //   bk_snp_* bk_snpr_*  snoops to the banks and their answers to the snoop controller
    // ======================================================================
    wire [NB-1:0]               bk_resp_valid;
    wire [NB*RESP_W-1:0]        bk_resp;
    wire [NB*LINE_W-1:0]        bk_resp_line;       // C-20: per bank: the line a vector load answers with, 4*512 = 2048 bits
    wire [NB*RCNT_W-1:0]        resp_free_by_bank_0;   // port 0 merger: free places of each bank's FIFO (bank b = bits [b*3 +: 3]), 12 bits
    wire [NB*RCNT_W-1:0]        resp_free_by_bank_1;   // port 1 merger, 12 bits
    wire [NB*RCNT_W-1:0]        resp_free_by_bank_2;   // port 2 merger, 12 bits
    wire [NB*RCNT_W-1:0]        resp_free_by_bank_3;   // port 3 (VPU) merger, 12 bits
    wire [NB*RCNT_W-1:0]        resp_free_next_by_bank_0;  // cache_v6: port 0 merger: free places after this cycle (next credit), 12 bits
    wire [NB*RCNT_W-1:0]        resp_free_next_by_bank_1;  // port 1, 12 bits
    wire [NB*RCNT_W-1:0]        resp_free_next_by_bank_2;  // port 2, 12 bits
    wire [NB*RCNT_W-1:0]        resp_free_next_by_bank_3;  // port 3, 12 bits
    wire [NB-1:0]               bk_rsv_event;       // per bank: an LR or SC completed in Stage 2 this cycle
    reg  [NB-1:0]               other_rsv_event_q;  // per bank: an LR or SC completed in ANOTHER bank last cycle (flops)
    // cache_v6: prefetch hints to the banks, and the banks' records for the prefetch unit
    wire [NB-1:0]               pf_hint_valid;      // per bank: a prefetch hint, 4 bits
    wire [NB*LINE_ADDR_W-1:0]   pf_hint_line;       // per bank: its line address, 4*58 bits
    wire [NB-1:0]               pf_hint_unique;     // per bank: it wants the line unique, 4 bits
    wire [NB-1:0]               pf_hint_ready;      // per bank: the bank's prefetch register may take a hint, 4 bits
    wire [NB-1:0]               bk_train_valid;     // per bank: a training record, 4 bits
    wire [NB*PORT_W-1:0]        bk_train_port;      // per bank: its port (unused by the unit: SMS uses bank lanes), 4*2 bits
    wire [NB*PC_W-1:0]          bk_train_pc;        // per bank: its PC, 4*64 bits
    wire [NB*ADDR_W-1:0]        bk_train_addr;      // per bank: its byte address, 4*64 bits
    wire [NB-1:0]               bk_train_is_store;  // per bank: store class, 4 bits
    wire [NB-1:0]               bk_train_miss;      // per bank: it missed, 4 bits
    wire [NB-1:0]               bk_train_pf_hit;    // per bank: it hit a prefetched line, 4 bits
    wire [NB-1:0]               bk_train_no_train;  // per bank: ignore the record, 4 bits
    wire [NB-1:0]               bk_evict_valid;     // per bank: a line left the array, 4 bits
    wire [NB*LINE_ADDR_W-1:0]   bk_evict_line;      // per bank: its line address, 4*58 bits
    wire [NB-1:0]               bk_pf_fb_valid;     // per bank: feedback for a hint, 4 bits
    wire [NB*LINE_ADDR_W-1:0]   bk_pf_fb_line;      // per bank: its line address, 4*58 bits
    wire [NB*2-1:0]             bk_pf_fb_result;    // per bank: PF_FB_*, 4*2 bits
    wire [NB-1:0]               bk_ecc_ce;          // per bank: a scrub was written (pulse), 4 bits
    wire [NB-1:0]               bk_ecc_ue;          // per bank: an uncorrectable error was seen (pulse), 4 bits

    wire [NB-1:0]               bk_rd_valid, bk_rd_ready, bk_rd_nc, bk_rd_unique;
    wire [NB*AXI_ADDR_W-1:0]    bk_rd_addr;
    wire [NB*8-1:0]             bk_rd_len;
    wire [NB*3-1:0]             bk_rd_size;
    wire [NB*AXI_SUBID_W-1:0]   bk_rd_sub;
    wire [NB-1:0]               bk_r_valid;
    wire [AXI_SUBID_W-1:0]      bk_r_sub;
    wire                        bk_r_nc, bk_r_last, bk_r_err, bk_r_shared;
    wire [AXI_DATA_W-1:0]       bk_r_data;

    wire [NB-1:0]               bk_wr_valid, bk_wr_ready, bk_wr_nc;
    wire [NB*AXI_ADDR_W-1:0]    bk_wr_addr;
    wire [NB*8-1:0]             bk_wr_len;
    wire [NB*3-1:0]             bk_wr_size;
    wire [NB*AXI_SUBID_W-1:0]   bk_wr_sub;
    wire [NB*LINE_W-1:0]        bk_wr_data;
    wire [NB*AXI_STRB_W-1:0]    bk_wr_strb;
    wire [NB-1:0]               bk_b_valid;
    wire [AXI_SUBID_W-1:0]      bk_b_sub;
    wire                        bk_b_nc, bk_b_err;

    reg                         mt_start;      // pulse to every bank
    reg                         mt_op;         // MAINT_CLEAN / MAINT_FLUSH
    wire [NB-1:0]               mt_done;       // pulse from each bank
    wire [NB-1:0]               bk_lock_err;   // AMO lock time-out pulse from each bank
    wire                        l2_err;        // cache_v5: one-cycle pulse: an L2 answer came with l2_resp_err = 1
    wire [NB-1:0]               refill_busy;   // cache_v5: NB = 4 bits: the L2 port is handing a line to bank b (hold its snoops)

    wire [NB-1:0]               bk_snp_valid, bk_snp_ready;
    wire [LINE_ADDR_W-1:0]      bk_snp_line;
    wire                        bk_snp_inv;
    wire [NB-1:0]               bk_snpr_valid;
    wire [NB*SNPR_W-1:0]        bk_snpr_pkt;
    wire [NB*SNPR_CNT_W-1:0]    bk_snpr_free;

    // bank b: all its buses are slices of the wide signals (bk_xxx[b*W +: W]); the shared inputs (bk_snp_line, bk_r_data ...) go to every bank
    genvar b;
    generate
        for (b = 0; b < NB; b = b + 1) begin : g_bank
            l1d_bank #(`L1D_PARAMS_PASS, .BANK_ID(b)) u_bank (
                .clk                          (clk),
                .rst                          (rst),
                // cache_v6: the arrivals of the four ports (pins packets, bank selected by the front) and the drained request
                .arrival_request_0            (bk_arrival_req_0[b]),
                .arrival_request_1            (bk_arrival_req_1[b]),
                .arrival_request_2            (bk_arrival_req_2[b]),
                .arrival_request_3            (bk_arrival_req_3[b]),
                .arrival_packet_0             (req_packet_0),
                .arrival_packet_1             (req_packet_1),
                .arrival_packet_2             (req_packet_2),
                .arrival_packet_3             (req_packet_3),
                .arrival_pc_0                 (pc_cpu_0),
                .arrival_pc_1                 (pc_cpu_1),
                .arrival_pc_2                 (pc_cpu_2),
                .arrival_pc_3                 (pc_none),
                .drain_request                (bk_drain_req[b]),
                .drain_packet                 (bk_drain_pkt[b*REQ_W +: REQ_W]),
                .drain_pc                     (bk_drain_pc[b*PC_W +: PC_W]),
                .arrival_allocated_0          (bk_arrival_alloc_0[b]),
                .arrival_allocated_1          (bk_arrival_alloc_1[b]),
                .arrival_allocated_2          (bk_arrival_alloc_2[b]),
                .arrival_allocated_3          (bk_arrival_alloc_3[b]),
                .drain_allocated              (bk_drain_alloc[b]),
                .tracker_free_count_q         (bk_tracker_free[b*TRK_CNT_W +: TRK_CNT_W]),
                .vec_slot_available           (bk_vec_slot_ok[b]),
                .lsu_request_line_data        (bk_arrival_line[b*LINE_W +: LINE_W]),      // C-20: vector store line at the bank door
                .lsu_response_valid           (bk_resp_valid[b]),
                .lsu_response_packet          (bk_resp[b*RESP_W +: RESP_W]),
                .lsu_response_line_data       (bk_resp_line[b*LINE_W +: LINE_W]),        // C-20: vector load line beside the answer
                .lsu_response_fifo_free_0     (resp_free_by_bank_0[b*RCNT_W +: RCNT_W]),   // this bank's credit in the port 0 merger
                .lsu_response_fifo_free_1     (resp_free_by_bank_1[b*RCNT_W +: RCNT_W]),   // ... port 1
                .lsu_response_fifo_free_2     (resp_free_by_bank_2[b*RCNT_W +: RCNT_W]),   // ... port 2
                .lsu_response_fifo_free_3     (resp_free_by_bank_3[b*RCNT_W +: RCNT_W]),   // ... port 3
                .lsu_response_fifo_free_next_0 (resp_free_next_by_bank_0[b*RCNT_W +: RCNT_W]),   // the same after this cycle
                .lsu_response_fifo_free_next_1 (resp_free_next_by_bank_1[b*RCNT_W +: RCNT_W]),
                .lsu_response_fifo_free_next_2 (resp_free_next_by_bank_2[b*RCNT_W +: RCNT_W]),
                .lsu_response_fifo_free_next_3 (resp_free_next_by_bank_3[b*RCNT_W +: RCNT_W]),
                .snoop_request_valid          (bk_snp_valid[b]),
                .snoop_request_line_addr      (bk_snp_line),
                .snoop_request_invalidate     (bk_snp_inv),
                .snoop_request_ready          (bk_snp_ready[b]),
                .snoop_response_valid         (bk_snpr_valid[b]),
                .snoop_response_packet        (bk_snpr_pkt[b*SNPR_W +: SNPR_W]),
                .snoop_response_fifo_free     (bk_snpr_free[b*SNPR_CNT_W +: SNPR_CNT_W]),
                .prefetch_request_valid       (pf_hint_valid[b]),
                .prefetch_request_line_addr   (pf_hint_line[b*LINE_ADDR_W +: LINE_ADDR_W]),
                .prefetch_request_unique      (pf_hint_unique[b]),
                .prefetch_request_ready       (pf_hint_ready[b]),
                .axi_read_req_valid           (bk_rd_valid[b]),
                .axi_read_req_ready           (bk_rd_ready[b]),
                .axi_read_req_addr            (bk_rd_addr[b*AXI_ADDR_W +: AXI_ADDR_W]),
                .axi_read_req_burst_len       (bk_rd_len[b*8 +: 8]),
                .axi_read_req_size            (bk_rd_size[b*3 +: 3]),
                .axi_read_req_sub_id          (bk_rd_sub[b*AXI_SUBID_W +: AXI_SUBID_W]),
                .axi_read_req_non_cacheable   (bk_rd_nc[b]),
                .axi_read_req_unique          (bk_rd_unique[b]),
                .axi_read_data_valid          (bk_r_valid[b]),
                .axi_read_data_sub_id         (bk_r_sub),
                .axi_read_data_non_cacheable  (bk_r_nc),
                .axi_read_data                (bk_r_data),
                .axi_read_data_last           (bk_r_last),
                .axi_read_data_error          (bk_r_err),
                .axi_read_data_shared         (bk_r_shared),
                .axi_write_req_valid          (bk_wr_valid[b]),
                .axi_write_req_ready          (bk_wr_ready[b]),
                .axi_write_req_addr           (bk_wr_addr[b*AXI_ADDR_W +: AXI_ADDR_W]),
                .axi_write_req_burst_len      (bk_wr_len[b*8 +: 8]),
                .axi_write_req_size           (bk_wr_size[b*3 +: 3]),
                .axi_write_req_sub_id         (bk_wr_sub[b*AXI_SUBID_W +: AXI_SUBID_W]),
                .axi_write_req_non_cacheable  (bk_wr_nc[b]),
                .axi_write_req_data           (bk_wr_data[b*LINE_W +: LINE_W]),
                .axi_write_req_strobe         (bk_wr_strb[b*AXI_STRB_W +: AXI_STRB_W]),
                .axi_write_resp_valid         (bk_b_valid[b]),
                .axi_write_resp_sub_id        (bk_b_sub),
                .axi_write_resp_non_cacheable (bk_b_nc),
                .axi_write_resp_error         (bk_b_err),
                .refill_handover_busy         (refill_busy[b]),
                .maint_start                  (mt_start),
                .maint_op_is_flush            (mt_op),
                .maint_done                   (mt_done[b]),
                .amo_lock_timeout_pulse       (bk_lock_err[b]),
                .rsv_event_pulse              (bk_rsv_event[b]),            // A2: LR/SC done in this bank
                .other_bank_rsv_event         (other_rsv_event_q[b]),       // A2: LR/SC done in another bank last cycle
                // cache_v6: records for the prefetch unit and the ECC pulses
                .train_valid                  (bk_train_valid[b]),
                .train_port                   (bk_train_port[b*PORT_W +: PORT_W]),
                .train_pc                     (bk_train_pc[b*PC_W +: PC_W]),
                .train_addr                   (bk_train_addr[b*ADDR_W +: ADDR_W]),
                .train_is_store               (bk_train_is_store[b]),
                .train_miss                   (bk_train_miss[b]),
                .train_pf_hit                 (bk_train_pf_hit[b]),
                .train_no_train               (bk_train_no_train[b]),
                .evict_valid                  (bk_evict_valid[b]),
                .evict_line_addr              (bk_evict_line[b*LINE_ADDR_W +: LINE_ADDR_W]),
                .pf_fb_valid                  (bk_pf_fb_valid[b]),
                .pf_fb_line_addr              (bk_pf_fb_line[b*LINE_ADDR_W +: LINE_ADDR_W]),
                .pf_fb_result                 (bk_pf_fb_result[b*2 +: 2]),
                .ecc_correctable              (bk_ecc_ce[b]),
                .ecc_uncorrectable            (bk_ecc_ue[b])
            );
        end
    endgenerate

    // ======================================================================
    // PREFETCH UNIT (cache_v6, l1d_prefetch_unit): the stride, stream and SMS prefetchers beside the banks. Trained by the
    // banks' records, it sends at most one hint per bank per cycle into the bank's prefetch register. Without HAS_PF the
    // banks see no hints (their prefetch path stays built but idle).
    // ======================================================================
    generate
        if (HAS_PF != 0) begin : g_prefetch
            l1d_prefetch_unit #(`L1D_PARAMS_PASS) u_prefetch (
                .clk              (clk),
                .rst              (rst),
                .pf_enable        (pf_enable),
                .train_valid      (bk_train_valid),
                .train_pc         (bk_train_pc),
                .train_addr       (bk_train_addr),
                .train_is_store   (bk_train_is_store),
                .train_miss       (bk_train_miss),
                .train_pf_hit     (bk_train_pf_hit),
                .train_no_train   (bk_train_no_train),
                .evict_valid      (bk_evict_valid),
                .evict_line_addr  (bk_evict_line),
                .pf_fb_valid      (bk_pf_fb_valid),
                .pf_fb_line_addr  (bk_pf_fb_line),
                .pf_fb_result     (bk_pf_fb_result),
                .pf_req_ready     (pf_hint_ready),
                .pf_req_valid     (pf_hint_valid),
                .pf_req_line_addr (pf_hint_line),
                .pf_req_unique    (pf_hint_unique)
            );
        end else begin : g_no_prefetch
            assign pf_hint_valid  = {NB{1'b0}};
            assign pf_hint_line   = {(NB*LINE_ADDR_W){1'b0}};
            assign pf_hint_unique = {NB{1'b0}};
        end
    endgenerate

    // ======================================================================
    // ECC STATUS (cache_v6): one pulse per cycle in which some bank wrote a scrub; a sticky flag for an uncorrectable error
    // ======================================================================
    always @(posedge clk) begin : ecc_status_proc
        if (rst) begin
            ecc_correctable_pulse    <= 1'b0;
            ecc_uncorrectable_sticky <= 1'b0;
        end else begin
            ecc_correctable_pulse <= |bk_ecc_ce;
            if (|bk_ecc_ue) ecc_uncorrectable_sticky <= 1'b1;
        end
    end // always ecc_status_proc

    // ======================================================================
    // Snoop control unit
    // ======================================================================
    // ======================================================================
    // SNOOP CONTROL UNIT: routes snoops to the addressed bank and merges the banks' answers
    // ======================================================================
    l1d_snoop_ctrl #(`L1D_PARAMS_PASS) u_snoop (
        .clk                           (clk),
        .rst                           (rst),
        .snoop_request_valid           (snoop_request_valid),
        .snoop_request_ready           (snoop_request_ready),
        .snoop_request_addr            (snoop_request_addr),
        .snoop_request_invalidate      (snoop_request_invalidate),
        .bank_snoop_valid              (bk_snp_valid),
        .bank_snoop_line_addr          (bk_snp_line),
        .bank_snoop_invalidate         (bk_snp_inv),
        .bank_snoop_ready              (bk_snp_ready),
        .bank_snoop_response_valid     (bk_snpr_valid),
        .bank_snoop_response_packet    (bk_snpr_pkt),
        .bank_snoop_response_fifo_free (bk_snpr_free),
        .snoop_response_valid          (snoop_response_valid),
        .snoop_response_ready          (snoop_response_ready),
        .snoop_response_addr           (snoop_response_addr),
        .snoop_response_present        (snoop_response_present),
        .snoop_response_dirty          (snoop_response_dirty),
        .snoop_response_data           (snoop_response_data)
    );

    // ======================================================================
    // RESPONSE PORTS: every bank answer carries the number of the port that asked (RESP_PORT_LSB field). Each port has its own
    // l1d_resp_merge (u_merge_0 .. u_merge_3) with one FIFO per bank; a bank answer is pushed only into the merger of its port
    // (resp_for_port_N, one bit per bank). Each merger's free counts go back to the banks (resp_free_by_bank_N), so a bank
    // issues a request only when its port's FIFO has room (credit_q in l1d_bank_s0, from the free_next counts).
    // Example: bank 1 answers a load of port 0 -> resp_for_port_0 = 4'b0010; the answer appears on final_r_*_0 while the
    //          other ports keep showing their own answers.
    // Port 3 (VPU, HAS_VPU = 1): its merger is 586 bits wide (EXTRA_W = LINE_W) and stores {line, answer packet with its 64-bit
    // data field set to 0}.
    // ======================================================================
    localparam VPKT_W = RESP_W + LINE_W;             // VPU answer stored in its merger: 74 + 512 = 586 bits
    localparam [RESP_W-1:0] RESP_DATA_MASK = {{(RESP_W-WRITE_DATA_WIDTH){1'b0}}, {WRITE_DATA_WIDTH{1'b1}}} << RESP_DATA_LSB;  // the 64-bit data field
    localparam [RCNT_W-1:0] RESP_FIFO_FULL_FREE = RESP_FIFO_DEPTH;   // free count of an empty response FIFO, 3 bits
    wire [NB-1:0]        resp_for_port_0;            // per bank: its answer is valid and is for port 0, 4 bits
    wire [NB-1:0]        resp_for_port_1;            // ... for port 1
    wire [NB-1:0]        resp_for_port_2;            // ... for port 2
    wire [NB-1:0]        resp_for_port_3;            // ... for port 3
    wire [NB*VPKT_W-1:0] vpu_bank_resp;              // per bank: {line, packet without data} for the VPU merger, 4*586 = 2344 bits
    genvar route_bank;                               // loop variable: the bank whose answer is routed
    generate
        for (route_bank = 0; route_bank < NB; route_bank = route_bank + 1) begin : g_route
            wire [PORT_W-1:0] answer_port = bk_resp[route_bank*RESP_W + RESP_PORT_LSB +: PORT_W];   // port field of the answer, 2 bits
            assign resp_for_port_0[route_bank] = bk_resp_valid[route_bank] && (answer_port == 2'd0);
            assign resp_for_port_1[route_bank] = bk_resp_valid[route_bank] && (answer_port == 2'd1);
            assign resp_for_port_2[route_bank] = bk_resp_valid[route_bank] && (answer_port == 2'd2);
            assign resp_for_port_3[route_bank] = bk_resp_valid[route_bank] && (answer_port == 2'd3);
            assign vpu_bank_resp[route_bank*VPKT_W +: VPKT_W] =
                {bk_resp_line[route_bank*LINE_W +: LINE_W], (bk_resp[route_bank*RESP_W +: RESP_W] & ~RESP_DATA_MASK)};
        end
    endgenerate

    wire              merged_valid_0, merged_valid_1, merged_valid_2;   // ports 0..2: an answer is shown
    wire [RESP_W-1:0] merged_packet_0;               // port 0 answer packet, RESP_W = 74 bits
    wire [RESP_W-1:0] merged_packet_1;               // port 1 answer packet, 74 bits
    wire [RESP_W-1:0] merged_packet_2;               // port 2 answer packet, 74 bits
    l1d_resp_merge #(`L1D_PARAMS_PASS) u_merge_0 (
        .clk (clk), .rst (rst),
        .bank_response_valid (resp_for_port_0), .bank_response_packet (bk_resp), .bank_response_fifo_free (resp_free_by_bank_0),
        .bank_response_fifo_free_next (resp_free_next_by_bank_0),
        .merged_response_valid (merged_valid_0), .merged_response_packet (merged_packet_0), .lsu_read_enable (read_enable_from_lsu_0)
    );
    l1d_resp_merge #(`L1D_PARAMS_PASS) u_merge_1 (
        .clk (clk), .rst (rst),
        .bank_response_valid (resp_for_port_1), .bank_response_packet (bk_resp), .bank_response_fifo_free (resp_free_by_bank_1),
        .bank_response_fifo_free_next (resp_free_next_by_bank_1),
        .merged_response_valid (merged_valid_1), .merged_response_packet (merged_packet_1), .lsu_read_enable (read_enable_from_lsu_1)
    );
    l1d_resp_merge #(`L1D_PARAMS_PASS) u_merge_2 (
        .clk (clk), .rst (rst),
        .bank_response_valid (resp_for_port_2), .bank_response_packet (bk_resp), .bank_response_fifo_free (resp_free_by_bank_2),
        .bank_response_fifo_free_next (resp_free_next_by_bank_2),
        .merged_response_valid (merged_valid_2), .merged_response_packet (merged_packet_2), .lsu_read_enable (read_enable_from_lsu_2)
    );
    // the port pins are fields of the merged packets (field positions: RESP_* in l1d_fields.vh)
    assign final_r_cache_hit_from_scheduler_0 = merged_valid_0;
    assign final_r_mem_size_from_scheduler_0  = merged_packet_0[RESP_SIZE_LSB +: MEM_SIZE_BITS];
    assign final_r_master_id_from_scheduler_0 = merged_packet_0[RESP_MID_LSB  +: MASTER_ID_BITS];
    assign final_r_idx_from_scheduler_0       = merged_packet_0[RESP_IDX_LSB  +: IDX_BITS];
    assign final_r_data_from_scheduler_0      = merged_packet_0[RESP_DATA_LSB +: WRITE_DATA_WIDTH];
    assign final_r_cache_hit_from_scheduler_1 = merged_valid_1;
    assign final_r_mem_size_from_scheduler_1  = merged_packet_1[RESP_SIZE_LSB +: MEM_SIZE_BITS];
    assign final_r_master_id_from_scheduler_1 = merged_packet_1[RESP_MID_LSB  +: MASTER_ID_BITS];
    assign final_r_idx_from_scheduler_1       = merged_packet_1[RESP_IDX_LSB  +: IDX_BITS];
    assign final_r_data_from_scheduler_1      = merged_packet_1[RESP_DATA_LSB +: WRITE_DATA_WIDTH];
    assign final_r_cache_hit_from_scheduler_2 = merged_valid_2;
    assign final_r_hit_rd_wr_from_scheduler_2 = merged_packet_2[RESP_RDWR_BIT];
    assign final_r_mem_size_from_scheduler_2  = merged_packet_2[RESP_SIZE_LSB +: MEM_SIZE_BITS];
    assign final_r_master_id_from_scheduler_2 = merged_packet_2[RESP_MID_LSB  +: MASTER_ID_BITS];
    assign final_r_idx_from_scheduler_2       = merged_packet_2[RESP_IDX_LSB  +: IDX_BITS];
    assign final_r_data_from_scheduler_2      = merged_packet_2[RESP_DATA_LSB +: WRITE_DATA_WIDTH];

    // port 3 (VPU): the same merger with the line on top of each answer; without a VPU there are no port 3 answers
    generate
        if (HAS_VPU != 0) begin : g_vpu_out
            wire              merged_valid_3;        // a port 3 answer is shown
            wire [VPKT_W-1:0] merged_packet_3;       // the merged VPU answer {line, packet}, 586 bits
            l1d_resp_merge #(`L1D_PARAMS_PASS, .EXTRA_W(LINE_W)) u_merge_3 (
                .clk (clk), .rst (rst),
                .bank_response_valid (resp_for_port_3), .bank_response_packet (vpu_bank_resp),
                .bank_response_fifo_free (resp_free_by_bank_3),
                .bank_response_fifo_free_next (resp_free_next_by_bank_3),
                .merged_response_valid (merged_valid_3), .merged_response_packet (merged_packet_3),
                .lsu_read_enable (read_enable_from_lsu_3)
            );
            assign final_r_cache_hit_from_scheduler_3 = merged_valid_3;
            assign final_r_hit_rd_wr_from_scheduler_3 = merged_packet_3[RESP_RDWR_BIT];
            assign final_r_master_id_from_scheduler_3 = merged_packet_3[RESP_MID_LSB +: MASTER_ID_BITS];
            assign final_r_idx_from_scheduler_3       = merged_packet_3[RESP_IDX_LSB +: IDX_BITS];
            assign final_r_data_from_scheduler_3      = merged_packet_3[RESP_W +: LINE_W];
        end else begin : g_no_vpu
            // no port 3 requests are ever sent: its answer pins are 0 and every bank sees a free port 3 FIFO
            assign resp_free_by_bank_3                = {NB{RESP_FIFO_FULL_FREE}};
            assign resp_free_next_by_bank_3           = {NB{RESP_FIFO_FULL_FREE}};
            assign final_r_cache_hit_from_scheduler_3 = 1'b0;
            assign final_r_hit_rd_wr_from_scheduler_3 = 1'b0;
            assign final_r_master_id_from_scheduler_3 = {MASTER_ID_BITS{1'b0}};
            assign final_r_idx_from_scheduler_3       = {IDX_BITS{1'b0}};
            assign final_r_data_from_scheduler_3      = {LINE_W{1'b0}};
        end
    endgenerate

    // ======================================================================
    // GLOBAL LR/SC RESERVATION (A2). RISC-V allows one reservation per hart. Each bank keeps its own reservation unit, so when
    // an LR or SC completes in one bank, every OTHER bank must drop its reservation. The OR of the other banks' events is
    // registered (one flop per bank) to keep this path short (GR-1); the receiving bank also blocks an SC match in that
    // cycle (l1d_reservation_unit).
    // Example: LR in bank 0 (cycle t)  ->  other_rsv_event_q = 4'b1110 at t+1  ->  banks 1..3 clear their reservation.
    // ======================================================================
    integer lr_bank;                                 // loop variable: the bank whose "other bank" flag is computed
    always @(posedge clk) begin
        if (rst) other_rsv_event_q <= {NB{1'b0}};
        else
            for (lr_bank = 0; lr_bank < NB; lr_bank = lr_bank + 1)
                // OR of all banks' events except bank lr_bank itself (mask out bit lr_bank)
                other_rsv_event_q[lr_bank] <= |(bk_rsv_event & ~({{(NB-1){1'b0}}, 1'b1} << lr_bank));
    end

    // ======================================================================
    // L2 packet port
    // ======================================================================
    // ======================================================================
    // L2 PORT (cache_v5): shares the one L2 packet port between the banks (l1d_l2_port, from cache_v1_4). No AXI here;
    // AXI exists only in the dummy L2 (dummy_l2/l2_pkt_axi_bridge.v). refill_busy[b] holds bank b's snoops while a line
    // is handed to it beat by beat.
    // ======================================================================

    l1d_l2_port #(`L1D_PARAMS_PASS, .L2_ADDR_W(L2_ADDR_W), .L2_MID_W(L2_MID_W), .L2_TID_W(L2_TID_W),
                  .L2_MASTER_ID(L2_MASTER_ID)) u_l2_port (   // L2_ADDR_W = 56, L2_TID_W = 6
        .clk                           (clk),
        .rst                           (rst),
        .bank_read_req_valid           (bk_rd_valid),
        .bank_read_req_ready           (bk_rd_ready),
        .bank_read_req_addr            (bk_rd_addr),
        .bank_read_req_burst_len       (bk_rd_len),
        .bank_read_req_size            (bk_rd_size),
        .bank_read_req_sub_id          (bk_rd_sub),
        .bank_read_req_non_cacheable   (bk_rd_nc),
        .bank_read_req_unique          (bk_rd_unique),
        .bank_read_data_valid          (bk_r_valid),
        .bank_read_data_sub_id         (bk_r_sub),
        .bank_read_data_non_cacheable  (bk_r_nc),
        .bank_read_data                (bk_r_data),
        .bank_read_data_last           (bk_r_last),
        .bank_read_data_error          (bk_r_err),
        .bank_read_data_shared         (bk_r_shared),
        .bank_write_req_valid          (bk_wr_valid),
        .bank_write_req_ready          (bk_wr_ready),
        .bank_write_req_addr           (bk_wr_addr),
        .bank_write_req_burst_len      (bk_wr_len),
        .bank_write_req_size           (bk_wr_size),
        .bank_write_req_sub_id         (bk_wr_sub),
        .bank_write_req_non_cacheable  (bk_wr_nc),
        .bank_write_req_data           (bk_wr_data),
        .bank_write_req_strobe         (bk_wr_strb),
        .bank_write_resp_valid         (bk_b_valid),
        .bank_write_resp_sub_id        (bk_b_sub),
        .bank_write_resp_non_cacheable (bk_b_nc),
        .bank_write_resp_error         (bk_b_err),
        .bank_refill_replay_busy       (refill_busy),
        .l2_error_pulse                (l2_err),
        .l2_req_valid                  (l2_req_valid),
        .l2_req_ready                  (l2_req_ready),
        .l2_req_pkt                    (l2_req_pkt),
        .l2_resp_valid                 (l2_resp_valid),
        .l2_resp_ready                 (l2_resp_ready),
        .l2_resp_pkt                   (l2_resp_pkt),
        .l2_resp_err                   (l2_resp_err)
    );

    // sticky error flags: set by the first L2 error answer or the first AMO lock time-out; cleared only by reset
    always @(posedge clk) begin
        if (rst)         mem_err <= 1'b0;
        else if (l2_err) mem_err <= 1'b1;
    end

    always @(posedge clk) begin
        if (rst)               amo_err <= 1'b0;
        else if (|bk_lock_err) amo_err <= 1'b1;
    end

    // ======================================================================
    // Flush / clean control: start every bank, pulse the completion output
    // when all banks have finished. A request while one is running is ignored;
    // flush wins if both arrive together.
    // ======================================================================
    // ======================================================================
    // FLUSH / CLEAN CONTROL: start every bank, and pulse the completion output when ALL banks have finished. A request while one
    // is running is ignored; flush wins if both arrive together.
    //   mt_busy a walk is running   mt_fin banks that have finished   mt_fin_now = mt_fin | the banks finishing this cycle
    // ======================================================================
    reg          mt_busy;
    reg [NB-1:0] mt_fin;              // banks that have finished
    wire [NB-1:0] mt_fin_now = mt_fin | mt_done;

    always @(posedge clk) begin
        if (rst) begin
            mt_busy        <= 1'b0;
            mt_start       <= 1'b0;
            mt_op          <= MAINT_CLEAN;
            mt_fin         <= {NB{1'b0}};
            flush_complete <= 1'b0;
            clean_complete <= 1'b0;
        end else begin
            mt_start       <= 1'b0;
            flush_complete <= 1'b0;
            clean_complete <= 1'b0;
            if (!mt_busy) begin
                if (flush || clean) begin
                    mt_busy  <= 1'b1;
                    mt_start <= 1'b1;
                    mt_op    <= flush ? MAINT_FLUSH : MAINT_CLEAN;
                    mt_fin   <= {NB{1'b0}};
                end
            end else if (&mt_fin_now) begin
                mt_busy        <= 1'b0;
                mt_fin         <= {NB{1'b0}};
                flush_complete <= (mt_op == MAINT_FLUSH);
                clean_complete <= (mt_op == MAINT_CLEAN);
            end else begin
                mt_fin <= mt_fin_now;
            end
        end
    end

// ---- simulation-only trace: one entry per ROB index. Checks that an index is not reused before its response (a valid_cpu_N held for
//      more than one cycle shows up like this), reports the latency of every response, and reports any request that waits
//      L1D_DBG_WD cycles or more. ----
`ifdef L1D_DEBUG
    // ======================================================================
    // Debug trace (simulation only): one entry per ROB index. Checks that an
    // index is not reused before its response (a valid_cpu held for more than
    // one cycle shows up like this), reports the latency of every response,
    // and reports any request that waits L1D_DBG_WD cycles or more.
    // ======================================================================
    // A2: one entry per (port, ROB index): entry number = port * 2**IDX_BITS + index (each port has its own ROB indexes)
    localparam DBG_N = LSU_PORTS * (1 << IDX_BITS);   // trace entries, DBG_N = 4 * 8 = 32
    reg              dbg_ov [0:DBG_N-1];      // outstanding
    reg              dbg_ow [0:DBG_N-1];      // store
    reg              dbg_oc [0:DBG_N-1];      // cacheable
    reg [ADDR_W-1:0] dbg_oa [0:DBG_N-1];      // address
    integer          dbg_ot [0:DBG_N-1];      // issue cycle
    integer          dbg_cyc, dbg_i, dbg_age;   // cycle counter, entry loop, age

    initial begin
        dbg_cyc = 0;
        for (dbg_i = 0; dbg_i < DBG_N; dbg_i = dbg_i + 1) dbg_ov[dbg_i] = 1'b0;
    end

    // an answer taken by the LSU on port port_num: check it was outstanding and print its latency
    task dbg_answer;
        input integer        port_num;                // the response port, 0..3
        input                answer_valid;            // an answer is shown
        input                answer_taken;            // the LSU takes it (read enable)
        input [IDX_BITS-1:0] answer_idx;              // its ROB index, 3 bits
        integer              entry_num;               // trace entry of (port, index)
        begin
            if (answer_valid && answer_taken) begin
                entry_num = port_num * (1 << IDX_BITS) + answer_idx;
                if (!dbg_ov[entry_num])
                    $display("[%0t] L1D TOP  ERROR port %0d response for idx %0d, which is not outstanding", $time, port_num, answer_idx);
                else
                    $display("[%0t] L1D TOP  DONE  port %0d idx=%0d %s %s addr=%h latency=%0d cycles",
                             $time, port_num, answer_idx, (dbg_ow[entry_num] ? "ST" : "LD"), (dbg_oc[entry_num] ? "C " : "NC"),
                             dbg_oa[entry_num], dbg_cyc - dbg_ot[entry_num]);
                dbg_ov[entry_num] = 1'b0;
            end
        end
    endtask
    // a new request on port port_num: an index may be reused only after its answer
    task dbg_request;
        input integer        port_num;                // the request port, 0..3
        input                request_valid;           // a request this cycle
        input [IDX_BITS-1:0] request_idx;             // its ROB index, 3 bits
        input [ADDR_W-1:0]   request_addr;            // its address, 64 bits
        input                request_is_write;        // a store / SC / AMO / vector store
        input                request_cacheable;       // cacheable
        integer              entry_num;               // trace entry of (port, index)
        begin
            if (request_valid) begin
                entry_num = port_num * (1 << IDX_BITS) + request_idx;
                if (dbg_ov[entry_num])
                    $display("[%0t] L1D TOP  ERROR port %0d idx %0d reused while still outstanding (old addr %h, issued %0d cycles ago; new addr %h). valid_cpu_N must be 1 for exactly one cycle per request, and an index may be reused only after its response.",
                             $time, port_num, request_idx, dbg_oa[entry_num], dbg_cyc - dbg_ot[entry_num], request_addr);
                dbg_ov[entry_num] = 1'b1;
                dbg_ow[entry_num] = request_is_write;
                dbg_oc[entry_num] = request_cacheable;
                dbg_oa[entry_num] = request_addr;
                dbg_ot[entry_num] = dbg_cyc;
            end
        end
    endtask

    always @(posedge clk) begin
        if (rst) begin
            dbg_cyc = 0;
            for (dbg_i = 0; dbg_i < DBG_N; dbg_i = dbg_i + 1) dbg_ov[dbg_i] = 1'b0;
        end else begin
            dbg_cyc = dbg_cyc + 1;
            // answers taken and new requests, port by port
            dbg_answer(0, final_r_cache_hit_from_scheduler_0, read_enable_from_lsu_0, final_r_idx_from_scheduler_0);
            dbg_answer(1, final_r_cache_hit_from_scheduler_1, read_enable_from_lsu_1, final_r_idx_from_scheduler_1);
            dbg_answer(2, final_r_cache_hit_from_scheduler_2, read_enable_from_lsu_2, final_r_idx_from_scheduler_2);
            dbg_answer(3, final_r_cache_hit_from_scheduler_3, read_enable_from_lsu_3, final_r_idx_from_scheduler_3);
            dbg_request(0, req_valid_0, idx_from_cpu_0, PC_from_cpu_0, 1'b0, cacheable_0);
            dbg_request(1, req_valid_1, idx_from_cpu_1, PC_from_cpu_1, 1'b0, cacheable_1);
            dbg_request(2, req_valid_2, idx_from_cpu_2, PC_from_cpu_2, 1'b1, cacheable_2);
            dbg_request(3, req_valid_3, idx_from_cpu_3, PC_from_cpu_3, rd_wr_cpu_3, 1'b1);
            // watchdog
            for (dbg_i = 0; dbg_i < DBG_N; dbg_i = dbg_i + 1) begin
                dbg_age = dbg_cyc - dbg_ot[dbg_i];
                if (dbg_ov[dbg_i] && dbg_age > 0 && (dbg_age % `L1D_DBG_WD) == 0)
                    $display("[%0t] L1D TOP  STUCK port %0d idx=%0d %s %s addr=%h (bank %0d) has waited %0d cycles for its response; stop (ports 3..0)=%b, output valid=%b, read_enable=%b",
                             $time, dbg_i / (1 << IDX_BITS), dbg_i % (1 << IDX_BITS), (dbg_ow[dbg_i] ? "ST" : "LD"), (dbg_oc[dbg_i] ? "C " : "NC"),
                             dbg_oa[dbg_i], dbg_oa[dbg_i][BANK_LSB +: BANK_W], dbg_age,
                             {stop_3, stop_2, stop_1, stop_0},
                             {final_r_cache_hit_from_scheduler_3, final_r_cache_hit_from_scheduler_2, final_r_cache_hit_from_scheduler_1, final_r_cache_hit_from_scheduler_0},
                             {read_enable_from_lsu_3, read_enable_from_lsu_2, read_enable_from_lsu_1, read_enable_from_lsu_0});
            end
            // maintenance and errors
            if (!mt_busy && (flush || clean))
                $display("[%0t] L1D TOP  MAINT start %s", $time, flush ? "flush" : "clean");
            if (mt_busy && (flush || clean))
                $display("[%0t] L1D TOP  WARNING flush/clean request ignored: one is already running", $time);
            if (mt_busy && (&mt_fin_now))
                $display("[%0t] L1D TOP  MAINT all banks done", $time);
            if (l2_err && !mem_err)
                $display("[%0t] L1D TOP  ERROR L2 error answer: mem_err set", $time);
        end
    end
`endif

endmodule

`default_nettype wire
