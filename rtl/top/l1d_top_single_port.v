// ============================================================================
// l1d_top_single_port.v  -  L1 DATA CACHE v5 (port managed) behind ONE request port and ONE response port
// ============================================================================
// IN SIMPLE WORDS
//   cache_v5_port_managed has four typed request ports (0 / 1 loads, 2 stores and atomics, 3 VPU) and one answer port per
//   request port. An older core that has a SINGLE load/store port, like the core of cache_v2_name_changed_timing_opt, cannot
//   connect to that directly. This wrapper gives the cache the CPU-side pins of cache_v2_name_changed_timing_opt l1d_top
//   (cpu_request_* in, lsu_response_* out, input_queue_full, flush_request / clean_request) and inside:
//     1. sends a load or LR to port 0, and a store, SC or AMO to port 2 (ports 1 and 3 are not used);
//     2. keeps the v2 rule "a request is taken in every cycle cpu_request_valid = 1": a request that cannot go to its port this
//        cycle (the port's stop is 1) waits in a small input queue (one place per ROB index, as the v2 input queue u_inq),
//        and the queue is emptied IN ORDER, one request per cycle; with an empty queue a request goes straight through
//        (no added cycle on a hit);
//     3. merges the two answer ports (0 and 2) into the single lsu_response_* port: an answer stays on the pins until the
//        LSU takes it (lsu_response_read_enable = 1), and when both ports have an answer they take turns (no port starves).
//   The memory side is the L2 packet port of cache_v5 (no AXI inside the D-cache, README decision 6). For an AXI memory, put
//   dummy_l2/l2_pkt_axi_bridge.v next to it, as dummy_l2/l1d_top_axi_ports.v does.
//
//      single-port CPU pins --> [input queue + port pick] --> port 0 (load, LR) / port 2 (store, SC, AMO) --> l1d_top u_dcache
//      l1d_top u_dcache answer port 0 / port 2 --> [answer merge, turn taking] --> single lsu_response_* pins
//      l1d_top u_dcache <--> l2_req_* / l2_resp_* (L2 packet port)          snoop_* pins: straight through
//
// DIFFERENCES TO THE cache_v2_name_changed_timing_opt l1d_top PINS (the reference)
//   | v2 pin / behaviour                              | here                                                               |
//   |-------------------------------------------------|--------------------------------------------------------------------|
//   | cpu_request_*, lsu_response_*, input_queue_full | same names, widths and meaning                                     |
//   | AMO = two requests: AMO read (locks the line),  | AMO = ONE request (computed in the cache, cache_v5 / bank_pipe):    |
//   |   then AMO write with the computed value        |   cpu_request_is_atomic = 1, cpu_request_atomic_is_amo = 1, operand |
//   |                                                 |   in cpu_request_store_data, operation on the NEW pin               |
//   |                                                 |   cpu_request_amo_op; the answer is the old value                   |
//   | LR / SC                                         | same: is_atomic = 1, atomic_is_amo = 0, is_write = 0 LR / 1 SC;     |
//   |                                                 |   SC answers 0 = success, 1 = fail in the data word                 |
//   | m_axi_* (AXI4 master inside the cache)          | l2_req_* / l2_resp_* L2 packet port (643-bit packet)                |
//   | axi_error_sticky                                | l2_error_sticky (an L2 answer came with l2_resp_err = 1)            |
//   | amo_lock_timeout_sticky                         | same                                                               |
//   | flush_request, clean_request, flush_complete,   | same                                                               |
//   |   clean_complete, snoop_*                       |                                                                    |
//   | requests always pass the input queue (1 cycle)  | queue only when the target port is stopped (bypass when empty)       |
//
// PARAMETERS (defaults from l1d_defines.vh; the ones that matter here)
//   | name             | default | meaning                                                                         |
//   |------------------|---------|---------------------------------------------------------------------------------|
//   | IDX_BITS         | 3       | ROB index bits                                                                  |
//   | INQ_DEPTH        | 8       | input queue places = 2**IDX_BITS (one per ROB index, as v2), at least 2           |
//   | HAS_INPUT_QUEUE  | 1       | 1 = input queue built (v2 rule, LSU may ignore input_queue_full);               |
//   |                  |         | 0 = no queue: the LSU MUST NOT send while input_queue_full = 1 (= port stop);   |
//   |                  |         |     the request pins then go to the port pins with no logic                     |
//   | HAS_VPU          | 0       | passed to l1d_top; 0 = port 3 not built as a VPU port (it is unused here)        |
//   | all others       |         | passed unchanged to l1d_top                                                     |
//
// EXAMPLE (default configuration, both ports free, queue empty)
//   cycle 0: store idx 1 to 0x40   -> to port 2 straight away (queue stays empty)
//   cycle 1: load  idx 2 to 0x40   -> to port 0 straight away; the cache keeps the order store -> load on the same line,
//                                     so the load reads the stored bytes
//   cycle 5: port 2 has the store acknowledge, port 0 the load data in the same cycle -> the pins show the one whose turn it
//            is, the other one waits on its port (held, nothing lost) and is shown after the first one is taken.
//   If port 2 had stop_2 = 1 in cycle 0, the store would go into the queue, and the load of cycle 1 would queue BEHIND it
//   (in-order issue), so the store still reaches the cache first.
//
// RULES FOR THE LSU
//   - at most 2**IDX_BITS requests in flight (an index is reused only after its answer), so the queue never overflows;
//   - an SC is sent only after its LR was answered; at most one LR in flight (l1d_top port rules);
//   - an AMO is ONE request (see the table); the v2 two-step AMO (AMO read + AMO write) is NOT supported.
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

module l1d_top_single_port #(
    // ---- address and organisation (passed to l1d_top) ----
    parameter ADDR_W           = `L1D_ADDR_W,             // default 64: address bits
    parameter LINE_BYTES       = `L1D_LINE_BYTES,         // default 64: bytes per cache line
    parameter NUM_BANKS        = `L1D_NUM_BANKS,          // default 4: banks
    parameter SETS             = `L1D_SETS,               // default 64: sets per bank
    parameter WAYS             = `L1D_WAYS,               // default 4: ways per set
    // ---- LSU port ----
    parameter WRITE_DATA_WIDTH = `L1D_WRITE_DATA_WIDTH,   // default 64: store data / load data bits
    parameter MEM_SIZE_BITS    = `L1D_MEM_SIZE_BITS,      // default 2: access size code bits
    parameter MASTER_ID_BITS   = `L1D_MASTER_ID_BITS,     // default 2: master id bits
    parameter IDX_BITS         = `L1D_IDX_BITS,           // default 3: ROB index bits
    parameter INQ_DEPTH        = (1 << IDX_BITS),         // default 8: input queue places, one per ROB index (as v2 u_inq), at least 2
    parameter HAS_INPUT_QUEUE  = 1,                       // default 1: 1 = input queue built (v2 rule); 0 = LSU obeys input_queue_full
    parameter PORT_QUEUE_DEPTH = `L1D_PORT_QUEUE_DEPTH,   // default 2: skid places per request port inside l1d_top
    // ---- per-bank resources (passed to l1d_top) ----
    parameter LSU_BUF_DEPTH    = `L1D_LSU_BUF_DEPTH,      // default from l1d_defines.vh
    parameter TRK_ENTRIES      = `L1D_TRK_ENTRIES,        // reserved
    parameter TRK_REFILL_RSV   = `L1D_TRK_REFILL_RSV,     // reserved
    parameter MSHR_ENTRIES     = `L1D_MSHR_ENTRIES,       // default from l1d_defines.vh: MSHR entries per bank
    parameter MSHR_NC_ENTRIES  = `L1D_MSHR_NC_ENTRIES,    // default from l1d_defines.vh: non-cacheable entries per bank
    parameter VC_SLOTS         = `L1D_VC_SLOTS,           // default from l1d_defines.vh: victim cache slots per bank
    parameter RESP_FIFO_DEPTH  = `L1D_RESP_FIFO_DEPTH,    // default from l1d_defines.vh: answer FIFO places per bank and port
    parameter REFILL_SKIP      = `L1D_REFILL_SKIP,        // default from l1d_defines.vh
    // ---- FIFO stage, snoops and atomics (passed to l1d_top) ----
    parameter SNP_BUF_DEPTH    = `L1D_SNP_BUF_DEPTH,      // default from l1d_defines.vh
    parameter SNP_RESP_DEPTH   = `L1D_SNP_RESP_DEPTH,     // default from l1d_defines.vh
    parameter PF_FIFO_DEPTH    = `L1D_PF_FIFO_DEPTH,      // default from l1d_defines.vh
    parameter SNP_STARVE       = `L1D_SNP_STARVE,         // default from l1d_defines.vh
    parameter LOCK_TIMEOUT     = `L1D_LOCK_TIMEOUT,       // default from l1d_defines.vh
    parameter RSV_CLR_ON_EVICT = `L1D_RSV_CLR_ON_EVICT,   // default from l1d_defines.vh
    parameter HAS_SNOOP        = `L1D_HAS_SNOOP,          // default from l1d_defines.vh
    parameter HAS_AMO          = `L1D_HAS_AMO,            // default from l1d_defines.vh
    // ---- beat and bank-request address width inside the cache (passed to l1d_top) ----
    parameter AXI_DATA_W       = `L1D_AXI_DATA_W,         // default 128: beat width between the L2 port and the banks
    parameter AXI_ADDR_W       = `L1D_AXI_ADDR_W,         // default 64: address width of the bank requests
    // ---- L2 packet port (passed to l1d_top) ----
    parameter L2_ADDR_W        = 56,                      // default 56: packet address bits
    parameter L2_MID_W         = 2,                       // default 2: packet master id bits
    parameter L2_TID_W         = 6,                       // default 6: packet transaction id bits
    parameter L2_MASTER_ID     = 0,                       // default 0: master id this cache puts in every request
    parameter L2_PKT_W         = 1 + L2_ADDR_W + 1 + L2_MID_W + L2_TID_W + 1 + LINE_BYTES + LINE_BYTES*8,  // default 643 (derived, do not override)
    // ---- front end and feature switches (passed to l1d_top) ----
    parameter LSU_PORTS        = `L1D_LSU_PORTS,          // default 4: must be 4 (named ports of l1d_top)
    parameter HAS_MISS         = `L1D_HAS_MISS,           // default from l1d_defines.vh
    parameter HAS_VC           = `L1D_HAS_VC,             // default from l1d_defines.vh
    parameter HAS_NC           = `L1D_HAS_NC,             // default from l1d_defines.vh
    parameter HAS_VPU          = 0,                       // default 0: port 3 is not used by a single-port core
    // ---- cache_v6: tracker, prefetch and ECC settings (passed to l1d_top) ----
    parameter PC_W             = `L1D_PC_W,               // default 64: instruction PC bits (the single-port core gives no PC: 0)
    parameter TRK_ALLOC_MAX    = `L1D_TRK_ALLOC_MAX,      // default 4: arrivals a bank tracker allocates in one cycle
    parameter PF_MSHR_MAX      = `L1D_PF_MSHR_MAX,        // default 2: MSHR entries per bank a prefetch may hold
    parameter HAS_ECC          = `L1D_HAS_ECC,            // default 1: SECDED on the arrays
    parameter HAS_PF           = `L1D_HAS_PF              // default 1: the prefetch unit is built (trained without PCs: stream only, O-8)
) (
    input  wire                         clk,                        // clock: all registers change on its rising edge
    input  wire                         rst,                        // synchronous reset, active high

    // ---- cache maintenance (v2 names) ----
    input  wire                         flush_request,              // write back all dirty lines, then invalidate all
    input  wire                         clean_request,              // write back all dirty lines, keep them valid
    output wire                         flush_complete,             // one-cycle pulse: a flush has finished in every bank
    output wire                         clean_complete,             // one-cycle pulse: a clean has finished in every bank

    // ---- LSU request (v2 names; taken in every cycle cpu_request_valid = 1 when HAS_INPUT_QUEUE = 1) ----
    input  wire                         cpu_request_valid,          // a request from the CPU / LSU this cycle
    input  wire                         cpu_request_cacheable,      // 1 = cacheable, 0 = non-cacheable
    input  wire                         cpu_request_is_write,       // 1 = write (store, SC)
    input  wire [MEM_SIZE_BITS-1:0]     cpu_request_size,           // 0 = 8, 1 = 16, 2 = 32, 3 = 64 bits; MEM_SIZE_BITS = 2 bits
    input  wire [MASTER_ID_BITS-1:0]    cpu_request_master_id,      // 0 = LSU, 1 = MMU; MASTER_ID_BITS = 2 bits
    input  wire [IDX_BITS-1:0]          cpu_request_rob_index,      // ROB index, returned with the response; IDX_BITS = 3 bits
    input  wire [ADDR_W-1:0]            cpu_request_addr,           // byte address; ADDR_W = 64 bits
    input  wire [WRITE_DATA_WIDTH-1:0]  cpu_request_store_data,     // store data / SC data / AMO operand in the low bits; 64 bits
    input  wire                         cpu_request_is_atomic,      // atomic request (LR / SC or AMO)
    input  wire                         cpu_request_atomic_is_amo,  // with is_atomic: 0 = LR / SC, 1 = AMO (one request)
    input  wire [3:0]                   cpu_request_amo_op,         // NEW against v2: which AMO (AMO_SWAP .. AMO_MAXU, l1d_encodings.vh), 4 bits
    output wire                         input_queue_full,           // the input queue is full (HAS_INPUT_QUEUE = 0: the target port's stop)

    // ---- LSU response (v2 names; held until lsu_response_read_enable = 1) ----
    input  wire                         lsu_response_read_enable,   // the LSU takes the current response
    output wire                         lsu_response_valid,         // a response is shown to the LSU
    output wire                         lsu_response_is_write_ack,  // 1 = store / SC acknowledge, 0 = load / LR data or AMO old value
    output wire [MEM_SIZE_BITS-1:0]     lsu_response_size,          // size of the answered request, 2 bits
    output wire [MASTER_ID_BITS-1:0]    lsu_response_master_id,     // master id of the answered request, 2 bits
    output wire [IDX_BITS-1:0]          lsu_response_rob_index,     // ROB index of the answered request, 3 bits
    output wire [WRITE_DATA_WIDTH-1:0]  lsu_response_data,          // load data from bit 0 (no sign extension), SC result, AMO old value; 64 bits

    // ---- snoop request / answer (straight through) ----
    input  wire                         snoop_request_valid,        // a snoop request from outside
    output wire                         snoop_request_ready,        // the cache can take it
    input  wire [ADDR_W-1:0]            snoop_request_addr,         // any address inside the line, 64 bits
    input  wire                         snoop_request_invalidate,   // 1 = invalidate, 0 = read
    output wire                         snoop_response_valid,       // a snoop answer is shown
    input  wire                         snoop_response_ready,       // the receiver takes the answer
    output wire [ADDR_W-1:0]            snoop_response_addr,        // line address of the answered snoop, 64 bits
    output wire                         snoop_response_present,     // the cache held the line
    output wire                         snoop_response_dirty,       // snoop_response_data is a dirty line to write to memory
    output wire [LINE_BYTES*8-1:0]      snoop_response_data,        // the line, LINE_BYTES*8 = 512 bits

    // ---- status ----
    output wire                         l2_error_sticky,            // sticky: an L2 answer came with l2_resp_err = 1 (v2: axi_error_sticky)
    output wire                         amo_lock_timeout_sticky,    // sticky: an AMO lock timed out
    output wire                         ecc_correctable_pulse,      // cache_v6: one cycle per ECC scrub written
    output wire                         ecc_uncorrectable_sticky,   // cache_v6: sticky: an uncorrectable ECC error was seen

    // ---- L2 packet port (straight through; packet layout in rtl/l2/l1d_l2_port.v) ----
    output wire                         l2_req_valid,               // a request packet is offered to the L2
    input  wire                         l2_req_ready,               // the L2 takes it
    output wire [L2_PKT_W-1:0]          l2_req_pkt,                 // request packet, L2_PKT_W = 643 bits
    input  wire                         l2_resp_valid,              // a response packet is offered by the L2
    output wire                         l2_resp_ready,              // the cache takes it
    input  wire [L2_PKT_W-1:0]          l2_resp_pkt,                // response packet, 643 bits
    input  wire                         l2_resp_err                 // the response is an error (sets l2_error_sticky)
);

    // ======================================================================
    // REQUEST PACKET OF THE INPUT QUEUE: the single-port request fields, packed in one word
    //   | goes_to_store_port | is_atomic | is_amo | amo_op | is_write | cacheable | size | master_id | rob_index | addr | data |
    // goes_to_store_port is decided once at the pins (store, SC or AMO -> port 2; load or LR -> port 0) and stored with the
    // request, so the issue logic does not decode it again from the queue output.
    // ======================================================================
    // field positions inside the packet, counted from bit 0 (data is the lowest field)
    localparam DATA_LSB      = 0;                                   // data field starts at bit 0
    localparam ADDR_LSB      = DATA_LSB + WRITE_DATA_WIDTH;         // = 64
    localparam ROB_LSB       = ADDR_LSB + ADDR_W;                   // = 128
    localparam MASTER_LSB    = ROB_LSB + IDX_BITS;                  // = 131
    localparam SIZE_LSB      = MASTER_LSB + MASTER_ID_BITS;         // = 133
    localparam CACHEABLE_BIT = SIZE_LSB + MEM_SIZE_BITS;            // = 135
    localparam WRITE_BIT     = CACHEABLE_BIT + 1;                   // = 136
    localparam AMO_OP_LSB    = WRITE_BIT + 1;                       // = 137 (4 bits, 137..140)
    localparam AMO_BIT       = AMO_OP_LSB + 4;                      // = 141
    localparam ATOMIC_BIT    = AMO_BIT + 1;                         // = 142
    localparam STORE_PORT_BIT = ATOMIC_BIT + 1;                     // = 143: the top bit, so a packet is STORE_PORT_BIT + 1 = 144 bits

    // ---- the request on the pins, packed ----
    // a request goes to the store port (2) when it writes (store, SC) or is an AMO; a load or LR goes to load port 0
    wire pin_goes_to_store_port = cpu_request_is_write || (cpu_request_is_atomic && cpu_request_atomic_is_amo);
    wire [STORE_PORT_BIT:0] pin_request = {pin_goes_to_store_port, cpu_request_is_atomic, cpu_request_atomic_is_amo,
                                           cpu_request_amo_op, cpu_request_is_write, cpu_request_cacheable,
                                           cpu_request_size, cpu_request_master_id, cpu_request_rob_index,
                                           cpu_request_addr, cpu_request_store_data};   // STORE_PORT_BIT + 1 = 144 bits

    // ---- port stops from the cache (registered inside l1d_top) ----
    wire load_port_stop;                                            // port 0 must not send this cycle
    wire store_port_stop;                                           // port 2 must not send this cycle

    // ---- the request handed to the cache this cycle (one at most, to port 0 or port 2) ----
    wire                    issue_valid;                            // a request goes to the cache this cycle
    wire [STORE_PORT_BIT:0] issue_request;                          // the request that goes, 144 bits

    generate
        if (HAS_INPUT_QUEUE != 0) begin : g_input_queue
            // ==================================================================
            // INPUT QUEUE (as the v2 u_inq, but with a bypass): the oldest waiting request goes first; with an empty queue the
            // request on the pins goes in the same cycle. A request goes only when its own port has stop = 0; otherwise it
            // (and everything behind it) waits, so the requests reach the cache in program order.
            // ==================================================================
            wire                    queue_empty;                    // no request waits
            wire                    queue_full;                     // every place used (cannot happen under the ROB rule)
            wire [STORE_PORT_BIT:0] queue_front;                    // the oldest waiting request, 144 bits
            wire                    queue_push;                     // put the pin request into the queue this cycle
            wire                    queue_pop;                      // the oldest waiting request goes to the cache this cycle

            // candidate = the request that may go this cycle: the queue front, or the pin request when nothing waits
            wire                    candidate_valid   = !queue_empty || cpu_request_valid;
            wire [STORE_PORT_BIT:0] candidate_request = queue_empty ? pin_request : queue_front;
            wire                    candidate_port_stopped = candidate_request[STORE_PORT_BIT] ? store_port_stop : load_port_stop;

            assign issue_valid   = candidate_valid && !candidate_port_stopped;
            assign issue_request = candidate_request;
            assign queue_pop     = !queue_empty && !candidate_port_stopped;           // the front goes
            // the pin request is queued unless it went straight through (queue empty and its port free)
            assign queue_push    = cpu_request_valid && !(queue_empty && !candidate_port_stopped);

            l1d_fifo #(.WIDTH(STORE_PORT_BIT + 1), .DEPTH(INQ_DEPTH)) u_input_queue (   // WIDTH = 144 bits, DEPTH = 8
                .clk        (clk),
                .rst        (rst),
                .push       (queue_push),
                .push_data  (pin_request),
                .pop        (queue_pop),
                .front_data (queue_front),
                .empty      (queue_empty),
                .full       (queue_full),
                .used_slots (),
                .free_slots ()
            );
            assign input_queue_full = queue_full;

`ifndef SYNTHESIS
            // the LSU broke the ROB rule: a request arrived while the queue was full (it is lost)
            always @(posedge clk)
                if (!rst && cpu_request_valid && queue_full)
                    $display("[%0t] L1D SINGLE PORT  ERROR request idx %0d lost: input queue full (more than %0d requests in flight)",
                             $time, cpu_request_rob_index, INQ_DEPTH);
`endif
        end else begin : g_no_input_queue
            // ==================================================================
            // NO INPUT QUEUE: the pins go to the port with no logic. The LSU does not know in advance which port its next
            // request uses, so input_queue_full is the OR of both port stops (as in dummy_l2/l1d_top_axi.v).
            // ==================================================================
            assign issue_valid      = cpu_request_valid;
            assign issue_request    = pin_request;
            assign input_queue_full = load_port_stop || store_port_stop;
`ifndef SYNTHESIS
            always @(posedge clk)
                if (!rst && cpu_request_valid && (pin_goes_to_store_port ? store_port_stop : load_port_stop))
                    $display("[%0t] L1D SINGLE PORT  ERROR request idx %0d sent while its port is stopped (HAS_INPUT_QUEUE = 0)",
                             $time, cpu_request_rob_index);
`endif
        end
    endgenerate

    // ---- the issued request, split into the fields of the typed ports ----
    wire                        issue_to_store_port = issue_request[STORE_PORT_BIT];                          // 1 = port 2, 0 = port 0
    wire                        issue_is_atomic     = issue_request[ATOMIC_BIT];                              // LR / SC / AMO
    wire                        issue_is_amo        = issue_request[AMO_BIT];                                 // with is_atomic: AMO
    wire [3:0]                  issue_amo_op        = issue_request[AMO_OP_LSB +: 4];                         // AMO operation, 4 bits
    wire                        issue_cacheable     = issue_request[CACHEABLE_BIT];                           // 1 = cacheable
    wire [MEM_SIZE_BITS-1:0]    issue_size          = issue_request[SIZE_LSB +: MEM_SIZE_BITS];               // 2 bits
    wire [MASTER_ID_BITS-1:0]   issue_master_id     = issue_request[MASTER_LSB +: MASTER_ID_BITS];            // 2 bits
    wire [IDX_BITS-1:0]         issue_rob_index     = issue_request[ROB_LSB +: IDX_BITS];                     // 3 bits
    wire [ADDR_W-1:0]           issue_addr          = issue_request[ADDR_LSB +: ADDR_W];                      // 64 bits
    wire [WRITE_DATA_WIDTH-1:0] issue_data          = issue_request[DATA_LSB +: WRITE_DATA_WIDTH];            // 64 bits

    wire load_port_valid  = issue_valid && !issue_to_store_port;                   // request on port 0 (load, LR)
    wire store_port_valid = issue_valid &&  issue_to_store_port;                   // request on port 2 (store, SC, AMO)

    // ======================================================================
    // ANSWER MERGE: port 0 and port 2 answers -> the single lsu_response_* port.
    //   - an answer that is shown and not taken stays shown (shown_held_q / shown_is_store_q), even if the other port gets one;
    //   - with no held answer and both ports ready, store_turn_q picks; after every taken answer the turn goes to the OTHER
    //     port, so a stream of load answers cannot starve the store port (and the other way round);
    //   - the port that is not shown keeps its answer (its read enable stays 0): nothing is lost.
    // ======================================================================
    wire                        load_answer_valid;                  // port 0 shows an answer
    wire [MEM_SIZE_BITS-1:0]    load_answer_size;                   // 2 bits
    wire [MASTER_ID_BITS-1:0]   load_answer_master_id;              // 2 bits
    wire [IDX_BITS-1:0]         load_answer_rob_index;              // 3 bits
    wire [WRITE_DATA_WIDTH-1:0] load_answer_data;                   // 64 bits
    wire                        store_answer_valid;                 // port 2 shows an answer
    wire                        store_answer_is_write_ack;          // 1 = store / SC acknowledge, 0 = AMO old value
    wire [MEM_SIZE_BITS-1:0]    store_answer_size;                  // 2 bits
    wire [MASTER_ID_BITS-1:0]   store_answer_master_id;             // 2 bits
    wire [IDX_BITS-1:0]         store_answer_rob_index;             // 3 bits
    wire [WRITE_DATA_WIDTH-1:0] store_answer_data;                  // 64 bits

    reg  store_turn_q;                                              // 1 = port 2 goes first when both ports have an answer
    reg  shown_held_q;                                              // an answer was shown last cycle and not taken
    reg  shown_is_store_q;                                          // ... and it came from port 2

    // which port the pins show this cycle
    wire show_store = shown_held_q ? shown_is_store_q
                                   : (store_answer_valid && (!load_answer_valid || store_turn_q));
    assign lsu_response_valid        = show_store ? store_answer_valid        : load_answer_valid;
    assign lsu_response_is_write_ack = show_store ? store_answer_is_write_ack : 1'b0;      // port 0 never acknowledges a write
    assign lsu_response_size         = show_store ? store_answer_size         : load_answer_size;
    assign lsu_response_master_id    = show_store ? store_answer_master_id    : load_answer_master_id;
    assign lsu_response_rob_index    = show_store ? store_answer_rob_index    : load_answer_rob_index;
    assign lsu_response_data         = show_store ? store_answer_data         : load_answer_data;

    wire answer_taken      = lsu_response_valid && lsu_response_read_enable;   // the LSU takes the shown answer this cycle
    wire take_load_answer  = answer_taken && !show_store;                        // read enable of port 0
    wire take_store_answer = answer_taken &&  show_store;                        // read enable of port 2

    always @(posedge clk) begin
        if (rst) begin
            store_turn_q     <= 1'b0;
            shown_held_q     <= 1'b0;
            shown_is_store_q <= 1'b0;
        end else begin
            // the turn goes to the other port after every taken answer
            if (answer_taken)
                store_turn_q <= !show_store;
            // remember a shown, not taken answer so it stays on the pins next cycle
            shown_held_q     <= lsu_response_valid && !lsu_response_read_enable;
            shown_is_store_q <= show_store;
        end
    end

    // ======================================================================
    // THE D-CACHE (cache_v5_port_managed l1d_top): port 0 = loads / LR, port 2 = stores / SC / AMO, ports 1 and 3 idle
    // (valid 0; their read enables 1, they never answer).
    // ======================================================================
`ifndef SYNTHESIS
    initial if (LSU_PORTS != 4)
        $display("L1D SINGLE PORT  ERROR LSU_PORTS = %0d: l1d_top has the named ports _0 .. _3 and needs 4", LSU_PORTS);
`endif

    // cache_v6: the single-port core has no PC pin and no prefetch switch: PCs are 0, prefetching is on
    wire [PC_W-1:0] pc_none;                                        // PC given to the cache, 64 bits of 0
    wire            prefetch_on;                                    // the prefetch switch, tied on
    assign pc_none     = {PC_W{1'b0}};
    assign prefetch_on = 1'b1;

    l1d_top #(
        .ADDR_W(ADDR_W), .LINE_BYTES(LINE_BYTES), .NUM_BANKS(NUM_BANKS), .SETS(SETS), .WAYS(WAYS),
        .WRITE_DATA_WIDTH(WRITE_DATA_WIDTH), .MEM_SIZE_BITS(MEM_SIZE_BITS), .MASTER_ID_BITS(MASTER_ID_BITS),
        .IDX_BITS(IDX_BITS), .PORT_QUEUE_DEPTH(PORT_QUEUE_DEPTH), .LSU_BUF_DEPTH(LSU_BUF_DEPTH), .TRK_ENTRIES(TRK_ENTRIES),
        .TRK_REFILL_RSV(TRK_REFILL_RSV), .MSHR_ENTRIES(MSHR_ENTRIES), .MSHR_NC_ENTRIES(MSHR_NC_ENTRIES),
        .VC_SLOTS(VC_SLOTS), .RESP_FIFO_DEPTH(RESP_FIFO_DEPTH), .REFILL_SKIP(REFILL_SKIP),
        .SNP_BUF_DEPTH(SNP_BUF_DEPTH), .SNP_RESP_DEPTH(SNP_RESP_DEPTH), .PF_FIFO_DEPTH(PF_FIFO_DEPTH),
        .SNP_STARVE(SNP_STARVE), .LOCK_TIMEOUT(LOCK_TIMEOUT), .RSV_CLR_ON_EVICT(RSV_CLR_ON_EVICT),
        .HAS_SNOOP(HAS_SNOOP), .HAS_AMO(HAS_AMO), .AXI_DATA_W(AXI_DATA_W), .AXI_ADDR_W(AXI_ADDR_W),
        .L2_ADDR_W(L2_ADDR_W), .L2_MID_W(L2_MID_W), .L2_TID_W(L2_TID_W), .L2_MASTER_ID(L2_MASTER_ID),
        .LSU_PORTS(LSU_PORTS), .HAS_MISS(HAS_MISS), .HAS_VC(HAS_VC), .HAS_NC(HAS_NC), .HAS_VPU(HAS_VPU),
        .PC_W(PC_W), .TRK_ALLOC_MAX(TRK_ALLOC_MAX), .PF_MSHR_MAX(PF_MSHR_MAX), .HAS_ECC(HAS_ECC), .HAS_PF(HAS_PF)
    ) u_dcache (
        .clk(clk), .rst(rst),
        .flush(flush_request), .clean(clean_request), .flush_complete(flush_complete), .clean_complete(clean_complete),
        // ---- cache_v6: PCs (none), prefetch switch (on), ECC status ----
        .pc_cpu_0(pc_none), .pc_cpu_1(pc_none), .pc_cpu_2(pc_none), .pf_enable(prefetch_on),
        .ecc_correctable_pulse(ecc_correctable_pulse), .ecc_uncorrectable_sticky(ecc_uncorrectable_sticky),
        // ---- port 0: load / LR ----
        .valid_cpu_0          (load_port_valid),
        .cacheable_0          (issue_cacheable),
        .mem_size_cpu_0       (issue_size),
        .master_id_from_cpu_0 (issue_master_id),
        .idx_from_cpu_0       (issue_rob_index),
        .PC_from_cpu_0        (issue_addr),
        .is_lr_cpu_0          (issue_is_atomic),                    // on port 0 an atomic is always an LR
        .stop_0               (load_port_stop),
        // ---- port 1: not used ----
        .valid_cpu_1          (1'b0),
        .cacheable_1          (1'b1),
        .mem_size_cpu_1       ({MEM_SIZE_BITS{1'b0}}),
        .master_id_from_cpu_1 ({MASTER_ID_BITS{1'b0}}),
        .idx_from_cpu_1       ({IDX_BITS{1'b0}}),
        .PC_from_cpu_1        ({ADDR_W{1'b0}}),
        .is_lr_cpu_1          (1'b0),
        .stop_1               (),
        // ---- port 2: store / SC / AMO ----
        .valid_cpu_2          (store_port_valid),
        .cacheable_2          (issue_cacheable),
        .mem_size_cpu_2       (issue_size),
        .master_id_from_cpu_2 (issue_master_id),
        .idx_from_cpu_2       (issue_rob_index),
        .PC_from_cpu_2        (issue_addr),
        .data_from_cpu_2      (issue_data),
        .is_sc_cpu_2          (issue_is_atomic && !issue_is_amo),   // SC = atomic write that is not an AMO
        .is_amo_cpu_2         (issue_is_atomic &&  issue_is_amo),   // AMO = one request, computed in the cache
        .amo_op_cpu_2         (issue_amo_op),
        .stop_2               (store_port_stop),
        // ---- port 3: VPU, not used ----
        .valid_cpu_3          (1'b0),
        .rd_wr_cpu_3          (1'b0),
        .cacheable_3          (1'b1),
        .master_id_from_cpu_3 ({MASTER_ID_BITS{1'b0}}),
        .idx_from_cpu_3       ({IDX_BITS{1'b0}}),
        .PC_from_cpu_3        ({ADDR_W{1'b0}}),
        .data_from_cpu_3      ({(LINE_BYTES*8){1'b0}}),
        .stop_3               (),
        // ---- answers: port 0 and port 2 go to the merge above; ports 1 and 3 never answer ----
        .read_enable_from_lsu_0             (take_load_answer),
        .final_r_cache_hit_from_scheduler_0 (load_answer_valid),
        .final_r_mem_size_from_scheduler_0  (load_answer_size),
        .final_r_master_id_from_scheduler_0 (load_answer_master_id),
        .final_r_idx_from_scheduler_0       (load_answer_rob_index),
        .final_r_data_from_scheduler_0      (load_answer_data),
        .read_enable_from_lsu_1             (1'b1),
        .final_r_cache_hit_from_scheduler_1 (),
        .final_r_mem_size_from_scheduler_1  (),
        .final_r_master_id_from_scheduler_1 (),
        .final_r_idx_from_scheduler_1       (),
        .final_r_data_from_scheduler_1      (),
        .read_enable_from_lsu_2             (take_store_answer),
        .final_r_cache_hit_from_scheduler_2 (store_answer_valid),
        .final_r_hit_rd_wr_from_scheduler_2 (store_answer_is_write_ack),
        .final_r_mem_size_from_scheduler_2  (store_answer_size),
        .final_r_master_id_from_scheduler_2 (store_answer_master_id),
        .final_r_idx_from_scheduler_2       (store_answer_rob_index),
        .final_r_data_from_scheduler_2      (store_answer_data),
        .read_enable_from_lsu_3             (1'b1),
        .final_r_cache_hit_from_scheduler_3 (),
        .final_r_hit_rd_wr_from_scheduler_3 (),
        .final_r_master_id_from_scheduler_3 (),
        .final_r_idx_from_scheduler_3       (),
        .final_r_data_from_scheduler_3      (),
        // ---- snoops (straight through) ----
        .snoop_request_valid     (snoop_request_valid),
        .snoop_request_ready     (snoop_request_ready),
        .snoop_request_addr      (snoop_request_addr),
        .snoop_request_invalidate(snoop_request_invalidate),
        .snoop_response_valid    (snoop_response_valid),
        .snoop_response_ready    (snoop_response_ready),
        .snoop_response_addr     (snoop_response_addr),
        .snoop_response_present  (snoop_response_present),
        .snoop_response_dirty    (snoop_response_dirty),
        .snoop_response_data     (snoop_response_data),
        // ---- status ----
        .mem_err                 (l2_error_sticky),
        .amo_err                 (amo_lock_timeout_sticky),
        // ---- L2 packet port (straight through) ----
        .l2_req_valid            (l2_req_valid),
        .l2_req_ready            (l2_req_ready),
        .l2_req_pkt              (l2_req_pkt),
        .l2_resp_valid           (l2_resp_valid),
        .l2_resp_ready           (l2_resp_ready),
        .l2_resp_pkt             (l2_resp_pkt),
        .l2_resp_err             (l2_resp_err)
    );

endmodule

`default_nettype wire
