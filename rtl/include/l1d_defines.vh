// ============================================================================
// l1d_defines.vh  -  the default value of every cache_v2 setting
// ============================================================================
// IN SIMPLE WORDS
//   This is the "settings sheet" of the cache. Sizes (how many sets, ways, banks),
//   queue depths and feature switches are written here ONCE. Every Verilog module
//   reads its defaults from here, so changing one number here changes the whole
//   design. A testbench can still override any of them on l1d_top alone (for example
//   the TINY test configuration), because l1d_top passes the real values down.
//
// WHAT IS A `define?  (question from an earlier session)
//   `define NAME value   makes NAME a text macro. Wherever the compiler later sees
//   `NAME it pastes `value`. It is a copy-and-paste done BEFORE the real compile.
//   Example:  parameter SETS = `L1D_SETS   becomes   parameter SETS = 64
//
// WHAT IS THE INCLUDE GUARD?  (question from an earlier session)
//   A header file can be included by many modules. The three lines
//       `ifndef L1D_DEFINES_VH      "if the name L1D_DEFINES_VH is NOT defined yet ..."
//       `define L1D_DEFINES_VH      "... define it now (so the next include is skipped) ..."
//       ...the real content...
//       `endif                      "... end of the conditional part"
//   make the file be read only ONCE per compile, even when ten modules say
//   `include "l1d_defines.vh". Without the guard every `define would be repeated
//   (a warning). The other two include files (l1d_fields.vh, l1d_encodings.vh) have
//   NO guard on purpose: they declare localparams, which must be repeated INSIDE
//   every module that uses them.
//
// THE TWO BIG MACROS AT THE END OF THE FILE
//   `L1D_PARAMS       the whole parameter list. Every module below l1d_top is written as
//                         module l1d_xxx #(`L1D_PARAMS) ( ... ports ... );
//                     so every module knows ADDR_W, LINE_BYTES, SETS, WAYS ... without
//                     retyping them. (Verilog-2005 has no "package", so a macro is used.)
//   `L1D_PARAMS_PASS  the matching connection list  .ADDR_W(ADDR_W), .SETS(SETS) ...
//                     used when a module is placed inside another:
//                         l1d_xxx #(`L1D_PARAMS_PASS) u_xxx ( ... );
//                     so one change at the top reaches every level of the hierarchy.
//
// SEE ALSO
//   l1d_fields.vh     widths and bit positions DERIVED from these settings
//   l1d_encodings.vh  named constants (states, kinds, codes) shared by all modules
// ============================================================================

// ---- include guard (explained in the header): read this file only once ----
`ifndef L1D_DEFINES_VH
`define L1D_DEFINES_VH

// ---- Address and line ----
// A 64-bit address is split like this (see the picture in l1d_fields.vh):
//     | tag | set index | bank | byte offset |
`define L1D_ADDR_W            64                          // address width in bits (the cpu_request_addr input of l1d_top)
`define L1D_LINE_BYTES        64      // cache line size in bytes

// ---- Organisation ----
// 4 banks x 64 sets x 4 ways x 64 bytes = 64 KB. The bank is chosen by the address bits just
// above the 6 line-offset bits (address[7:6]), so neighbouring lines go to different banks.
`define L1D_NUM_BANKS         4       // independent banks, selected by the bits above the line offset
`define L1D_SETS              64      // sets per bank
`define L1D_WAYS              4       // ways per set

// ---- LSU (load/store unit) port ----
`define L1D_WRITE_DATA_WIDTH  64                          // width of cpu_request_store_data and lsu_response_data (load data comes back from bit 0, no sign extension)
`define L1D_MEM_SIZE_BITS     2                           // cpu_request_size: 0 = 8, 1 = 16, 2 = 32, 3 = 64 bits
`define L1D_MASTER_ID_BITS    2                           // cpu_request_master_id: 0 = LSU, 1 = MMU
`define L1D_IDX_BITS          3                           // cpu_request_rob_index (the ROB index): returned with the response so the CPU can match it

// ---- Per-bank queues and tracking ----
// seat = one waiting request in the LSU Request Buffer; MSHR = Miss Status Holding Register (one line being
// fetched or swapped back); VC = victim cache; NC = non-cacheable.
`define L1D_LSU_BUF_DEPTH     8                           // UNUSED since cache_v6 (kept for the shared parameter list); the tracker depth is L1D_TRK_ENTRIES
`define L1D_TRK_ENTRIES       8                           // cache_v6: request tracker entries PER BANK (l1d_req_tracker): a request holds one from arrival to completion
`define L1D_TRK_ALLOC_MAX     4                           // cache_v6: arrivals a bank tracker allocates in ONE cycle (4 = one per port; 2 = shorter free-pick path, O-2)
`define L1D_TRK_REFILL_RSV    2                           // UNUSED in cache_v2 (kept from the Rev K spec)
`define L1D_PC_W              64                          // cache_v6: instruction PC of a request (pc_cpu_N pins, kept in the tracker entry for prefetch training)
`define L1D_MSHR_ENTRIES      8                           // cacheable MSHR entries: one per line being fetched from memory or swapped back from the victim cache
`define L1D_MSHR_NC_ENTRIES   2                           // places in the queue of non-cacheable accesses (done one at a time, in order)
`define L1D_VC_SLOTS          8                           // victim-cache slots: lines recently evicted from the cache, clean or dirty
`define L1D_RESP_FIFO_DEPTH   4                           // per-bank response FIFO in l1d_resp_merge; its free places are the bank's response credit
`define L1D_REFILL_SKIP       2                           // after a refill is granted, Priority Select grants nothing for this many cycles (the arrays are busy)

// ---- FIFO stage, snoops and atomics (new in v2) ----
`define L1D_SNP_BUF_DEPTH     2                           // entries of the snoop buffer of each bank (served oldest first PER LINE, see l1d_snoop_buf.v)
`define L1D_SNP_RESP_DEPTH    2       // snoop response FIFO per bank in l1d_snoop_ctrl
`define L1D_PF_FIFO_DEPTH     4       // UNUSED since lean S0: one prefetch register per bank
`define L1D_PF_MSHR_MAX       2       // cache_v6: MSHR entries per bank a prefetch may hold at one time (the rest stay for demand misses)
`define L1D_PF_HOLD_MAX       2       // cache_v6: cycles a prefetcher's request may wait for its bank before it is dropped (SH-1)
`define L1D_SNP_STARVE        8       // snoop grants a waiting refill token lets pass before it wins
`define L1D_LOCK_TIMEOUT      1024    // cycles an AMO lock may wait for its AMO write
`define L1D_RSV_CLR_ON_EVICT  1       // 1 = evicting the reserved line clears the LR reservation (needed with snoops)

// AXI4 master
`define L1D_AXI_DATA_W        128     // 64 / 128 / 256 / 512
`define L1D_AXI_ADDR_W        64

// Front end (A2): LSU_PORTS request ports, each with its own response port. Every port can reach every bank:
// l1d_port_front sorts the requests by bank: one skid per port (PORT_QUEUE_DEPTH places) and one l1d_bank_order per bank (lean S0),
// and l1d_top has one l1d_resp_merge per port. 4 = default (was 1 = single port + bank selector before A2).
`define L1D_LSU_PORTS         4       // default 4: request ports (each with its own response port)
`define L1D_PORT_QUEUE_DEPTH  2       // default 2: skid places per request port (one for the late request, BUG-7)

// VPU port (cache_v5 C-20). With HAS_VPU = 1 (l1d_top parameter) the LAST request port (LSU_PORTS-1 = 3) is the VPU port:
// whole-line (512-bit) vector loads and stores, address 64-byte aligned, no byte mask, no atomics. A vector store keeps its
// line data in a small per-bank buffer of L1D_VEC_SLOTS entries until it has written the line.
`ifndef L1D_HAS_VPU
`define L1D_HAS_VPU           1       // default 1: request port LSU_PORTS-1 is the 512-bit VPU port (0 = four 64-bit ports, as in A2)
`endif
`define L1D_VEC_SLOTS         2       // default 2: vector store data places per bank (a vector store waits at the bank door when all are used)

// Feature switches (1 = built). v2 adds snoops and atomics; cache_v6 adds the prefetchers (HAS_PF) and SECDED ECC on the
// data and tag arrays (HAS_ECC). Both may be set from the command line (-D L1D_HAS_ECC=0) for a build without them.
`define L1D_HAS_MISS          1
`define L1D_HAS_VC            1
`define L1D_HAS_NC            1
`define L1D_HAS_SNOOP         1
`define L1D_HAS_AMO           1
`ifndef L1D_HAS_PF
`define L1D_HAS_PF            1       // default 1: stride + stream + SMS prefetchers built (l1d_prefetch_unit), prefetch path in the banks
`endif
`ifndef L1D_HAS_ECC
`define L1D_HAS_ECC           1       // default 1: SECDED (72,64) per 8-byte granule on the data arrays, SECDED (57,50) on the tags
`endif

// ---------------------------------------------------------------------------
// Debug trace (simulation only; never synthesised, off by default)
//   Turn it on with one of:
//     - Vivado: xvlog -d L1D_DEBUG   (GUI: Simulation Settings > Compilation >
//       "Verilog options" > Defines: L1D_DEBUG)
//     - Icarus: iverilog -DL1D_DEBUG
//     - or remove the "//" in front of the `define on the next line
//   Every unit then prints its events with [time], and a unit that has not
//   moved for L1D_DBG_WD cycles prints a STUCK report (repeated every
//   L1D_DBG_WD cycles). Lines start with "L1D" so a log can be filtered.
// ---------------------------------------------------------------------------
// `define L1D_DEBUG
`ifndef L1D_DBG_WD
`define L1D_DBG_WD 500
`endif

// ---------------------------------------------------------------------------
// Shared parameter list. Every cache module below l1d_top declares
//     module l1d_xxx #(`L1D_PARAMS) ( ... );
// and every instantiation passes them on with
//     l1d_xxx #(`L1D_PARAMS_PASS) u_xxx ( ... );
// so one change at the top reaches the whole design. l1d_fields.vh derives all
// widths from this list.
// ---------------------------------------------------------------------------
// A backslash at the end of a line means: the macro continues on the next line.
`define L1D_PARAMS \
    parameter ADDR_W           = `L1D_ADDR_W, \
    parameter LINE_BYTES       = `L1D_LINE_BYTES, \
    parameter NUM_BANKS        = `L1D_NUM_BANKS, \
    parameter SETS             = `L1D_SETS, \
    parameter WAYS             = `L1D_WAYS, \
    parameter WRITE_DATA_WIDTH = `L1D_WRITE_DATA_WIDTH, \
    parameter MEM_SIZE_BITS    = `L1D_MEM_SIZE_BITS, \
    parameter MASTER_ID_BITS   = `L1D_MASTER_ID_BITS, \
    parameter IDX_BITS         = `L1D_IDX_BITS, \
    parameter LSU_BUF_DEPTH    = `L1D_LSU_BUF_DEPTH, \
    parameter TRK_ENTRIES      = `L1D_TRK_ENTRIES, \
    parameter TRK_REFILL_RSV   = `L1D_TRK_REFILL_RSV, \
    parameter MSHR_ENTRIES     = `L1D_MSHR_ENTRIES, \
    parameter MSHR_NC_ENTRIES  = `L1D_MSHR_NC_ENTRIES, \
    parameter VC_SLOTS         = `L1D_VC_SLOTS, \
    parameter RESP_FIFO_DEPTH  = `L1D_RESP_FIFO_DEPTH, \
    parameter REFILL_SKIP      = `L1D_REFILL_SKIP, \
    parameter SNP_BUF_DEPTH    = `L1D_SNP_BUF_DEPTH, \
    parameter SNP_RESP_DEPTH   = `L1D_SNP_RESP_DEPTH, \
    parameter PF_FIFO_DEPTH    = `L1D_PF_FIFO_DEPTH, \
    parameter SNP_STARVE       = `L1D_SNP_STARVE, \
    parameter LOCK_TIMEOUT     = `L1D_LOCK_TIMEOUT, \
    parameter RSV_CLR_ON_EVICT = `L1D_RSV_CLR_ON_EVICT, \
    parameter HAS_SNOOP        = `L1D_HAS_SNOOP, \
    parameter HAS_AMO          = `L1D_HAS_AMO, \
    parameter AXI_DATA_W       = `L1D_AXI_DATA_W, \
    parameter AXI_ADDR_W       = `L1D_AXI_ADDR_W, \
    parameter LSU_PORTS        = `L1D_LSU_PORTS, \
    parameter PC_W             = `L1D_PC_W, \
    parameter TRK_ALLOC_MAX    = `L1D_TRK_ALLOC_MAX, \
    parameter PF_MSHR_MAX      = `L1D_PF_MSHR_MAX, \
    parameter HAS_ECC          = `L1D_HAS_ECC, \
    parameter HAS_PF           = `L1D_HAS_PF

`define L1D_PARAMS_PASS \
    .ADDR_W(ADDR_W), \
    .LINE_BYTES(LINE_BYTES), \
    .NUM_BANKS(NUM_BANKS), \
    .SETS(SETS), \
    .WAYS(WAYS), \
    .WRITE_DATA_WIDTH(WRITE_DATA_WIDTH), \
    .MEM_SIZE_BITS(MEM_SIZE_BITS), \
    .MASTER_ID_BITS(MASTER_ID_BITS), \
    .IDX_BITS(IDX_BITS), \
    .LSU_BUF_DEPTH(LSU_BUF_DEPTH), \
    .TRK_ENTRIES(TRK_ENTRIES), \
    .TRK_REFILL_RSV(TRK_REFILL_RSV), \
    .MSHR_ENTRIES(MSHR_ENTRIES), \
    .MSHR_NC_ENTRIES(MSHR_NC_ENTRIES), \
    .VC_SLOTS(VC_SLOTS), \
    .RESP_FIFO_DEPTH(RESP_FIFO_DEPTH), \
    .REFILL_SKIP(REFILL_SKIP), \
    .SNP_BUF_DEPTH(SNP_BUF_DEPTH), \
    .SNP_RESP_DEPTH(SNP_RESP_DEPTH), \
    .PF_FIFO_DEPTH(PF_FIFO_DEPTH), \
    .SNP_STARVE(SNP_STARVE), \
    .LOCK_TIMEOUT(LOCK_TIMEOUT), \
    .RSV_CLR_ON_EVICT(RSV_CLR_ON_EVICT), \
    .HAS_SNOOP(HAS_SNOOP), \
    .HAS_AMO(HAS_AMO), \
    .AXI_DATA_W(AXI_DATA_W), \
    .AXI_ADDR_W (AXI_ADDR_W), \
    .LSU_PORTS(LSU_PORTS), \
    .PC_W(PC_W), \
    .TRK_ALLOC_MAX(TRK_ALLOC_MAX), \
    .PF_MSHR_MAX(PF_MSHR_MAX), \
    .HAS_ECC(HAS_ECC), \
    .HAS_PF(HAS_PF)

`endif // L1D_DEFINES_VH
