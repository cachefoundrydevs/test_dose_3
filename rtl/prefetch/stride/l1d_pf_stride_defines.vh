// l1d_pf_stride_defines.vh
// Default value of every l1d_pf_stride parameter.
// Used for: l1d_pf_stride.v and its testbenches. A value may also be set
// on the command line (-D L1D_PFS_RPT_WAYS=2), so every define below is
// guarded with `ifndef. Sizes are design choices (README section 10),
// to be checked with a trace model.

`ifndef L1D_PF_STRIDE_DEFINES_VH
`define L1D_PF_STRIDE_DEFINES_VH

// ---------------- interface widths ----------------
`ifndef L1D_PFS_PC_W
`define L1D_PFS_PC_W        64    // virtual PC from the core (user)
`endif
`ifndef L1D_PFS_PA_W
`define L1D_PFS_PA_W        56    // physical address used (user)
`endif
`ifndef L1D_PFS_ADDR_W
`define L1D_PFS_ADDR_W      64    // pf_addr port (l1d_top pf_addr)
`endif
`ifndef L1D_PFS_LINE_BYTES
`define L1D_PFS_LINE_BYTES  64    // cache line in bytes
`endif
`ifndef L1D_PFS_PAGE_BYTES
`define L1D_PFS_PAGE_BYTES  4096  // prefetches never leave this page
`endif

// ---------------- table (RPT) ----------------
`ifndef L1D_PFS_RPT_SETS
`define L1D_PFS_RPT_SETS    16    // sets, index PC[4:1]
`endif
`ifndef L1D_PFS_RPT_WAYS
`define L1D_PFS_RPT_WAYS    4     // ways per set (at least 2)
`endif
`ifndef L1D_PFS_TAG_W
`define L1D_PFS_TAG_W       12    // PC tag, XOR fold of PC[63:5]
`endif
`ifndef L1D_PFS_PC_LSB
`define L1D_PFS_PC_LSB      1     // PC bit 0 dropped (2-byte aligned)
`endif

// ---------------- prefetch ----------------
`ifndef L1D_PFS_DIST
`define L1D_PFS_DIST        2     // strides ahead (power of two)
`endif
`ifndef L1D_PFS_FILT_DEPTH
`define L1D_PFS_FILT_DEPTH  4     // recent prefetch lines remembered
`endif
`ifndef L1D_PFS_KEY_W
`define L1D_PFS_KEY_W       16    // filter key: page hash + line in page
`endif

`endif // L1D_PF_STRIDE_DEFINES_VH
