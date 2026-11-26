// l1d_pf_stride.v
// Stride prefetcher of the L1D: a PC-indexed Reference Prediction Table
// (RPT) with the Chen and Baer trust states, page-bounded, byte strides.
// Used for: one shared instance beside l1d_top. Trains on every accepted
// cacheable load and store (program order); sends one line prefetch per
// trigger to l1d_top pf_valid / pf_addr / pf_ready.
// Spec: ../README.md (sections 2, 10, 13). Pipeline, forwarding,
// measured timing and the differences from the README: ../DESIGN.md.
//
//   tr_* -> T tap -> L lookup (CAM) -> R read, victim -> D compares
//        -> U trust update -> W table write -> G target -> F filter
//        -> X send -> pf_* (registered)
//
// One event enters per cycle and the pipeline never stalls (T-5). The
// 4 events ahead of an event are forwarded, so no training sample is
// lost or delayed; the result equals one-at-a-time processing except
// one rule (DESIGN.md section 4): a new entry never takes a way that
// the events of the 3 cycles before wrote.
//
// Assumptions (V-9.2):
//   A1 at most one training event per cycle, in program order
//   A2 RPT_SETS, RPT_WAYS, LINE_BYTES, PAGE_BYTES, DIST powers of two;
//      RPT_WAYS >= 4 (3 ways may be masked, a victim always remains)
//   A3 PA_W <= ADDR_W, PA_W > page bits, KEY_W > line-in-page bits
//   A4 pf_valid is held until pf_ready = 1 or a newer prefetch replaces
//      it (newest first); nothing upstream ever waits for pf_ready
// Combinational outputs (V-3.11): none, pf_* are registered.

`ifndef L1D_PF_STRIDE_V
`define L1D_PF_STRIDE_V
`default_nettype none
`include "l1d_pf_stride_defines.vh"

module l1d_pf_stride #(
   parameter PC_W       = `L1D_PFS_PC_W,       // 64 bits
   parameter PA_W       = `L1D_PFS_PA_W,       // 56 bits
   parameter ADDR_W     = `L1D_PFS_ADDR_W,     // 64 bits
   parameter LINE_BYTES = `L1D_PFS_LINE_BYTES, // 64 bytes
   parameter PAGE_BYTES = `L1D_PFS_PAGE_BYTES, // 4096 bytes
   parameter RPT_SETS   = `L1D_PFS_RPT_SETS,   // 16 sets
   parameter RPT_WAYS   = `L1D_PFS_RPT_WAYS,   // 4 ways
   parameter TAG_W      = `L1D_PFS_TAG_W,      // 12 bits
   parameter PC_LSB     = `L1D_PFS_PC_LSB,     // bit 1
   parameter DIST       = `L1D_PFS_DIST,       // 2 strides
   parameter FILT_DEPTH = `L1D_PFS_FILT_DEPTH, // 4 entries
   parameter KEY_W      = `L1D_PFS_KEY_W       // 16 bits
) (
   rst,
   clk,
   clear,
   tr_valid,
   tr_pc,
   tr_pa,
   tr_is_store,
   pf_ready,
   pf_valid,
   pf_addr,
   pf_unique
);

   // ---------------- derived sizes ----------------
   localparam NS       = RPT_SETS;                 // 16 sets
   localparam NW       = RPT_WAYS;                 // 4 ways
   localparam ENTRIES  = NS * NW;                  // 64 entries
   localparam SET_W    = $clog2(NS);               // 4 bits
   localparam OFFSET_W = $clog2(LINE_BYTES);       // 6 bits
   localparam PAGE_W   = $clog2(PAGE_BYTES);       // 12 bits
   localparam LIP_W    = PAGE_W - OFFSET_W;        // 6 bits line in page
   localparam PPN_W    = PA_W - PAGE_W;            // 44 bits page number
   localparam PH_W     = KEY_W - LIP_W;            // 10 bits page hash
   localparam STRIDE_W = PAGE_W + 1;               // 13 bits, signed
   localparam DSH      = $clog2(DIST);             // 1: x2 is a shift
   localparam TGT_W    = PAGE_W + DSH + 2;         // 15 bits, signed
   localparam FOLD_LSB = PC_LSB + SET_W;           // bit 5
   localparam FOLD_W   = PC_W - FOLD_LSB;          // 59 bits folded
   localparam EKEY_W   = SET_W + TAG_W + 1;        // 17 bits set,tag,st
   localparam PADA_W   = ADDR_W - PA_W;            // 8 bits zero pad
   localparam NFW      = 3;                        // 3 forward sources
   localparam NEA      = 4;                        // 4 events ahead seen

   // trust states (README section 2); the reset value is S_INIT
   localparam ST_W = 2;                            // 2 bits, 4 states
   localparam [ST_W-1:0] S_INIT   = 2'd0;
   localparam [ST_W-1:0] S_TRANS  = 2'd1;
   localparam [ST_W-1:0] S_STEADY = 2'd2;
   localparam [ST_W-1:0] S_NOPRED = 2'd3;

   // eviction classes, first evicted first (README section 10):
   // free, No-pred, Initial, Transient, Steady; one-hot
   localparam NCLS = 5;                            // 5 classes
   localparam [NCLS-1:0] C_FREE   = 5'b00001;
   localparam [NCLS-1:0] C_NOPRED = 5'b00010;
   localparam [NCLS-1:0] C_INIT   = 5'b00100;
   localparam [NCLS-1:0] C_TRANS  = 5'b01000;
   localparam [NCLS-1:0] C_STEADY = 5'b10000;

   // forward source of U (one-hot): the event 1, 2 or 3 ahead
   localparam [NFW-1:0] FW_NONE = 3'b000;
   localparam [NFW-1:0] FW_1    = 3'b001;
   localparam [NFW-1:0] FW_2    = 3'b010;
   localparam [NFW-1:0] FW_3    = 3'b100;

   localparam [STRIDE_W-1:0] STRIDE_ZERO = {STRIDE_W{1'b0}};
   localparam [NW-1:0]       WAY_ZERO    = {NW{1'b0}};

   // ---------------- ports ----------------
   input  wire                rst;          // synchronous reset
   input  wire                clk;          // clock
   input  wire                clear;        // invalidate all (satp write)
   input  wire                tr_valid;     // one accepted LSU request
   input  wire [PC_W-1:0]     tr_pc;        // PC_W = 64 bits, virtual
   input  wire [PA_W-1:0]     tr_pa;        // PA_W = 56 bits, physical
   input  wire                tr_is_store;  // 1 = store
   input  wire                pf_ready;     // l1d_pf_in has room
   output reg                 pf_valid;     // prefetch request
   output reg  [ADDR_W-1:0]   pf_addr;      // ADDR_W = 64 bits, line
   output reg                 pf_unique;    // 1 = store line (S-10)

   // ---------------- declarations: table (flops) ----------------
   reg  [ENTRIES-1:0]         rpt_valid;                 // 64 bits
   reg  [ENTRIES-1:0]         rpt_st;                    // 64 bits
   reg  [TAG_W-1:0]           rpt_tag    [0:ENTRIES-1];  // 12 b x 64
   reg  [ST_W-1:0]            rpt_state  [0:ENTRIES-1];  // 2 b x 64
   reg  [STRIDE_W-1:0]        rpt_stride [0:ENTRIES-1];  // 13 b x 64
   reg  [PAGE_W-1:0]          rpt_lo     [0:ENTRIES-1];  // 12 b x 64
   reg  [PH_W-1:0]            rpt_ph     [0:ENTRIES-1];  // 10 b x 64

   // ---------------- declarations: T, tap ----------------
   reg  [TAG_W-1:0]           tap_tag;      // TAG_W = 12 bits, folded
   reg  [PH_W-1:0]            tap_ph;       // PH_W = 10 bits, page hash
   reg  [NS-1:0]              tap_set1h;    // NS = 16 sets, decoded
   reg  [NEA:1]               tap_ss;       // 4 bits, same set as event k
   reg  [NEA:1]               t_ss;         // 4 bits, same set as event k
   reg                        t_valid;      // event in T
   reg  [SET_W-1:0]           t_set;        // SET_W = 4 bits
   reg  [NS-1:0]              t_set1h;      // NS = 16 bits, one-hot
   reg  [TAG_W-1:0]           t_tag;        // TAG_W = 12 bits
   reg                        t_st;         // store event
   reg  [PAGE_W-1:0]          t_lo;         // PAGE_W = 12 bits, offset
   reg  [PH_W-1:0]            t_ph;         // PH_W = 10 bits
   reg  [PPN_W-1:0]           t_ppn;        // PPN_W = 44 bits
   wire [EKEY_W-1:0]          t_key;        // EKEY_W = 17 bits

   // ---------------- declarations: L, lookup ----------------
   reg  [ENTRIES-1:0]         l_hit;        // 64 bits, CAM match
   reg  [NW*NCLS-1:0]         l_cls;        // 4 x 5 bits, class
   reg  [NCLS-1:0]            e_cls;        // 5 bits, one entry
   reg  [NW*NCLS-1:0]         l_vcls;       // 4 x 5 bits, victim class
   reg  [NW-1:0]              l_cand;       // NW = 4, victim candidates
   reg  [NW-1:0]              l_rpl;        // NW = 4, replaced, ev. 2-4
   reg  [ENTRIES-1:0]         l_g4;         // 64 bits, event 4's entry
   wire [STRIDE_W-1:0]        l_dl1;        // 13 bits, to the R event
   wire [STRIDE_W-1:0]        l_dl2;        // 13 bits, to the D event
   wire [STRIDE_W-1:0]        l_dl3;        // 13 bits, to the U event
   wire [STRIDE_W-1:0]        tap_lo_x;     // 13 bits, next t_lo
   wire [STRIDE_W-1:0]        t_lo_x;       // 13 bits, next r_lo
   wire [STRIDE_W-1:0]        r_lo_x;       // 13 bits, next d_lo
   wire [STRIDE_W-1:0]        d_lo_x;       // 13 bits, d_lo (next u_lo)

   // ---------------- declarations: R, read and victim ----------------
   reg                        r_valid;      // event in R
   reg  [SET_W-1:0]           r_set;        // SET_W = 4 bits
   reg  [NS-1:0]              r_set1h;      // NS = 16 bits
   reg  [TAG_W-1:0]           r_tag;        // TAG_W = 12 bits
   reg                        r_st;         // store event
   reg  [PAGE_W-1:0]          r_lo;         // PAGE_W = 12 bits
   reg  [PH_W-1:0]            r_ph;         // PH_W = 10 bits
   reg  [PPN_W-1:0]           r_ppn;        // PPN_W = 44 bits
   reg  [ENTRIES-1:0]         r_hit;        // 64 bits
   reg  [ENTRIES-1:0]         r_g4;         // 64 bits, event 4's entry
   reg  [NW*NCLS-1:0]         r_cls;        // 4 x 5 bits, victim class
   reg  [NW-1:0]              r_cand;       // NW = 4, victim candidates
   reg  [NW-1:0]              r_avail;      // NW = 4, candidates now
   reg  [NW-1:0]              r_rpl;        // NW = 4, replaced, ev. 2-4
   reg                        r_hitok;      // CAM hit not replaced
   reg  [NEA:1]               r_m;          // same entry as event k
   reg                        r_ss1;        // same set as event 1
   reg  [NFW:1]               r_sp;         // same page as event k
   reg  [STRIDE_W-1:0]        r_dl1;        // 13 bits
   reg  [STRIDE_W-1:0]        r_dl2;        // 13 bits
   reg  [STRIDE_W-1:0]        r_dl3;        // 13 bits
   wire [EKEY_W-1:0]          r_key;        // EKEY_W = 17 bits
   reg  [NW-1:0]              r_hitway;     // NW = 4, CAM way
   reg  [NW-1:0]              r_vict;       // NW = 4, victim one-hot
   reg  [NW-1:0]              r_way;        // NW = 4, chosen way
   reg                        r_alloc;      // new entry
   reg  [NFW-1:0]             r_src;        // 3 bits, forward source
   reg                        r_spm;        // same page, forwarded
   reg  [STRIDE_W-1:0]        r_dlm;        // 13 bits, forwarded delta
   reg  [STRIDE_W-1:0]        r_rd_stride;  // 13 bits, table read
   reg  [ST_W-1:0]            r_rd_state;   // 2 bits, table read
   reg  [PAGE_W-1:0]          r_rd_lo;      // 12 bits, table read
   reg  [PH_W-1:0]            r_rd_ph;      // 10 bits, table read
   reg  [STRIDE_W-1:0]        r_ht_stride;  // 13 bits, CAM-hit read
   reg  [ST_W-1:0]            r_ht_state;   // 2 bits
   reg  [PAGE_W-1:0]          r_ht_lo;      // 12 bits
   reg  [PH_W-1:0]            r_ht_ph;      // 10 bits
   reg  [STRIDE_W-1:0]        r_g4_stride;  // 13 bits, event 4's read
   reg  [ST_W-1:0]            r_g4_state;   // 2 bits
   reg  [PAGE_W-1:0]          r_g4_lo;      // 12 bits
   reg  [PH_W-1:0]            r_g4_ph;      // 10 bits

   // ---------------- declarations: D, delta ----------------
   reg                        d_valid;      // event in D
   reg  [SET_W-1:0]           d_set;        // SET_W = 4 bits
   reg  [NS-1:0]              d_set1h;      // NS = 16 bits
   reg  [TAG_W-1:0]           d_tag;        // TAG_W = 12 bits
   reg                        d_st;         // store event
   reg  [PAGE_W-1:0]          d_lo;         // PAGE_W = 12 bits
   reg  [PH_W-1:0]            d_ph;         // PH_W = 10 bits
   reg  [PPN_W-1:0]           d_ppn;        // PPN_W = 44 bits
   reg  [NW-1:0]              d_way;        // NW = 4 ways
   reg                        d_alloc;      // new entry
   reg  [NFW-1:0]             d_src;        // 3 bits
   reg                        d_spm;        // same page, forwarded
   reg  [STRIDE_W-1:0]        d_dlm;        // 13 bits
   reg  [STRIDE_W-1:0]        d_rd_stride;  // 13 bits
   reg  [ST_W-1:0]            d_rd_state;   // 2 bits
   reg  [PAGE_W-1:0]          d_rd_lo;      // 12 bits
   reg  [PH_W-1:0]            d_rd_ph;      // 10 bits
   wire [EKEY_W-1:0]          d_key;        // EKEY_W = 17 bits
   wire [STRIDE_W-1:0]        d_dlt;        // 13 bits, delta to table
   wire [STRIDE_W-1:0]        r_rd_lo_x;    // 13 bits, next d_rd_lo
   wire [STRIDE_W-1:0]        d_rd_lo_x;    // 13 bits, d_rd_lo extended
   wire                       d_eqt;        // delta == table stride
   reg                        d_c1k;        // delta == event 1 kept
   reg                        d_c1n;        // delta == event 1 new delta
   reg                        d_c1z;        // delta == 0
   reg                        d_c2;         // delta == event 2 stride
   reg                        d_c3;         // delta == event 3 stride
   reg                        d_none;       // no forward source
   reg                        d_cf;         // delta == F's stride
   reg                        d_cr;         // delta == U's table read
   reg                        d_cm;         // delta == U's forwarded dl
   reg                        d_cd;         // delta == U's table dl

   // ---------------- declarations: U, update ----------------
   reg                        u_valid;      // event in U
   reg  [SET_W-1:0]           u_set;        // SET_W = 4 bits
   reg  [NS-1:0]              u_set1h;      // NS = 16 bits
   reg  [TAG_W-1:0]           u_tag;        // TAG_W = 12 bits
   reg                        u_st;         // store event
   reg  [PAGE_W-1:0]          u_lo;         // PAGE_W = 12 bits
   reg  [PH_W-1:0]            u_ph;         // PH_W = 10 bits
   reg  [PPN_W-1:0]           u_ppn;        // PPN_W = 44 bits
   reg  [NW-1:0]              u_way;        // NW = 4 ways
   reg                        u_alloc;      // new entry
   reg  [NFW-1:0]             u_src;        // 3 bits
   reg                        u_none;       // u_src == FW_NONE
   reg                        u_spm;        // same page, forwarded
   reg  [STRIDE_W-1:0]        u_dlm;        // 13 bits
   reg                        u_spt;        // same page, table
   reg  [STRIDE_W-1:0]        u_dlt;        // 13 bits
   reg                        u_ct;         // correct, table source
   reg                        u_c1k;        // see d_c1k .. d_c3
   reg                        u_c1n;
   reg                        u_c1z;
   reg                        u_c2;
   reg                        u_c3;
   reg  [STRIDE_W-1:0]        u_rd_stride;  // 13 bits
   reg  [ST_W-1:0]            u_rd_state;   // 2 bits
   wire [EKEY_W-1:0]          u_key;        // EKEY_W = 17 bits
   reg  [STRIDE_W-1:0]        e_stride;     // 13 bits, entry stride
   reg  [ST_W-1:0]            e_state;      // 2 bits, entry state
   reg                        e_sp;         // same page as last
   reg  [STRIDE_W-1:0]        e_dl;         // 13 bits, delta
   reg                        correct;      // delta == stride
   reg  [STRIDE_W-1:0]        new_dl;       // 13 bits, delta or 0
   reg  [STRIDE_W-1:0]        nx_stride;    // 13 bits, written
   reg  [ST_W-1:0]            nx_state;     // 2 bits, written
   reg  [NCLS-1:0]            nx_cls;       // 5 bits, class written
   reg                        issue;        // Steady and correct
   reg                        keep;         // stride kept

   // ---------------- declarations: W, table write ----------------
   reg                        w_valid;      // event in W
   reg  [SET_W-1:0]           w_set;        // SET_W = 4 bits
   reg  [NS-1:0]              w_set1h;      // NS = 16 bits
   reg  [TAG_W-1:0]           w_tag;        // TAG_W = 12 bits
   reg                        w_st;         // store event
   reg  [PAGE_W-1:0]          w_lo;         // PAGE_W = 12 bits
   reg  [PH_W-1:0]            w_ph;         // PH_W = 10 bits
   reg  [PPN_W-1:0]           w_ppn;        // PPN_W = 44 bits
   reg  [NW-1:0]              w_way;        // NW = 4 ways
   reg  [NCLS-1:0]            w_cls;        // 5 bits
   reg  [STRIDE_W-1:0]        w_nx_stride;  // 13 bits
   reg  [ST_W-1:0]            w_nx_state;   // 2 bits
   reg                        w_issue;      // prefetch wanted
   reg  [STRIDE_W-1:0]        w_stride;     // 13 bits, for the target
   reg                        w_keep;       // stride kept (not alloc)
   reg                        w_alloc;      // new entry, stride 0
   wire [EKEY_W-1:0]          w_key;        // EKEY_W = 17 bits

   // ---------------- declarations: G, target ----------------
   reg  [NW-1:0]              g_way;        // NW = 4 ways
   reg  [STRIDE_W-1:0]        g_nx_stride;  // 13 bits
   reg  [ST_W-1:0]            g_nx_state;   // 2 bits
   reg                        g_issue;      // prefetch wanted
   reg                        g_st;         // store line
   reg  [LIP_W-1:0]           g_cur;        // LIP_W = 6 bits, trigger
   reg  [PH_W-1:0]            g_ph;         // PH_W = 10 bits
   reg  [PPN_W-1:0]           g_ppn;        // PPN_W = 44 bits
   wire [TGT_W-1:0]           w_lo_x;       // 15 bits, next g_lo
   wire [TGT_W-1:0]           w_step;       // 15 bits, next stride x DIST
   wire [TGT_W-1:0]           g_tgt;        // 15 bits, target offset
   reg                        g_in_page;    // target in this page

   // ---------------- declarations: F, filter and output ----------------
   reg                        f_valid;      // candidate in F
   reg                        f_inp;        // target in the page
   reg  [STRIDE_W-1:0]        f_nx_stride;  // 13 bits
   reg  [ST_W-1:0]            f_nx_state;   // 2 bits
   reg                        f_st;         // store line
   reg  [PPN_W-1:0]           f_ppn;        // PPN_W = 44 bits
   reg  [LIP_W-1:0]           f_lip;        // LIP_W = 6 bits, target
   reg  [LIP_W-1:0]           f_cur;        // LIP_W = 6 bits, trigger
   reg  [PH_W-1:0]            f_ph;         // PH_W = 10 bits
   wire [KEY_W-1:0]           f_key;        // KEY_W = 16 bits
   reg                        f_dup;        // recently prefetched
   reg                        f_pass;       // send this candidate
   reg  [FILT_DEPTH-1:0]      filt_v;       // FILT_DEPTH = 4 entries
   reg  [KEY_W-1:0]           filt_key [0:FILT_DEPTH-1]; // 16 b x 4

   // ---------------- declarations: X, send ----------------
   reg                        x_pass;       // candidate passed F
   reg  [KEY_W-1:0]           x_key;        // KEY_W = 16 bits
   reg                        x_st;         // store line
   reg  [PPN_W-1:0]           x_ppn;        // PPN_W = 44 bits
   reg  [LIP_W-1:0]           x_lip;        // LIP_W = 6 bits

   // ================= T: fold, decode, tap register =================
   always @* begin : tap_proc
      integer i;                                 // bit index
      tap_tag = {TAG_W{1'b0}};
      for (i = 0; i < FOLD_W; i = i + 1) begin
         tap_tag[i % TAG_W] = tap_tag[i % TAG_W] ^ tr_pc[FOLD_LSB + i];
      end
      tap_ph = {PH_W{1'b0}};
      for (i = 0; i < PPN_W; i = i + 1) begin
         tap_ph[i % PH_W] = tap_ph[i % PH_W] ^ tr_pa[PAGE_W + i];
      end
      for (i = 0; i < NS; i = i + 1) begin
         tap_set1h[i] = (tr_pc[PC_LSB +: SET_W] == i[SET_W-1:0]);
      end
      // same set as the events 1 .. 4 ahead (now in L, R, D, U)
      tap_ss[1] = t_valid && (tr_pc[PC_LSB +: SET_W] == t_set);
      tap_ss[2] = r_valid && (tr_pc[PC_LSB +: SET_W] == r_set);
      tap_ss[3] = d_valid && (tr_pc[PC_LSB +: SET_W] == d_set);
      tap_ss[4] = u_valid && (tr_pc[PC_LSB +: SET_W] == u_set);
   end // always tap_proc

   always @(posedge clk) begin : t_reg_proc
      if (rst) begin
         t_valid <= 1'b0;
      end
      else begin
         t_valid <= tr_valid && !clear;
      end
      t_set   <= tr_pc[PC_LSB +: SET_W];
      t_set1h <= tap_set1h;
      t_ss    <= tap_ss;
      t_tag   <= tap_tag;
      t_st    <= tr_is_store;
      t_lo    <= tr_pa[PAGE_W-1:0];
      t_ph    <= tap_ph;
      t_ppn   <= tr_pa[PA_W-1:PAGE_W];
   end // always t_reg_proc

   assign t_key = {t_set, t_tag, t_st};
   assign r_key = {r_set, r_tag, r_st};
   assign d_key = {d_set, d_tag, d_st};
   assign u_key = {u_set, u_tag, u_st};
   assign w_key = {w_set, w_tag, w_st};

   // ================= L: CAM lookup, deltas to the events ahead ======
   // Every entry compares its tag with the event (set one-hot from T),
   // so no set multiplexer sits in front of the compare (T-2).
   always @* begin : lookup_proc
      integer s;                                 // set index
      integer w;                                 // way index
      integer e;                                 // entry index
      l_cls = {(NW * NCLS){1'b0}};
      for (s = 0; s < NS; s = s + 1) begin
         for (w = 0; w < NW; w = w + 1) begin
            e = s * NW + w;
            l_hit[e] = t_set1h[s] && rpt_valid[e] &&
                       (rpt_tag[e] == t_tag) && (rpt_st[e] == t_st);
            // event 4 is in W now and in G when this event reads in R
            l_g4[e]  = t_set1h[s] && w_way[w];
            if (!rpt_valid[e]) begin
               e_cls = C_FREE;
            end
            else begin
               case (rpt_state[e])
                  S_NOPRED: e_cls = C_NOPRED;
                  S_INIT:   e_cls = C_INIT;
                  S_TRANS:  e_cls = C_TRANS;
                  default:  e_cls = C_STEADY;
               endcase
            end // else (!rpt_valid[e])
            if (t_set1h[s]) begin
               l_cls[w*NCLS +: NCLS] = l_cls[w*NCLS +: NCLS] | e_cls;
            end
         end
      end
      // victim inputs, from flops only: event 4 (in W) writes its way
      // at the end of this cycle, so its new class stands in; the ways
      // of events 2 and 3 (in D, U) are masked (event 1's in R)
      for (w = 0; w < NW; w = w + 1) begin
         if (t_ss[4] && w_way[w]) begin
            l_vcls[w*NCLS +: NCLS] = w_cls;
         end
         else begin
            l_vcls[w*NCLS +: NCLS] = l_cls[w*NCLS +: NCLS];
         end
         l_cand[w] = !(t_ss[2] && d_way[w]) && !(t_ss[3] && u_way[w]);
         l_rpl[w]  = (t_ss[2] && d_way[w]) || (t_ss[3] && u_way[w]) ||
                     (t_ss[4] && w_way[w]);
      end
   end // always lookup_proc

   // byte deltas to the 3 events ahead, t_lo - r_lo, - d_lo, - u_lo;
   // each subtract holds its own copy of the operand flops (fed with
   // the values those flops load), so it starts at clean flop outputs
   assign tap_lo_x = {1'b0, tr_pa[PAGE_W-1:0]};
   assign t_lo_x   = {1'b0, t_lo};
   assign r_lo_x   = {1'b0, r_lo};
   assign d_lo_x   = {1'b0, d_lo};

   l1d_ks_add_q #(.W(STRIDE_W), .SUB(1)) u1_ks_add_q (
      .clk (clk),
      .a_d (tap_lo_x),
      .b_d (t_lo_x),
      .sum (l_dl1)
   );

   l1d_ks_add_q #(.W(STRIDE_W), .SUB(1)) u2_ks_add_q (
      .clk (clk),
      .a_d (tap_lo_x),
      .b_d (r_lo_x),
      .sum (l_dl2)
   );

   l1d_ks_add_q #(.W(STRIDE_W), .SUB(1)) u3_ks_add_q (
      .clk (clk),
      .a_d (tap_lo_x),
      .b_d (d_lo_x),
      .sum (l_dl3)
   );

   always @(posedge clk) begin : l_reg_proc
      if (rst) begin
         r_valid <= 1'b0;
      end
      else begin
         r_valid <= t_valid && !clear;
      end
      r_set   <= t_set;
      r_set1h <= t_set1h;
      r_tag   <= t_tag;
      r_st    <= t_st;
      r_lo    <= t_lo;
      r_ph    <= t_ph;
      r_ppn   <= t_ppn;
      r_hit   <= l_hit;
      r_g4    <= l_g4;
      r_cls   <= l_vcls;
      r_cand  <= l_cand;
      r_rpl   <= l_rpl;
      r_m[1]  <= r_valid && (r_key == t_key);
      r_m[2]  <= d_valid && (d_key == t_key);
      r_m[3]  <= u_valid && (u_key == t_key);
      r_m[4]  <= w_valid && (w_key == t_key);
      r_ss1   <= t_ss[1];
      r_sp[1] <= (r_ph == t_ph);
      r_sp[2] <= (d_ph == t_ph);
      r_sp[3] <= (u_ph == t_ph);
      r_dl1   <= l_dl1;
      r_dl2   <= l_dl2;
      r_dl3   <= l_dl3;
   end // always l_reg_proc

   // ================= R: read the entry, pick the way =================
   // Events ahead at this point: 1 in D, 2 in U, 3 in W, 4 in G.
   // Two reads in parallel, from flops only (never the victim logic):
   // the CAM hit, and event 4's way (event 4 has written the table).
   always @* begin : read_proc
      integer s;                                 // set index
      integer w;                                 // way index
      integer e;                                 // entry index
      r_hitway    = WAY_ZERO;
      r_ht_stride = STRIDE_ZERO;
      r_ht_state  = S_INIT;
      r_ht_lo     = {PAGE_W{1'b0}};
      r_ht_ph     = {PH_W{1'b0}};
      r_g4_stride = STRIDE_ZERO;
      r_g4_state  = S_INIT;
      r_g4_lo     = {PAGE_W{1'b0}};
      r_g4_ph     = {PH_W{1'b0}};
      for (s = 0; s < NS; s = s + 1) begin
         for (w = 0; w < NW; w = w + 1) begin
            e = s * NW + w;
            r_hitway[w] = r_hitway[w] | r_hit[e];
            if (r_hit[e]) begin
               r_ht_stride = r_ht_stride | rpt_stride[e];
               r_ht_state  = r_ht_state  | rpt_state[e];
               r_ht_lo     = r_ht_lo     | rpt_lo[e];
               r_ht_ph     = r_ht_ph     | rpt_ph[e];
            end
            if (r_g4[e]) begin
               r_g4_stride = r_g4_stride | rpt_stride[e];
               r_g4_state  = r_g4_state  | rpt_state[e];
               r_g4_lo     = r_g4_lo     | rpt_lo[e];
               r_g4_ph     = r_g4_ph     | rpt_ph[e];
            end
         end
      end
      if (r_m[4]) begin
         r_rd_stride = r_g4_stride;
         r_rd_state  = r_g4_state;
         r_rd_lo     = r_g4_lo;
         r_rd_ph     = r_g4_ph;
      end
      else begin
         r_rd_stride = r_ht_stride;
         r_rd_state  = r_ht_state;
         r_rd_lo     = r_ht_lo;
         r_rd_ph     = r_ht_ph;
      end // else (r_m[4])
   end // always read_proc

   // Victim: lowest class first, then the lowest way, among the ways no
   // event of the 3 cycles before writes (events 2, 3 masked in L,
   // event 1 here). Pairwise: a way wins if it beats every other
   // candidate, so no priority chain over the classes.
   always @* begin : victim_proc
      integer a;                                 // way index
      integer b;                                 // way index
      integer c;                                 // class index
      integer k;                                 // class index
      reg     lower;                             // class(b) < class(a)
      reg     same;                              // class(b) = class(a)
      for (a = 0; a < NW; a = a + 1) begin
         r_avail[a] = r_cand[a] && !(r_ss1 && d_way[a]);
      end
      // a CAM hit stands unless an event ahead wrote that way (L marks
      // events 2 .. 4, event 1 here)
      r_hitok = ((r_hitway & ~r_rpl & ~(d_way & {NW{r_ss1}})) !=
                 WAY_ZERO);
      for (a = 0; a < NW; a = a + 1) begin
         r_vict[a] = r_avail[a];
         for (b = 0; b < NW; b = b + 1) begin
            if (b != a) begin
               lower = 1'b0;
               same  = 1'b0;
               for (c = 0; c < NCLS; c = c + 1) begin
                  same = same | (r_cls[a*NCLS + c] & r_cls[b*NCLS + c]);
                  for (k = 0; k < c; k = k + 1) begin
                     lower = lower | (r_cls[a*NCLS + c] & r_cls[b*NCLS + k]);
                  end
               end
               // b beats a: lower class, or same class and lower way
               if (r_avail[b] && (lower || (same && (b < a)))) begin
                  r_vict[a] = 1'b0;
               end
            end
         end
      end
   end // always victim_proc

   // Source of the entry, a priority chain on purpose (V-5.19): the
   // youngest event ahead with the same entry, else a CAM hit that no
   // event ahead has replaced, else allocate the victim. Event 4's way
   // is never replaced by event 1 (3-cycle mask), so m4 needs no check.
   always @* begin : select_proc
      r_way   = r_vict;
      r_alloc = 1'b1;
      r_src   = FW_NONE;
      r_spm   = 1'b0;
      r_dlm   = STRIDE_ZERO;
      if (r_m[1]) begin
         r_way   = d_way;
         r_alloc = 1'b0;
         r_src   = FW_1;
         r_spm   = r_sp[1];
         r_dlm   = r_dl1;
      end
      else if (r_m[2]) begin
         r_way   = u_way;
         r_alloc = 1'b0;
         r_src   = FW_2;
         r_spm   = r_sp[2];
         r_dlm   = r_dl2;
      end
      else if (r_m[3]) begin
         r_way   = w_way;
         r_alloc = 1'b0;
         r_src   = FW_3;
         r_spm   = r_sp[3];
         r_dlm   = r_dl3;
      end
      else if (r_m[4]) begin
         r_way   = g_way;
         r_alloc = 1'b0;
      end
      else if (r_hitok) begin
         r_way   = r_hitway;
         r_alloc = 1'b0;
      end // else if (CAM hit not replaced)
      else begin
         r_way   = r_vict;      // null branch: allocate, defaults stand
      end // else (allocate)
   end // always select_proc

   always @(posedge clk) begin : r_reg_proc
      if (rst) begin
         d_valid <= 1'b0;
      end
      else begin
         d_valid <= r_valid && !clear;
      end
      d_set       <= r_set;
      d_set1h     <= r_set1h;
      d_tag       <= r_tag;
      d_st        <= r_st;
      d_lo        <= r_lo;
      d_ph        <= r_ph;
      d_ppn       <= r_ppn;
      d_way       <= r_way;
      d_alloc     <= r_alloc;
      d_src       <= r_src;
      d_spm       <= r_spm;
      d_dlm       <= r_dlm;
      d_rd_stride <= r_rd_stride;
      d_rd_state  <= r_rd_state;
      d_rd_lo     <= r_rd_lo;
      d_rd_ph     <= r_rd_ph;
   end // always r_reg_proc

   // ================= D: deltas, compares for U =================
   // Events ahead at this point: 1 in U, 2 in W, 3 in G. Every compare
   // U needs is made here from flops (and U's source mux), so U only
   // selects: event 1's next stride is its kept stride, its new delta
   // or 0, and all three are compared; events 2 and 3 are final.
   assign r_rd_lo_x = {1'b0, r_rd_lo};
   assign d_rd_lo_x = {1'b0, d_rd_lo};

   // d_lo - d_rd_lo (operand flops inside, loaded with r_lo, r_rd_lo)
   l1d_ks_add_q #(.W(STRIDE_W), .SUB(1)) u4_ks_add_q (
      .clk (clk),
      .a_d (r_lo_x),
      .b_d (r_rd_lo_x),
      .sum (d_dlt)
   );

   // d_lo - d_rd_lo == d_rd_stride, as d_rd_lo + d_rd_stride == d_lo
   l1d_add_eq #(.W(STRIDE_W)) u1_add_eq (
      .a  (d_rd_lo_x),
      .b  (d_rd_stride),
      .c  (d_lo_x),
      .eq (d_eqt)
   );

   // compare with every value first, then select (u_src is one-hot)
   always @* begin : fwd_cmp_proc
      d_none = (d_src == FW_NONE);
      d_c1z  = (d_dlm == STRIDE_ZERO);
      d_c2   = (d_dlm == w_nx_stride);
      d_c3   = (d_dlm == g_nx_stride);
      d_cf   = (d_dlm == f_nx_stride);
      d_cr   = (d_dlm == u_rd_stride);
      d_cm   = (d_dlm == u_dlm);
      d_cd   = (d_dlm == u_dlt);
      // U's e_stride is w_, g_, f_nx_stride or u_rd_stride
      d_c1k  = (u_src[0] && d_c2) || (u_src[1] && d_c3) ||
               (u_src[2] && d_cf) || (u_none && d_cr);
      // U's new delta is e_sp ? e_dl : 0
      d_c1n  = (u_none ? (u_spt ? d_cd : d_c1z) : (u_spm ? d_cm : d_c1z));
   end // always fwd_cmp_proc

   always @(posedge clk) begin : d_reg_proc
      if (rst) begin
         u_valid <= 1'b0;
      end
      else begin
         u_valid <= d_valid && !clear;
      end
      u_set       <= d_set;
      u_set1h     <= d_set1h;
      u_tag       <= d_tag;
      u_st        <= d_st;
      u_lo        <= d_lo;
      u_ph        <= d_ph;
      u_ppn       <= d_ppn;
      u_way       <= d_way;
      u_alloc     <= d_alloc;
      u_src       <= d_src;
      u_none      <= d_none;
      u_spm       <= d_spm;
      u_dlm       <= d_dlm;
      u_spt       <= (d_ph == d_rd_ph);
      u_dlt       <= d_dlt;
      u_ct        <= (d_ph == d_rd_ph) && d_eqt;
      u_c1k       <= d_c1k;
      u_c1n       <= d_c1n;
      u_c1z       <= d_c1z;
      u_c2        <= d_c2;
      u_c3        <= d_c3;
      u_rd_stride <= d_rd_stride;
      u_rd_state  <= d_rd_state;
   end // always d_reg_proc

   // ================= U: trust state machine =================
   // Events ahead at this point: 1 in W, 2 in G, 3 in F (their results).
   // A change of page counts as incorrect and gives stride 0, as a
   // delta above 4095 bytes does (README section 13).
   // u_src is one-hot or zero (u_none), so every select is an AND-OR
   always @* begin : update_proc
      e_stride  = ({STRIDE_W{u_src[0]}} & w_nx_stride) |
                  ({STRIDE_W{u_src[1]}} & g_nx_stride) |
                  ({STRIDE_W{u_src[2]}} & f_nx_stride) |
                  ({STRIDE_W{u_none}}   & u_rd_stride);
      e_state   = ({ST_W{u_src[0]}} & w_nx_state) |
                  ({ST_W{u_src[1]}} & g_nx_state) |
                  ({ST_W{u_src[2]}} & f_nx_state) |
                  ({ST_W{u_none}}   & u_rd_state);
      e_sp      = u_none ? u_spt : u_spm;
      e_dl      = u_none ? u_dlt : u_dlm;
      // delta == stride, compared in D: only a select is left here
      correct   = (u_src[0] && u_spm && ((w_alloc && u_c1z) ||
                                         (w_keep && u_c1k) ||
                                         (!w_alloc && !w_keep && u_c1n))) ||
                  (u_src[1] && u_spm && u_c2) ||
                  (u_src[2] && u_spm && u_c3) ||
                  (u_none && u_ct);
      new_dl    = e_sp ? e_dl : STRIDE_ZERO;
      // the stride changes to the new delta unless Steady or correct
      keep      = !u_alloc && (correct || (e_state == S_STEADY));
      if (u_alloc) begin
         nx_stride = STRIDE_ZERO;
      end
      else if (keep) begin
         nx_stride = e_stride;
      end
      else begin
         nx_stride = new_dl;
      end
      case (e_state)
         S_INIT:   nx_state = correct ? S_STEADY : S_TRANS;
         S_TRANS:  nx_state = correct ? S_STEADY : S_NOPRED;
         S_STEADY: nx_state = correct ? S_STEADY : S_INIT;
         default:  nx_state = correct ? S_TRANS  : S_NOPRED;
      endcase
      if (u_alloc) begin
         nx_state  = S_INIT;
      end
      case (nx_state)
         S_NOPRED: nx_cls = C_NOPRED;
         S_INIT:   nx_cls = C_INIT;
         S_TRANS:  nx_cls = C_TRANS;
         default:  nx_cls = C_STEADY;
      endcase
      issue = !u_alloc && correct && (e_state != S_NOPRED) &&
              (e_stride != STRIDE_ZERO);
   end // always update_proc

   always @(posedge clk) begin : u_reg_proc
      if (rst) begin
         w_valid <= 1'b0;
         w_issue <= 1'b0;
      end
      else begin
         w_valid <= u_valid && !clear;
         w_issue <= u_valid && !clear && issue;
      end
      w_set       <= u_set;
      w_set1h     <= u_set1h;
      w_tag       <= u_tag;
      w_st        <= u_st;
      w_lo        <= u_lo;
      w_ph        <= u_ph;
      w_ppn       <= u_ppn;
      w_way       <= u_way;
      w_cls       <= nx_cls;
      w_nx_stride <= nx_stride;
      w_nx_state  <= nx_state;
      w_stride    <= e_stride;
      w_keep      <= keep;
      w_alloc     <= u_alloc;
   end // always u_reg_proc

   // ================= W: table write (fan-out only) =================
   // valid and state: reset and clear; the other fields have no reset
   // (guarded by valid, S-6), so rst never reaches their enables.
   always @(posedge clk) begin : table_ctl_proc
      integer s;                                 // set index
      integer w;                                 // way index
      if (rst || clear) begin
         rpt_valid <= {ENTRIES{1'b0}};
         for (s = 0; s < ENTRIES; s = s + 1) begin
            rpt_state[s] <= S_INIT;
         end
      end
      else begin
         for (s = 0; s < NS; s = s + 1) begin
            for (w = 0; w < NW; w = w + 1) begin
               if (w_valid && w_set1h[s] && w_way[w]) begin
                  rpt_valid[s * NW + w] <= 1'b1;
                  rpt_state[s * NW + w] <= w_nx_state;
               end
            end
         end
      end // else (rst || clear)
   end // always table_ctl_proc

   always @(posedge clk) begin : table_data_proc
      integer s;                                 // set index
      integer w;                                 // way index
      for (s = 0; s < NS; s = s + 1) begin
         for (w = 0; w < NW; w = w + 1) begin
            if (w_valid && w_set1h[s] && w_way[w]) begin
               rpt_st[s * NW + w]     <= w_st;
               rpt_tag[s * NW + w]    <= w_tag;
               rpt_stride[s * NW + w] <= w_nx_stride;
               rpt_lo[s * NW + w]     <= w_lo;
               rpt_ph[s * NW + w]     <= w_ph;
            end
         end
      end
   end // always table_data_proc

   always @(posedge clk) begin : w_reg_proc
      if (rst) begin
         g_issue <= 1'b0;
      end
      else begin
         g_issue <= w_issue && !clear;
      end
      g_way       <= w_way;
      g_nx_stride <= w_nx_stride;
      g_nx_state  <= w_nx_state;
      g_st        <= w_st;
      g_cur       <= w_lo[PAGE_W-1:OFFSET_W];
      g_ph        <= w_ph;
      g_ppn       <= w_ppn;
   end // always w_reg_proc

   // ================= G: target address, page check =================
   // g_lo + g_stride x DIST (operand flops inside, loaded from W)
   assign w_lo_x = {{(TGT_W - PAGE_W){1'b0}}, w_lo};
   assign w_step = {{(TGT_W - STRIDE_W){w_stride[STRIDE_W-1]}}, w_stride}
                   << DSH;

   l1d_ks_add_q #(.W(TGT_W), .SUB(0)) u5_ks_add_q (
      .clk (clk),
      .a_d (w_lo_x),
      .b_d (w_step),
      .sum (g_tgt)
   );

   always @* begin : page_proc
      g_in_page = (g_tgt[TGT_W-1:PAGE_W] == {(TGT_W - PAGE_W){1'b0}});
   end // always page_proc

   always @(posedge clk) begin : g_reg_proc
      if (rst) begin
         f_valid <= 1'b0;
      end
      else begin
         f_valid <= g_issue && !clear;
      end
      f_inp       <= g_in_page;
      f_nx_stride <= g_nx_stride;
      f_nx_state  <= g_nx_state;
      f_st        <= g_st;
      f_ppn       <= g_ppn;
      f_lip       <= g_tgt[PAGE_W-1:OFFSET_W];
      f_cur       <= g_cur;
      f_ph        <= g_ph;
   end // always g_reg_proc

   // ================= F: duplicate filter =================
   // Dropped: target in the trigger's own line, or one of the last
   // FILT_DEPTH lines sent (a hash: a false match only drops). The
   // candidate in X (passed last cycle) is the newest filter entry; the
   // filter holds the FILT_DEPTH - 1 before it, or FILT_DEPTH when X is
   // empty. f_pass drives one flop only (x_pass).
   assign f_key = {f_ph, f_lip};

   always @* begin : filter_proc
      integer i;                                 // filter index
      f_dup = x_pass && (x_key == f_key);
      for (i = 0; i < FILT_DEPTH - 1; i = i + 1) begin
         if (filt_v[i] && (filt_key[i] == f_key)) begin
            f_dup = 1'b1;
         end
      end
      if (!x_pass && filt_v[FILT_DEPTH-1] &&
          (filt_key[FILT_DEPTH-1] == f_key)) begin
         f_dup = 1'b1;
      end
      f_pass = f_valid && f_inp && !f_dup && (f_lip != f_cur);
   end // always filter_proc

   always @(posedge clk) begin : f_reg_proc
      if (rst) begin
         x_pass <= 1'b0;
      end
      else begin
         x_pass <= f_pass && !clear;
      end
      x_key <= f_key;
      x_st  <= f_st;
      x_ppn <= f_ppn;
      x_lip <= f_lip;
   end // always f_reg_proc

   // ================= X: filter insert, output =================
   // Every enable here is the flop x_pass.
   always @(posedge clk) begin : filt_v_proc
      if (rst || clear) begin
         filt_v <= {FILT_DEPTH{1'b0}};
      end
      else if (x_pass) begin
         filt_v <= {filt_v[FILT_DEPTH-2:0], 1'b1};
      end
   end // always filt_v_proc

   always @(posedge clk) begin : filt_key_proc
      integer i;                                 // filter index
      if (x_pass) begin
         filt_key[0] <= x_key;
         for (i = 1; i < FILT_DEPTH; i = i + 1) begin
            filt_key[i] <= filt_key[i-1];
         end
      end
   end // always filt_key_proc

   // held until taken; a newer prefetch replaces it (newest first)
   always @(posedge clk) begin : out_valid_proc
      if (rst || clear) begin
         pf_valid <= 1'b0;
      end
      else if (x_pass) begin
         pf_valid <= 1'b1;
      end
      else if (pf_ready) begin
         pf_valid <= 1'b0;
      end
   end // always out_valid_proc

   always @(posedge clk) begin : out_data_proc
      if (x_pass) begin
         pf_addr   <= {{PADA_W{1'b0}}, x_ppn, x_lip, {OFFSET_W{1'b0}}};
         pf_unique <= x_st;
      end
   end // always out_data_proc

   // ================= debug checks (simulation only) =================
`ifdef L1D_DEBUG
   // pragma translate_off
   initial begin : param_chk_proc
      if ((1 << SET_W) != NS || (1 << $clog2(NW)) != NW ||
          (1 << DSH) != DIST || NW < 4 || FILT_DEPTH < 2)
         $display("L1D PF_STRIDE %m ERROR parameter assumption A2");
      if (PA_W > ADDR_W || PA_W <= PAGE_W || KEY_W <= LIP_W)
         $display("L1D PF_STRIDE %m ERROR parameter assumption A3");
   end
   always @(posedge clk) begin : watch_proc
      if (!rst && r_valid && ((r_way == WAY_ZERO) ||
          ((r_way & (r_way - 1'b1)) != WAY_ZERO)))
         $display("[%0t] L1D PF_STRIDE %m ERROR way not one-hot", $time);
      // the 3-cycle mask keeps event 1 off event 4's way when the m4
      // branch is taken (select_proc); event 1 with event 4's key is
      // r_m[1] and wins first
      if (!rst && r_valid && r_m[4] && !r_m[1] && !r_m[2] && !r_m[3] &&
          r_ss1 && ((g_way & d_way) != WAY_ZERO))
         $display("[%0t] L1D PF_STRIDE %m ERROR event 4's way replaced",
                  $time);
   end
   // pragma translate_on
`endif

endmodule

`default_nettype wire
`endif // L1D_PF_STRIDE_V
