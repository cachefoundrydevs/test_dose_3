// ==========================================================================
// l1d_pf_sms_expand.v - SMS expander: pattern -> one line request per cycle
// ==========================================================================
// IN SIMPLE WORDS : When the core finds a learned footprint for a new
//   region it hands over (region, set of lines, trigger line). This
//   block keeps up to 4 such jobs in "slots" and turns each into
//   single line requests, nearest line after the trigger first, then
//   wrapping round the region. Every request takes a bank credit;
//   a bank with no credit waits. The cache returns a credit with its
//   feedback (issued, in cache, in flight or overwritten).
//   If feedback never comes (a bug elsewhere), a time-out gives all
//   credits back, so the wait is bounded (S-1).
// STAGES (each output is a flop, T-1):
//   X0  pick one slot (round robin)             -> sel_q
//   X1  find-first-set, clear that bit          -> x1_* regs
//   X2  credit check, output register           -> pf_* regs
//   Slot load from the core is combinational in the cycle of the core
//   decision (core S2) and registered in the slot (V-3.11, named).
// INSTANTIATED BY : l1d_pf_sms, instance u_expand, 1 copy.
//
// PARAMETERS:
//   name              default unit     meaning
//   LINE_ADDR_W       50      bits     physical line address
//   LIR_W             5       bits     line number inside a region
//   N_BANKS           4       banks    cache banks (credit counters)
//   EXPAND_SLOTS      4       slots    patterns issued at the same time
//   CREDIT_PER_BANK   2       count    requests without feedback per bank
//   CREDIT_WAIT_MAX   64      cycles   wait for a credit, then give up
//
// PORTS (default widths):
//   clk, rst          in  1    clock, synchronous reset (high)
//   pf_enable         in  1    1 = on; low empties the block
//   load_valid        in  1    core hands over a pattern (combinational)
//   load_region       in  45   region number
//   load_mask         in  32   lines still to fetch (trigger bit cleared)
//   load_line         in  5    trigger line (order starts after it)
//   pf_ready          in  1    the shared port can take a request
//   fb_valid          in  1    feedback for a prefetch request
//   fb_addr           in  50   its line address (bank = low bits)
//   fb_result         in  2    its result (all results free a credit)
//   pf_valid          out 1    prefetch request (registered)
//   pf_line_addr      out 50   physical line address; bank = [1:0]
//
// WORKED EXAMPLE (README section 10, region 0x2468B, trigger line 0,
//   mask {3,6}): X0 picks the slot; X1 takes line 3 (bank 3), then line
//   6 (bank 2); X2 sends line addresses {0x2468B,3} and {0x2468B,6},
//   one per cycle, each taking a credit of its bank.
//
// ASSUMPTIONS (V-9.2): A1 N_BANKS and EXPAND_SLOTS >= 2. A2 the bank of
//   a line is its low log2(N_BANKS) bits. A3 every request gets exactly
//   one feedback (or the time-out frees the credit). A4 a load into a
//   full block is dropped (a pattern is a hint).
// COMBINATIONAL INPUT PATH: load_* arrive combinationally from the core.
// ==========================================================================
`ifndef L1D_PF_SMS_EXPAND_V
`define L1D_PF_SMS_EXPAND_V
`default_nettype none

module l1d_pf_sms_expand #(
    // ---- parameters first, each with its default (V-7.10) ----
    parameter LINE_ADDR_W    = 50, // default 50 bits: line address
    parameter LIR_W          = 5,  // default 5 bits: line in region
    parameter N_BANKS        = 4,  // default 4 banks
    parameter EXPAND_SLOTS   = 4,  // default 4 slots
    parameter CREDIT_PER_BANK = 2, // default 2: credits per bank
    parameter CREDIT_WAIT_MAX = 64 // default 64 cycles: credit wait
) (
    // ---- clock and reset ----
    input  wire                                 clk,          // clock
    input  wire                                 rst,          // sync reset
    // ---- control ----
    input  wire                                 pf_enable,    // 1 = on
    // ---- pattern from the core (combinational) ----
    input  wire                                 load_valid,   // pattern
    input  wire [LINE_ADDR_W-LIR_W-1:0]         load_region,  // 45 bits
    input  wire [(1<<LIR_W)-1:0]                load_mask,    // 32 bits
    input  wire [LIR_W-1:0]                     load_line,    // 5 bits
    // ---- shared prefetch port ----
    input  wire                                 pf_ready,     // accept
    output wire                                 pf_valid,     // request
    output wire [LINE_ADDR_W-1:0]               pf_line_addr, // 50 bits
    // ---- cache feedback ----
    input  wire                                 fb_valid,     // feedback
    input  wire [LINE_ADDR_W-1:0]               fb_addr,      // 50 bits
    input  wire [1:0]                           fb_result     // unused
);

    // ---- local constants (value at the default) ----
    localparam REGION_NUM_W = LINE_ADDR_W - LIR_W;      // 45 bits
    localparam PATTERN_W    = (1 << LIR_W);             // 32 bits
    localparam BANK_W       = $clog2(N_BANKS);          // 2 bits
    localparam CRED_W       = $clog2(CREDIT_PER_BANK + 1); // 2 bits
    localparam WAIT_W       = $clog2(CREDIT_WAIT_MAX + 1); // 7 bits
    localparam [CRED_W-1:0] CREDIT_V  = CREDIT_PER_BANK;   // 2
    localparam [WAIT_W-1:0] WAIT_MAX_V = CREDIT_WAIT_MAX;  // 64

    // ---- slots (slot s at [s*W +: W]) ----
    reg  [EXPAND_SLOTS-1:0]               slot_vld_q;    // 4: in use
    reg  [EXPAND_SLOTS*REGION_NUM_W-1:0]  slot_region_q; // 4 x 45
    reg  [EXPAND_SLOTS*PATTERN_W-1:0]     slot_mask_q;   // 4 x 32: todo
    reg  [EXPAND_SLOTS*PATTERN_W-1:0]     slot_high_q;   // 4 x 32: lines
                                                         //   above trigger
    // ---- load into a free slot ----
    reg  [EXPAND_SLOTS-1:0]               free_one;      // 4: slot to use
    reg                                   free_seen;     // 1: loop flag
    reg  [PATTERN_W-1:0]                  load_high;     // 32: bits > line
    wire                                  load_ok;       // 1: load stored
    // ---- X0 pick ----
    reg  [EXPAND_SLOTS-1:0]               rr_mask_q;     // 4: after last
    reg  [EXPAND_SLOTS-1:0]               sel_q;         // 4: picked slot
    reg                                   sel_vld_q;     // 1: slot picked
    reg  [EXPAND_SLOTS-1:0]               pick_high;     // 4: after last
    reg  [EXPAND_SLOTS-1:0]               pick_base;     // 4: search set
    reg  [EXPAND_SLOTS-1:0]               pick_one;      // 4: chosen
    reg  [EXPAND_SLOTS-1:0]               pick_above;    // 4: after chosen
    reg                                   pick_seen;     // 1: loop flag
    // ---- X1 find first set ----
    reg                                   rd_vld;        // 1: slot valid
    reg  [REGION_NUM_W-1:0]               rd_region;     // 45
    reg  [PATTERN_W-1:0]                  rd_mask;       // 32
    reg  [PATTERN_W-1:0]                  rd_high;       // 32
    reg  [PATTERN_W-1:0]                  ffs_in;        // 32: search set
    reg  [PATTERN_W-1:0]                  ffs_one;       // 32: lowest bit
    reg  [LIR_W-1:0]                      ffs_idx;       // 5: its number
    reg                                   ffs_seen;      // 1: loop flag
    wire                                  x1_take;       // 1: X1 makes one
    wire                                  allow_x1;      // 1: X1 may fire
    reg                                   x1_vld_q;      // 1: request held
    reg  [REGION_NUM_W-1:0]               x1_region_q;   // 45
    reg  [LIR_W-1:0]                      x1_line_q;     // 5
    // ---- X2 credit and output ----
    reg  [N_BANKS*CRED_W-1:0]             credit_q;      // 4 x 2: in use
    reg  [WAIT_W-1:0]                     wait_q;        // 7: credit wait
    reg                                   credit_ok;     // 1: bank has room
    wire [BANK_W-1:0]                     x1_bank;       // 2: bank of x1
    wire                                  go;            // 1: send x1
    reg                                   out_vld_q;     // 1: pf_valid
    reg  [LINE_ADDR_W-1:0]                out_addr_q;    // 50: pf address
    integer                               slot_idx;      // walks slots
    integer                               bank_idx;      // walks banks

    // ======================================================================
    // load: first free slot, one-hot
    // ======================================================================
    always @* begin : load_proc
        reg [LIR_W-1:0] bit_number; // bit index as a vector
        integer bit_idx;            // walks bits
        integer slot_idx;           // walks slots
        free_one  = {EXPAND_SLOTS{1'b0}};
        free_seen = 1'b0;
        for (slot_idx = 0; slot_idx < EXPAND_SLOTS; slot_idx = slot_idx + 1) begin
            if (!slot_vld_q[slot_idx] && !free_seen) begin
                free_one[slot_idx] = 1'b1;
                free_seen          = 1'b1;
            end
        end // for slot_idx
        // bit k of load_high is 1 when line k is after the trigger line
        load_high = {PATTERN_W{1'b0}};
        for (bit_idx = 0; bit_idx < PATTERN_W; bit_idx = bit_idx + 1) begin
            bit_number = bit_idx;
            load_high[bit_idx] = (bit_number > load_line);
        end // for bit_idx
    end // always load_proc
    assign load_ok = load_valid & pf_enable & free_seen
                     & (|load_mask);

    // ======================================================================
    // X0: round-robin pick of a slot that has lines left
    // ======================================================================
    always @* begin : pick_proc
        integer slot_idx;           // walks slots
        pick_high = slot_vld_q & rr_mask_q;
        pick_base = (|pick_high) ? pick_high : slot_vld_q;
        pick_one  = {EXPAND_SLOTS{1'b0}};
        pick_seen = 1'b0;
        for (slot_idx = 0; slot_idx < EXPAND_SLOTS; slot_idx = slot_idx + 1) begin
            if (pick_base[slot_idx] && !pick_seen) begin
                pick_one[slot_idx] = 1'b1;
                pick_seen          = 1'b1;
            end
        end // for slot_idx
        pick_above = {EXPAND_SLOTS{1'b0}};
        pick_seen  = 1'b0;
        for (slot_idx = 0; slot_idx < EXPAND_SLOTS; slot_idx = slot_idx + 1) begin
            pick_above[slot_idx] = pick_seen;
            pick_seen            = pick_seen | pick_one[slot_idx];
        end // for slot_idx
    end // always pick_proc

    // ======================================================================
    // X1: read the picked slot, find the first line after the trigger
    // ======================================================================
    always @* begin : read_proc
        reg [LIR_W-1:0] bit_number; // bit index as a vector
        integer bit_idx;            // walks bits
        integer slot_idx;           // walks slots
        rd_vld    = 1'b0;
        rd_region = {REGION_NUM_W{1'b0}};
        rd_mask   = {PATTERN_W{1'b0}};
        rd_high   = {PATTERN_W{1'b0}};
        for (slot_idx = 0; slot_idx < EXPAND_SLOTS; slot_idx = slot_idx + 1) begin
            rd_vld    = rd_vld | (slot_vld_q[slot_idx] & sel_q[slot_idx]);
            rd_region = rd_region | (slot_region_q[slot_idx*REGION_NUM_W +: REGION_NUM_W]
                                     & {REGION_NUM_W{sel_q[slot_idx]}});
            rd_mask   = rd_mask | (slot_mask_q[slot_idx*PATTERN_W +: PATTERN_W]
                                   & {PATTERN_W{sel_q[slot_idx]}});
            rd_high   = rd_high | (slot_high_q[slot_idx*PATTERN_W +: PATTERN_W]
                                   & {PATTERN_W{sel_q[slot_idx]}});
        end // for slot_idx
        // lines after the trigger first; if none, the lines before it
        ffs_in   = (|(rd_mask & rd_high)) ? (rd_mask & rd_high) : rd_mask;
        ffs_one  = {PATTERN_W{1'b0}};
        ffs_idx  = {LIR_W{1'b0}};
        ffs_seen = 1'b0;
        for (bit_idx = 0; bit_idx < PATTERN_W; bit_idx = bit_idx + 1) begin
            bit_number = bit_idx;
            if (ffs_in[bit_idx] && !ffs_seen) begin
                ffs_one[bit_idx] = 1'b1;
                ffs_idx          = bit_number;
                ffs_seen         = 1'b1;
            end
        end // for bit_idx
    end // always read_proc

    assign allow_x1 = ~x1_vld_q | go;
    assign x1_take  = sel_vld_q & rd_vld & (|rd_mask) & allow_x1;

    // ======================================================================
    // X2: bank credit check and the output register
    // ======================================================================
    assign x1_bank = x1_line_q[BANK_W-1:0];
    always @* begin : credit_ok_proc
        integer bank_idx;           // walks banks
        credit_ok = 1'b0;
        for (bank_idx = 0; bank_idx < N_BANKS; bank_idx = bank_idx + 1) begin
            if (x1_bank == bank_idx) begin
                credit_ok = (credit_q[bank_idx*CRED_W +: CRED_W] < CREDIT_V);
            end
        end // for bank_idx
    end // always credit_ok_proc
    assign go = x1_vld_q & credit_ok & (~out_vld_q | pf_ready) & pf_enable;

    // ======================================================================
    // clocked state
    // ======================================================================
    always @(posedge clk) begin : state_proc
        if (rst || !pf_enable) begin
            // flush: slots, pipeline and output (credits survive a
            // switch-off; reset also clears them)
            slot_vld_q <= {EXPAND_SLOTS{1'b0}};
            sel_vld_q  <= 1'b0;
            sel_q      <= {EXPAND_SLOTS{1'b0}};
            x1_vld_q   <= 1'b0;
            out_vld_q  <= 1'b0;
            rr_mask_q  <= {EXPAND_SLOTS{1'b1}};
            wait_q     <= {WAIT_W{1'b0}};
            if (rst) begin
                credit_q <= {(N_BANKS*CRED_W){1'b0}};
            end
            else begin
                // a request held in the output register (not taken this
                // cycle) is thrown away: give its credit back
                for (bank_idx = 0; bank_idx < N_BANKS; bank_idx = bank_idx + 1) begin
                    if (out_vld_q && !pf_ready
                        && (out_addr_q[BANK_W-1:0] == bank_idx)
                        && (credit_q[bank_idx*CRED_W +: CRED_W] != {CRED_W{1'b0}})) begin
                        credit_q[bank_idx*CRED_W +: CRED_W] <=
                            credit_q[bank_idx*CRED_W +: CRED_W] - 1'b1;
                    end
                end // for bank_idx
            end // else (switch-off)
        end
        else begin
            // ---- load a pattern into the free slot ----
            for (slot_idx = 0; slot_idx < EXPAND_SLOTS; slot_idx = slot_idx + 1) begin
                if (load_ok && free_one[slot_idx]) begin
                    slot_vld_q[slot_idx] <= 1'b1;
                    slot_region_q[slot_idx*REGION_NUM_W +: REGION_NUM_W] <= load_region;
                    slot_mask_q[slot_idx*PATTERN_W +: PATTERN_W]         <= load_mask;
                    slot_high_q[slot_idx*PATTERN_W +: PATTERN_W]         <= load_high;
                end
            end // for slot_idx

            // ---- X0: pick (holds while X1 cannot take it) ----
            if (!sel_vld_q || allow_x1) begin
                sel_vld_q <= |slot_vld_q;
                sel_q     <= pick_one;
                if (|slot_vld_q) begin
                    rr_mask_q <= pick_above;
                end
            end // if (pick)

            // ---- X1: clear the line, hold the request ----
            if (allow_x1) begin
                x1_vld_q    <= x1_take;
                x1_region_q <= rd_region;
                x1_line_q   <= ffs_idx;
            end // if (allow_x1)
            for (slot_idx = 0; slot_idx < EXPAND_SLOTS; slot_idx = slot_idx + 1) begin
                if (x1_take && sel_q[slot_idx]) begin
                    slot_mask_q[slot_idx*PATTERN_W +: PATTERN_W] <= rd_mask & ~ffs_one;
                    if ((rd_mask & ~ffs_one) == {PATTERN_W{1'b0}}) begin
                        slot_vld_q[slot_idx] <= 1'b0;
                    end
                end
            end // for slot_idx

            // ---- X2: output register ----
            if (go) begin
                out_vld_q  <= 1'b1;
                out_addr_q <= {x1_region_q, x1_line_q};
            end
            else if (out_vld_q && pf_ready) begin
                out_vld_q <= 1'b0;
            end // else if (taken)

            // ---- credits: +1 on a send, -1 on feedback (not below 0) ----
            for (bank_idx = 0; bank_idx < N_BANKS; bank_idx = bank_idx + 1) begin
                if ((go && (x1_bank == bank_idx))
                    && !(fb_valid && (fb_addr[BANK_W-1:0] == bank_idx)
                         && (credit_q[bank_idx*CRED_W +: CRED_W] != {CRED_W{1'b0}}))) begin
                    credit_q[bank_idx*CRED_W +: CRED_W] <=
                        credit_q[bank_idx*CRED_W +: CRED_W] + 1'b1;
                end
                else if (!(go && (x1_bank == bank_idx))
                         && fb_valid && (fb_addr[BANK_W-1:0] == bank_idx)
                         && (credit_q[bank_idx*CRED_W +: CRED_W] != {CRED_W{1'b0}})) begin
                    credit_q[bank_idx*CRED_W +: CRED_W] <=
                        credit_q[bank_idx*CRED_W +: CRED_W] - 1'b1;
                end
            end // for bank_idx

            // ---- time-out: a request waiting too long for a credit ----
            if (x1_vld_q && !credit_ok) begin
                if (wait_q == WAIT_MAX_V) begin
                    credit_q <= {(N_BANKS*CRED_W){1'b0}};
                    wait_q   <= {WAIT_W{1'b0}};
                end
                else begin
                    wait_q <= wait_q + 1'b1;
                end
            end
            else begin
                wait_q <= {WAIT_W{1'b0}};
            end // else (no wait)
        end // else (run)
    end // always state_proc

    assign pf_valid     = out_vld_q;
    assign pf_line_addr = out_addr_q;

    // feedback result is not needed: every result frees one credit
    wire unused_ok; // 1: keeps lint quiet about fb_result
    assign unused_ok = &{1'b0, fb_result};

endmodule

`default_nettype wire
`endif
