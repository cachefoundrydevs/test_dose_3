// ============================================================================
// l1d_maint.v  -  flush / clean sequencer of one bank
// ============================================================================
// IN SIMPLE WORDS
//   The CPU can ask for a CLEAN ("write every dirty line to memory, but keep the lines") or a FLUSH ("write every
//   dirty line to memory, then throw ALL lines away"). l1d_top starts every bank (maint_start) and waits until all
//   banks have pulsed maint_done. This file is the walking robot of ONE bank: it visits every {set, way} in turn
//   and asks the pipeline to examine that line.
//
//   clean: every dirty line is written to memory; lines stay valid (now clean, state E)
//   flush: every dirty line is written to memory; then every line is invalid (state I)
//
// THE SEQUENCE   (a state machine with named states; the walker's position is set_r / way_r)
//   M_IDLE   nothing to do                                       --maint_start--> M_WAIT
//   M_WAIT   wait until the bank has no request, MSHR or non-cacheable work left (bank_idle).
//            The CPU sends no requests during a flush / clean.   --bank_idle--> M_DRAIN
//   M_DRAIN  empty the victim cache: clean lines are dropped, dirty lines written back; wait until every slot is free
//            (victim_cache_drain = 1)                            --victim_cache_empty--> M_REQ
//   M_REQ    offer a MAINTENANCE TOKEN for (set_r, way_r) to Priority Select (token_valid = 1)   --token_granted--> M_RUN
//   M_RUN    the token is in the pipeline:
//              Stage 1 reads the way's state; a dirty line is read from the data array; the state becomes
//                      clean (E) or invalid (I)
//              Stage 2 puts a dirty line into a free victim-cache slot as WBPEND, so the victim cache writes it to memory
//            when Stage 2 reports (s2_result_valid):
//              s2_result_held = 1 (no free victim slot)  --> back to M_REQ for the SAME set and way (retry)
//              otherwise next way (then next set); after the last way of the last set --> M_FINAL
//   M_FINAL  wait until every writeback has its B response (victim cache empty) --> maint_done pulse --> M_IDLE
//   It takes about 3 cycles per way plus the writebacks.
//
// WHAT KIND OF STATE MACHINE IS IT?   A Moore machine with 6 named states (outputs depend on the state only).
// TIMING OPT (M-1): the last-way / last-set compares and set_r + 1 are kept in flops (wlast_q, slast_q, set_inc_q); same cycles, same pins.
//
// PORTS (new name, old name, meaning)
//   maint_start                          (was start)                    start a flush or clean walk
//   maint_op_is_flush                    (was op)                       0 = clean, 1 = flush
//   victim_cache_empty                   (was vc_empty)                 every victim-cache slot is free
//   victim_cache_drain                   (was vc_drain)                 ask the victim cache to empty itself
//   token_valid                          (was tok_valid)                a maintenance token is offered to Priority Select
//   token_set                            (was tok_set)                  set of the token
//   token_way                            (was tok_way)                  way of the token
//   token_op_is_flush                    (was tok_op)                   0 = clean, 1 = flush
//   token_granted                        (was tok_gnt)                  Priority Select granted the token
//   s2_result_valid                      (was res_valid)                the token's Stage 2 result is available
//   s2_result_held                       (was res_held)                 no free victim slot: retry this way
//   maint_done                           (was done)                     one-cycle pulse: the walk is finished
//   maint_busy                           (was busy)                     a walk is running
//   bank_idle                                                           the bank has no request, MSHR or non-cacheable work left
//
// VERILOG WORDS USED IN THIS FILE
//   localparam [2:0] M_IDLE = 3'd0;   a named constant: the state numbers
//   case (ms) ... endcase             choose the action for the current state ms
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

module l1d_maint #(`L1D_PARAMS) (
    clk, rst,
    maint_start, maint_op_is_flush, bank_idle, victim_cache_empty, victim_cache_drain,
    token_valid, token_set, token_way, token_op_is_flush, token_granted,
    s2_result_valid, s2_result_held,
    maint_done, maint_busy
);
    `include "l1d_encodings.vh"
    `include "l1d_fields.vh"

    input  wire               clk;                        // clock: all registers change on its rising edge
    input  wire               rst;                        // synchronous reset, active high (control state goes to its idle value)
    input  wire               maint_start;                // start a flush or clean walk
    input  wire               maint_op_is_flush;          // 0 = clean, 1 = flush
    input  wire               bank_idle;                  // the bank has no request, MSHR or non-cacheable work left
    input  wire               victim_cache_empty;         // every victim-cache slot is free
    output wire               victim_cache_drain;         // ask the victim cache to empty itself
    // maintenance token
    output wire               token_valid;                // a maintenance token is offered to Priority Select
    output wire [INDEX_W-1:0] token_set;                  // set of the token
    output wire [WAY_W-1:0]   token_way;                  // way of the token
    output wire               token_op_is_flush;          // 0 = clean, 1 = flush
    input  wire               token_granted;              // Priority Select granted the token
    // result of the token (Stage 2)
    input  wire               s2_result_valid;            // the token's Stage 2 result is available
    input  wire               s2_result_held;             // no free victim slot: retry this way
    output reg                maint_done;                 // one-cycle pulse: the walk is finished
    output wire               maint_busy;                 // a walk is running

    // state numbers (see the header for what each state does)
    localparam [2:0] M_IDLE  = 3'd0;
    localparam [2:0] M_WAIT  = 3'd1;
    localparam [2:0] M_DRAIN = 3'd2;
    localparam [2:0] M_REQ   = 3'd3;   // token offered
    localparam [2:0] M_RUN   = 3'd4;   // token in the pipeline
    localparam [2:0] M_FINAL = 3'd5;

    // the last set and way: when the walker is there, the walk is finished
    localparam [INDEX_W-1:0] LAST_SET = SETS - 1;
    localparam [WAY_W-1:0]   LAST_WAY = WAYS - 1;

    reg [2:0]         ms;                                 // state of the sequencer (M_*)
    reg               op_r;                               // the requested operation, remembered at the start (clean or flush)
    reg [INDEX_W-1:0] set_r;                              // set the walker is at
    reg [WAY_W-1:0]   way_r;                              // way the walker is at, WAY_W = 2 bits
    // TIMING OPT (M-1, 5 GHz): the compares "way_r == LAST_WAY", "set_r == LAST_SET" and the incremented set "set_r + 1" are kept in flops,
    // loaded wherever set_r / way_r are loaded, so the enable of set_r (from s2_result_valid / s2_result_held) no longer waits for the
    // compares and the incrementer. Each flop equals its expression in every cycle (proof: formal check "maint" against the original).
    reg               wlast_q;                            // = (way_r == LAST_WAY)
    reg               slast_q;                            // = (set_r == LAST_SET)
    reg [INDEX_W-1:0] set_inc_q;                          // = set_r + 1 (wraps like set_r), INDEX_W = 6 bits
    localparam        WLAST0 = (LAST_WAY == {WAY_W{1'b0}});   // way 0 is the last way (WAYS = 1)
    localparam        SLAST0 = (LAST_SET == {INDEX_W{1'b0}}); // set 0 is the last set (SETS = 1)

    // ---- the state machine (clocked) ----
    always @(posedge clk) begin
        if (rst) begin
            ms    <= M_IDLE;
            op_r  <= MAINT_CLEAN;
            set_r <= {INDEX_W{1'b0}};
            way_r <= {WAY_W{1'b0}};
            wlast_q   <= WLAST0;
            slast_q   <= SLAST0;
            set_inc_q <= {INDEX_W{1'b0}} + 1'b1;
            maint_done  <= 1'b0;
        end else begin
            maint_done <= 1'b0;
            case (ms)
                M_IDLE: if (maint_start) begin
                    op_r  <= maint_op_is_flush;
                    set_r <= {INDEX_W{1'b0}};
                    way_r <= {WAY_W{1'b0}};
                    wlast_q   <= WLAST0;
                    slast_q   <= SLAST0;
                    set_inc_q <= {INDEX_W{1'b0}} + 1'b1;
                    ms    <= M_WAIT;
                end
                M_WAIT:  if (bank_idle) ms <= M_DRAIN;
                M_DRAIN: if (victim_cache_empty)  ms <= M_REQ;
                M_REQ:   if (token_granted)   ms <= M_RUN;
                M_RUN: if (s2_result_valid) begin
                    if (s2_result_held) begin
                        ms <= M_REQ;                         // same set / way again
                    end else if (wlast_q && slast_q) begin
                        ms <= M_FINAL;
                    end else begin
                        if (wlast_q) begin
                            way_r     <= {WAY_W{1'b0}};
                            set_r     <= set_inc_q;
                            wlast_q   <= WLAST0;
                            slast_q   <= (set_inc_q == LAST_SET);
                            set_inc_q <= set_inc_q + 1'b1;
                        end else begin
                            way_r     <= way_r + 1'b1;
                            wlast_q   <= ((way_r + 1'b1) == LAST_WAY);
                        end
                        ms <= M_REQ;
                    end
                end
                M_FINAL: if (victim_cache_empty) begin
                    maint_done <= 1'b1;
                    ms   <= M_IDLE;
                end
                default: ms <= M_IDLE;
            endcase
        end
    end

    // ---- outputs, read straight from the state ----
    assign victim_cache_drain  = (ms == M_DRAIN) || (ms == M_FINAL);
    assign token_valid = (ms == M_REQ);
    assign token_set   = set_r;
    assign token_way   = way_r;
    assign token_op_is_flush    = op_r;
    assign maint_busy      = (ms != M_IDLE);

// ---- simulation-only trace: state changes, held tokens, progress (every 16 sets), STUCK reports ----
`ifdef L1D_DEBUG
    // Debug trace (simulation only): sequencer states, held tokens, progress
    // (every 16 sets), STUCK reports.
    function [8*5-1:0] dbg_mname;
        input [2:0] s;
        begin
            case (s)
                M_IDLE:  dbg_mname = "IDLE";
                M_WAIT:  dbg_mname = "WAIT";
                M_DRAIN: dbg_mname = "DRAIN";
                M_REQ:   dbg_mname = "REQ";
                M_RUN:   dbg_mname = "RUN";
                default: dbg_mname = "FINAL";
            endcase
        end
    endfunction

    reg [2:0] dbg_ps;
    integer   dbg_age;
    initial begin dbg_ps = M_IDLE; dbg_age = 0; end

    always @(posedge clk) begin
        if (rst) begin
            dbg_ps  = M_IDLE;
            dbg_age = 0;
        end else begin
            if (ms != dbg_ps) begin
                if (!(dbg_ps == M_REQ && ms == M_RUN) && !(dbg_ps == M_RUN && ms == M_REQ))
                    $display("[%0t] L1D MAINT %m: %0s -> %0s (%s)", $time, dbg_mname(dbg_ps), dbg_mname(ms),
                             op_r == MAINT_FLUSH ? "flush" : "clean");
                dbg_ps  = ms;
                dbg_age = 0;
            end else if (ms != M_IDLE) begin
                dbg_age = dbg_age + 1;
                if (dbg_age % `L1D_DBG_WD == 0)
                    $display("[%0t] L1D MAINT %m STUCK in %0s for %0d cycles  set=%0d way=%0d bank_idle=%b vc_empty=%b",
                             $time, dbg_mname(ms), dbg_age, set_r, way_r, bank_idle, victim_cache_empty);
            end
            if (ms == M_RUN && s2_result_valid && s2_result_held)
                $display("[%0t] L1D MAINT %m set %0d way %0d held (no victim-cache slot), retry", $time, set_r, way_r);
            if (ms == M_RUN && s2_result_valid && !s2_result_held && way_r == LAST_WAY && (set_r % 16) == 15)
                $display("[%0t] L1D MAINT %m walked sets 0..%0d", $time, set_r);
            if (maint_done)
                $display("[%0t] L1D MAINT %m done", $time);
        end
    end
`endif

endmodule

`default_nettype wire
