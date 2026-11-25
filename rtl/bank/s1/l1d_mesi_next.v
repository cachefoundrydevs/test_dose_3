// ============================================================================
// l1d_mesi_next.v  -  MESI state logic: "what is the next state of this line?"  (new in v2)
// ============================================================================
// IN SIMPLE WORDS
//   Every cache line is in one of four states: I (invalid), S (shared, clean, others may have it),
//   E (exclusive, clean, only we have it), M (modified, dirty, only we have it).
//   Something happens to a line (an "event": a load hits it, a store hits it, a snoop arrives ...).
//   Given the event and the current state, this block says what the state becomes and whether a few
//   special things must happen. EVERY state change of the bank is decided here, so the whole coherence
//   behaviour is ONE table (PLAN_v2.md section 4). One copy per bank, used in Stage 1.
//
//   event            I      S                 E          M
//   load hit         -      S                 E          M
//   write hit        -      I + upgrade       M          M          (store, SC, AMO write)
//   unique hit       -      I + upgrade       E          M          (LR, AMO read)
//   install          install_state (any)      (any)      (any)      (a refill / swap puts a line in)
//   snoop read       -      S                 S          S + hand   (another core reads the line)
//   snoop invalidate -      I                 I          I + hand   (another core writes the line)
//   clean            -      S                 E          E          (dirty data is written back by the walk)
//   flush            -      I                 I          I
//
//   upgrade_needed (upg)  = a store-class request needs E or M but the line is only S. The copy is dropped
//                           and the request takes the normal MISS path with a UNIQUE read, so no stale S copy
//                           can survive the write ("upgrade rule").
//   snoop_hands_over_data (hand) = the line was M: the snoop answer carries the dirty data and the receiver
//                           writes it to memory.
//   state_write_enable    = 1 when the state flops must really be written. A line that is I stays I for every
//                           event except an install, so it needs no write.
//
// WHAT KIND OF LOGIC IS IT?
//   Combinational (no clock, no state): it is the next-state logic of the 256 per-line MESI machines (64 sets x
//   4 ways); the state register is l1d_meta_flops. upgrade_needed and snoop_hands_over_data are Mealy-style
//   outputs (they depend on the current state AND the event).
//
// PORTS (new name, old name, meaning)
//   event_code                           (was ev)                       what happens to the line (EV_* in l1d_encodings.vh)
//   current_state                        (was cur)                      state of the way now
//   install_state                        (was inst)                     state to install (EV_INSTALL only)
//   next_state                           (was nxt)                      state of the way after the event
//   state_write_enable                   (was we)                       1 = the state must be written
//   upgrade_needed                       (was upg)                      a store-class request hit an S line: drop it and fetch unique
//   snoop_hands_over_data                (was hand)                     a snoop hit an M line: the answer carries the data
//
// VERILOG WORDS USED IN THIS FILE
//   case (event_code) ... endcase   choose the branch for the event number (EV_* names, l1d_encodings.vh)
//   default: ;                      "do nothing for every other event" (the outputs keep the values set at the top)
//   next_state = current_state;     the defaults are written first and a branch overrides them: no latches
// ============================================================================

`default_nettype none
`include "l1d_defines.vh"

module l1d_mesi_next (
    input  wire [3:0] event_code,                         // what happens to the line (EV_* in l1d_encodings.vh)
    input  wire [1:0] current_state,                      // state of the way now
    input  wire [1:0] install_state,                      // state to install (EV_INSTALL only)
    output reg  [1:0] next_state,                         // state of the way after the event
    output reg        state_write_enable,                 // 1 = the state must be written
    output reg        upgrade_needed,                     // a store-class request hit an S line: drop it and fetch unique
    output reg        snoop_hands_over_data               // a snoop hit an M line: the answer carries the data
);
    `include "l1d_encodings.vh"

    // ---- the defaults: "nothing changes". Every branch below only overrides what it must change. ----
    always @* begin
        next_state  = current_state;
        state_write_enable   = 1'b0;
        upgrade_needed  = 1'b0;
        snoop_hands_over_data = 1'b0;
        case (event_code)
            // store / SC / AMO write hit: E becomes M (now dirty); S is dropped and re-fetched unique (the upgrade rule); M stays M
            EV_WRITE_HIT: begin
                if (current_state == ST_E) begin
                    next_state = ST_M; state_write_enable = 1'b1;
                end else if (current_state == ST_S) begin
                    next_state = ST_I; state_write_enable = 1'b1; upgrade_needed = 1'b1;
                end
            end
            // LR / AMO read hit: E and M stay; S is dropped and re-fetched unique
            EV_UNIQ_HIT: begin
                if (current_state == ST_S) begin
                    next_state = ST_I; state_write_enable = 1'b1; upgrade_needed = 1'b1;
                end
            end
            // a refill or swap puts a line in: the state comes from the MSHR entry (install_state)
            EV_INSTALL: begin
                next_state = install_state; state_write_enable = 1'b1;
            end
            // another core reads: our M / E copy becomes S; an M line hands its dirty data over
            EV_SNP_READ: begin
                if (current_state != ST_I && current_state != ST_S) begin
                    next_state  = ST_S; state_write_enable = 1'b1;
                    snoop_hands_over_data = (current_state == ST_M);
                end
            end
            // another core writes: our copy is thrown away; an M line hands its dirty data over
            EV_SNP_INV: begin
                if (current_state != ST_I) begin
                    next_state  = ST_I; state_write_enable = 1'b1;
                    snoop_hands_over_data = (current_state == ST_M);
                end
            end
            // clean walk: a dirty line is written back by the walk, so M becomes E
            EV_CLEAN: begin
                if (current_state == ST_M) begin
                    next_state = ST_E; state_write_enable = 1'b1;
                end
            end
            // flush walk: every valid line becomes invalid
            EV_FLUSH: begin
                if (current_state != ST_I) begin
                    next_state = ST_I; state_write_enable = 1'b1;
                end
            end
            default: ;                    // EV_NONE, EV_LOAD_HIT: no change
        endcase
    end

endmodule

`default_nettype wire
