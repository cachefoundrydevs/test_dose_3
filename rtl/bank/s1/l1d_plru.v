// ============================================================================
// l1d_plru.v  -  tree pseudo-LRU for ONE set (which way to throw out next?)
// ============================================================================
// IN SIMPLE WORDS
//   A set has WAYS (4) places for lines. When a new line must come in and every way is full, one line has to go.
//   True LRU (throw out the least recently used) needs a lot of bookkeeping. Tree PLRU is a cheap approximation:
//   WAYS-1 (3) bits form a small decision tree. Each bit answers "which half was used LESS recently?"
//
//                    node 0 (root)          bit = 0: left half (ways 0,1) less recent
//                   /            \          bit = 1: right half (ways 2,3) less recent
//             node 1              node 2    node 1 chooses between way 0 and 1
//             /    \              /    \    node 2 chooses between way 2 and 3
//          way 0  way 1       way 2  way 3
//
//   victim_way     : walk from the root, always following the bits -> the way to replace
//   plru_bits_next : the bits after an access to accessed_way: every node on the path to that way is set to
//                    point AWAY from it
//   Heap numbering: the children of node n are 2n+1 (left) and 2n+2 (right).
//
//   Worked example (4 ways, bits written b0 b1 b2, start 000):
//        access way 0 -> 110     access way 1 -> 100     access way 2 -> 001     access way 3 -> 000
//        access way 1 -> 100  and now victim_way = walk: b0 = 1 (go right), b2 = 0 (go left) = way 2.
//   True LRU would have chosen way 0 here (never touched again), PLRU chose way 2 - that is the price of the shortcut.
//
// WHAT KIND OF LOGIC IS IT?
//   Combinational (no clock): it is the next-state and output logic of the 64 per-set PLRU machines. The bits
//   themselves live in l1d_meta_flops. WAYS must be a power of two, at least 2 (not checked).
//
// PORTS (new name, old name, meaning)
//   plru_bits                            (was bits)                     the PLRU tree bits of the set
//   accessed_way                         (was acc_way)                  the way that is being accessed
//   victim_way                           (was victim)                   way the PLRU tree points to
//   plru_bits_next                       (was bits_new)                 PLRU tree bits after the access
//
// VERILOG WORDS USED IN THIS FILE
//   integer l, node, dir   loop counters / the tree position; the loops are unrolled when the logic is built
//   (victim_way << 1) | d  shift the walk result left by one bit and add the new bit d at the bottom
//   dir[WAY_W-1:0]         the low WAY_W bits of the integer dir
// ============================================================================

`default_nettype none

module l1d_plru #(
    parameter WAYS  = 4,
    parameter WAY_W = $clog2(WAYS),
    parameter NB    = WAYS - 1
) (
    input  wire [NB-1:0]    plru_bits,                    // the PLRU tree bits of the set
    input  wire [WAY_W-1:0] accessed_way,                 // the way that is being accessed
    output reg  [WAY_W-1:0] victim_way,                   // way the PLRU tree points to
    output reg  [NB-1:0]    plru_bits_next                // PLRU tree bits after the access
);

    // l = tree level, node = current tree node, dir = 0 (left) or 1 (right)
    integer l;
    integer node;
    integer dir;

    // ---- victim walk: from the root, follow the bit of each node; the directions taken are the bits of the way number ----
    // victim walk
    always @* begin
        node   = 0;
        victim_way = {WAY_W{1'b0}};
        for (l = 0; l < WAY_W; l = l + 1) begin
            dir    = plru_bits[node] ? 1 : 0;
            victim_way = (victim_way << 1) | dir[WAY_W-1:0];
            node   = 2 * node + 1 + dir;
        end
    end

    // update after an access to acc_way (most significant way bit = root level)
    // ---- update after an access: along the path to accessed_way (its most significant bit is the root level) set each
    //      node to point to the OTHER half ----
    integer ul;
    integer unode;
    integer udir;
    always @* begin
        unode    = 0;
        plru_bits_next = plru_bits;
        for (ul = 0; ul < WAY_W; ul = ul + 1) begin
            udir            = accessed_way[WAY_W-1-ul] ? 1 : 0;
            plru_bits_next[unode] = (udir == 0);     // point to the other half
            unode           = 2 * unode + 1 + udir;
        end
    end

endmodule

`default_nettype wire
