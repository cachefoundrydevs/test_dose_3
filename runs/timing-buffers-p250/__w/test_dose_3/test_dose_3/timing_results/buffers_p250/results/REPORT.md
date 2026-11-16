# cache — ASAP7 timing (4-cycle-hit pipeline)

Post-synthesis estimates: ideal clock, no wires, zero I/O delays, 5% clock uncertainty. SRAM models assume data clock-to-output 180 ps and tag 150 ps (defaults of the run); these are not foundry measurements.

Parents include their flattened children. Combinational modules use a virtual clock; their equivalent GHz value is not a measured operating clock. Generic blocks use their own default parameters; the parent rows include instance-specific sizes. The SRAM row uses the dual-port data-macro timing model (assumed, see macros/macro_assumptions.txt), not a synthesized flip-flop RAM.

Effective critical period = target period − worst setup slack; equivalent GHz = 1000 / effective critical period. Setup and uncertainty are included. Passing the workflow means measurement completed, not that every target was met. No post-route frequency or functional/latency compliance is established.

| Module | Target ps | Clock | Worst slack ps | Effective critical ps | Equivalent GHz | Area um² | Result |
|---|---:|---|---:|---:|---:|---:|---|
| l1d_maint | 250 | real | 63.693 | 186.307 | 5.3675 | 18 | MEETS |
| l1d_snoop_buf | 250 | real | 16.346 | 233.654 | 4.2798 | 167 | MEETS |
| l1d_snoop_ctrl | 250 | real | 32.156 | 217.844 | 4.5904 | 3117 | MEETS |
| l1d_vec_store_buf | 250 | real | 76.868 | 173.132 | 5.7759 | 695 | MEETS |

Raw CSV, input hashes/revision, constraints, synthesis/STA logs, library selections and detailed worst/hold paths are saved in the job artifacts. Review check_setup reports for unconstrained or other coverage warnings.
