# cache — ASAP7 timing (4-cycle-hit pipeline)

Post-synthesis estimates: ideal clock, no wires, zero I/O delays, 5% clock uncertainty. SRAM models assume data clock-to-output 180 ps and tag 150 ps (defaults of the run); these are not foundry measurements.

Parents include their flattened children. Combinational modules use a virtual clock; their equivalent GHz value is not a measured operating clock. Generic blocks use their own default parameters; the parent rows include instance-specific sizes. The SRAM row uses the dual-port data-macro timing model (assumed, see macros/macro_assumptions.txt), not a synthesized flip-flop RAM.

Effective critical period = target period − worst setup slack; equivalent GHz = 1000 / effective critical period. Setup and uncertainty are included. Passing the workflow means measurement completed, not that every target was met. No post-route frequency or functional/latency compliance is established.

| Module | Target ps | Clock | Worst slack ps | Effective critical ps | Equivalent GHz | Area um² | Result |
|---|---:|---|---:|---:|---:|---:|---|
| l1d_add_eq | 250 | virtual | 126.783 | 123.217 | 8.1158 | 6 | MEETS |
| l1d_age_matrix | 250 | real | 67.258 | 182.742 | 5.4722 | 41 | MEETS |
| l1d_amo_alu | 250 | virtual | -205.178 | 455.178 | 2.1969 | 156 | MISSES |
| l1d_arb_rr | 250 | real | 106.757 | 143.243 | 6.9811 | 4 | MEETS |
| l1d_bank_order | 250 | real | -94.120 | 344.120 | 2.9060 | 45 | MISSES |
| l1d_bank_r | 250 | real | — | — | — | — | SYNTH_FAIL |
| l1d_bank_s0 | 250 | real | -613.020 | 863.020 | 1.1587 | 2729 | MISSES |
| l1d_bank_s1 | 250 | real | -435.017 | 685.017 | 1.4598 | 5192 | MISSES |
| l1d_bank_s2 | 250 | real | -324.195 | 574.195 | 1.7416 | 1866 | MISSES |
| l1d_bank_s3 | 250 | real | 95.455 | 154.545 | 6.4706 | 274 | MEETS |
| l1d_book_select | 250 | real | 97.629 | 152.371 | 6.5629 | 12 | MEETS |
| l1d_cam | 250 | virtual | 136.671 | 113.329 | 8.8239 | 29 | MEETS |
| l1d_data_array | 250 | real | — | — | — | — | SYNTH_FAIL |
| l1d_ecc_enc | 250 | virtual | 62.391 | 187.609 | 5.3302 | 22 | MEETS |
| l1d_ecc_fix | 250 | virtual | 136.658 | 113.342 | 8.8229 | 21 | MEETS |
| l1d_ecc_syn | 250 | virtual | -163.731 | 413.731 | 2.4170 | 25 | MISSES |
| l1d_fifo | 250 | real | 32.150 | 217.850 | 4.5903 | 31 | MEETS |
| l1d_ks_add | 250 | virtual | 71.437 | 178.563 | 5.6003 | 9 | MEETS |
| l1d_ks_add_q | 250 | real | 32.064 | 217.936 | 4.5885 | 16 | MEETS |
| l1d_l2_port | 250 | real | -295.706 | 545.706 | 1.8325 | 1583 | MISSES |
| l1d_mesi_next | 250 | virtual | 113.695 | 136.305 | 7.3365 | 5 | MEETS |
| l1d_meta_flops | 250 | real | 59.565 | 190.435 | 5.2511 | 759 | MEETS |
| l1d_pf_sms | 250 | real | -325.943 | 575.943 | 1.7363 | 7589 | MISSES |
| l1d_pf_sms_core | 250 | real | -341.459 | 591.459 | 1.6907 | 6499 | MISSES |
| l1d_pf_sms_expand | 250 | real | -179.772 | 429.772 | 2.3268 | 391 | MISSES |
| l1d_pf_sms_fifo | 250 | real | -122.710 | 372.710 | 2.6831 | 241 | MISSES |
| l1d_pf_sms_front | 250 | real | -391.104 | 641.104 | 1.5598 | 807 | MISSES |
| l1d_pf_stream | 250 | real | -311.588 | 561.588 | 1.7807 | 1162 | MISSES |
| l1d_pf_stride | 250 | real | -43.382 | 293.382 | 3.4085 | 2972 | MISSES |
| l1d_plru | 250 | virtual | 206.824 | 43.176 | 23.1610 | 1 | MEETS |
| l1d_port_front | 250 | real | -410.777 | 660.777 | 1.5134 | 1700 | MISSES |
| l1d_prefetch_unit | 250 | real | -394.285 | 644.285 | 1.5521 | 12201 | MISSES |
| l1d_pri_enc | 250 | virtual | 209.983 | 40.017 | 24.9894 | 1 | MEETS |
| l1d_req_tracker | 250 | real | -559.558 | 809.558 | 1.2352 | 2119 | MISSES |
| l1d_reservation_unit | 250 | real | 19.544 | 230.456 | 4.3392 | 142 | MEETS |
| l1d_resp_gen | 250 | virtual | 53.621 | 196.379 | 5.0922 | 405 | MEETS |
| l1d_resp_merge | 250 | real | -65.174 | 315.174 | 3.1729 | 903 | MISSES |
| l1d_s1_lookup | 250 | virtual | 85.110 | 164.890 | 6.0646 | 41 | MEETS |
| l1d_sram_tdp | 250 | real | 57.500 | 192.500 | 5.1948 | 2404 | MEETS |
| l1d_tag_array | 250 | real | — | — | — | — | SYNTH_FAIL |

Raw CSV, input hashes/revision, constraints, synthesis/STA logs, library selections and detailed worst/hold paths are saved in the job artifacts. Review check_setup reports for unconstrained or other coverage warnings.
