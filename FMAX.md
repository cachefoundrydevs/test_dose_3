# Maximum frequency of every RTL module (one synthesis run per module)

ASAP7, post-synthesis, ideal clock (5 % uncertainty), no wires, zero I/O delay, assumed SRAM macros. A parent includes its flattened children.
Fmax = 1000 / (target period - worst setup slack). "flop to flop" uses register-to-register paths only; "all paths" includes input and output pin paths.

| Module | Clock | Target ps | Flop to flop: slack ps | Flop to flop: Fmax GHz | All paths: slack ps | All paths: Fmax GHz | Area um2 | Status |
|---|---|---:|---:|---:|---:|---:|---:|---|
| `l1d_add_eq` | none (combinational) | 250 | - | - | 126.8 | 8.12 | 6 | ok |
| `l1d_age_matrix` | clk | 250 | 170.2 | 12.53 | 67.3 | 5.47 | 41 | ok |
| `l1d_amo_alu` | none (combinational) | 250 | - | - | -205.2 | 2.20 | 156 | ok |
| `l1d_arb_rr` | clk | 250 | 106.8 | 6.98 | 106.8 | 6.98 | 4 | ok |
| `l1d_bank` | clk | 250 | - | - | - | - | - | SYNTH_FAIL |
| `l1d_bank_order` | clk | 250 | 39.7 | 4.76 | -94.1 | 2.91 | 45 | ok |
| `l1d_bank_r` | clk | 250 | - | - | - | - | - | SYNTH_FAIL |
| `l1d_bank_s0` | clk | 250 | -613.0 | 1.16 | -613.0 | 1.16 | 2729 | ok |
| `l1d_bank_s1` | clk | 250 | -358.5 | 1.64 | -435.0 | 1.46 | 5192 | ok |
| `l1d_bank_s2` | clk | 250 | - | - | -324.2 | 1.74 | 1866 | ok |
| `l1d_bank_s3` | clk | 250 | - | - | 95.5 | 6.47 | 274 | ok |
| `l1d_book_select` | clk | 250 | 97.6 | 6.56 | 97.6 | 6.56 | 12 | ok |
| `l1d_cam` | none (combinational) | 250 | - | - | 136.7 | 8.82 | 29 | ok |
| `l1d_data_array` | clk | 250 | - | - | - | - | - | SYNTH_FAIL |
| `l1d_ecc_enc` | none (combinational) | 250 | - | - | 62.4 | 5.33 | 22 | ok |
| `l1d_ecc_fix` | none (combinational) | 250 | - | - | 136.7 | 8.82 | 21 | ok |
| `l1d_ecc_syn` | none (combinational) | 250 | - | - | -163.7 | 2.42 | 25 | ok |
| `l1d_fifo` | clk | 250 | 32.1 | 4.59 | 32.1 | 4.59 | 31 | ok |
| `l1d_ks_add` | none (combinational) | 250 | - | - | 71.4 | 5.60 | 9 | ok |
| `l1d_ks_add_q` | clk | 250 | - | - | 32.1 | 4.59 | 16 | ok |
| `l1d_l2_port` | clk | 250 | -295.7 | 1.83 | -295.7 | 1.83 | 1583 | ok |
| `l1d_maint` | clk | 250 | 63.7 | 5.37 | 63.7 | 5.37 | 18 | ok |
| `l1d_mesi_next` | none (combinational) | 250 | - | - | 113.7 | 7.34 | 5 | ok |
| `l1d_meta_flops` | clk | 250 | 148.4 | 9.84 | 59.6 | 5.25 | 759 | ok |
| `l1d_mshr` | clk | 250 | -179.2 | 2.33 | -179.2 | 2.33 | 11815 | ok |
| `l1d_mshr_entry` | clk | 250 | 29.2 | 4.53 | 29.2 | 4.53 | 1408 | ok |
| `l1d_mshr_nc` | clk | 250 | 29.6 | 4.54 | 29.6 | 4.54 | 480 | ok |
| `l1d_pf_sms` | clk | 250 | -325.9 | 1.74 | -325.9 | 1.74 | 7589 | ok |
| `l1d_pf_sms_core` | clk | 250 | -341.5 | 1.69 | -341.5 | 1.69 | 6499 | ok |
| `l1d_pf_sms_expand` | clk | 250 | -179.8 | 2.33 | -179.8 | 2.33 | 391 | ok |
| `l1d_pf_sms_fifo` | clk | 250 | -122.7 | 2.68 | -122.7 | 2.68 | 241 | ok |
| `l1d_pf_sms_front` | clk | 250 | -391.1 | 1.56 | -391.1 | 1.56 | 807 | ok |
| `l1d_pf_stream` | clk | 250 | -311.6 | 1.78 | -311.6 | 1.78 | 1162 | ok |
| `l1d_pf_stride` | clk | 250 | -43.4 | 3.41 | -43.4 | 3.41 | 2972 | ok |
| `l1d_plru` | none (combinational) | 250 | - | - | 206.8 | 23.16 | 1 | ok |
| `l1d_port_front` | clk | 250 | -410.8 | 1.51 | -410.8 | 1.51 | 1700 | ok |
| `l1d_prefetch_unit` | clk | 250 | -394.3 | 1.55 | -394.3 | 1.55 | 12201 | ok |
| `l1d_pri_enc` | none (combinational) | 250 | - | - | 210.0 | 24.99 | 1 | ok |
| `l1d_req_tracker` | clk | 250 | -523.4 | 1.29 | -559.6 | 1.24 | 2119 | ok |
| `l1d_reservation_unit` | clk | 250 | 19.5 | 4.34 | 19.5 | 4.34 | 142 | ok |
| `l1d_resp_gen` | none (combinational) | 250 | - | - | 53.6 | 5.09 | 405 | ok |
| `l1d_resp_merge` | clk | 250 | -0.8 | 3.99 | -65.2 | 3.17 | 903 | ok |
| `l1d_s1_lookup` | none (combinational) | 250 | - | - | 85.1 | 6.06 | 41 | ok |
| `l1d_snoop_buf` | clk | 250 | 16.3 | 4.28 | 16.3 | 4.28 | 167 | ok |
| `l1d_snoop_ctrl` | clk | 250 | 32.2 | 4.59 | 32.2 | 4.59 | 3117 | ok |
| `l1d_sram_tdp` | clk | 250 | - | - | 57.5 | 5.19 | 2404 | ok |
| `l1d_tag_array` | clk | 250 | - | - | - | - | - | SYNTH_FAIL |
| `l1d_top` | clk | 250 | - | - | - | - | - | SYNTH_FAIL |
| `l1d_vec_store_buf` | clk | 250 | 76.9 | 5.78 | 76.9 | 5.78 | 695 | ok |
| `l1d_victim_cache` | clk | 250 | -12.7 | 3.81 | -12.7 | 3.81 | 3516 | ok |

50 modules.
