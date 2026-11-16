# ASAP7 synthesis + place and route

Ideal-wire assumptions removed: placement, clock tree and routing are done. SRAM macros are ESTIMATES
(no ASAP7 memory compiler). I/O delay 0, clock uncertainty 5 %. Fmax = 1000 / (period - worst setup slack).

| top | corner | period ps | synth | pnr | last stage | setup WS ps | TNS ps | hold WS ps | area um2 | Fmax GHz |
|---|---|---|---|---|---|---|---|---|---|---|
| l1d_bank | BC | 250 | 0 | 2 | 5_1_grt.json | -726.943 | -6023580.0 | -349.067 | 48503.2 | 1.024 |
| l1d_bank | BC | 350 | 0 | 2 | 5_1_grt.json | -658.122 | -4070800.0 | -329.551 | 48408.6 | 0.992 |
| l1d_bank | TC | 250 | 0 | 2 | 5_1_grt.json | -1016.47 | -10083000.0 | -460.438 | 48640.1 | 0.79 |
| l1d_bank | TC | 350 | 0 | 2 | 5_1_grt.json | -941.595 | -7978810.0 | -454.052 | 47948.1 | 0.774 |

Exit code 0 = stage finished; na = not started. Per-stage slacks:

- l1d_bank BC 250 ps: 2_1_floorplan [-590.2], 3_3_place_gp [-618.0], 3_4_place_resized [-618.0], 3_5_place_dp [-608.9], 4_1_cts [-769.8], 5_1_grt [-726.9]
- l1d_bank BC 350 ps: 2_1_floorplan [-495.2], 3_3_place_gp [-504.1], 3_4_place_resized [-504.1], 3_5_place_dp [-494.6], 4_1_cts [-670.7], 5_1_grt [-658.1]
- l1d_bank TC 250 ps: 2_1_floorplan [-825.1], 3_3_place_gp [-815.3], 3_4_place_resized [-815.3], 3_5_place_dp [-810.2], 4_1_cts [-1062.5], 5_1_grt [-1016.5]
- l1d_bank TC 350 ps: 2_1_floorplan [-730.1], 3_3_place_gp [-710.6], 3_4_place_resized [-710.6], 3_5_place_dp [-709.4], 4_1_cts [-974.2], 5_1_grt [-941.6]
