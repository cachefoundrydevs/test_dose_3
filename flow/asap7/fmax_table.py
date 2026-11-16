#!/usr/bin/env python3
"""fmax_table.py ARTIFACT_DIR OUT.md : one table of the maximum frequency of every RTL module, from the modules.csv files
written by run_timing.sh. Fmax = 1000 / (target period - worst setup slack). Two figures per module: all paths, and
flop to flop only (the stage path). Post-synthesis, ideal clock, no wires, zero I/O delay."""
import csv, sys
from pathlib import Path

def num(x):
    try: return float(x)
    except (TypeError, ValueError): return None

def fmax(period, slack):
    if period is None or slack is None: return None
    d = period - slack
    return 1000.0 / d if d > 0 else None

rows = []
for p in sorted(Path(sys.argv[1]).rglob('modules.csv')):
    with open(p, newline='') as f:
        rows += list(csv.DictReader(f))
rows.sort(key=lambda r: r['module'])
def g(v, n=3): return '-' if v is None else f'{v:.{n}f}'
out = ['# Maximum frequency of every RTL module (one synthesis run per module)', '',
       'ASAP7, post-synthesis, ideal clock (5 % uncertainty), no wires, zero I/O delay, assumed SRAM macros. A parent includes its flattened children.',
       'Fmax = 1000 / (target period - worst setup slack). "flop to flop" uses register-to-register paths only; "all paths" includes input and output pin paths.', '',
       '| Module | Clock | Target ps | Flop to flop: slack ps | Flop to flop: Fmax GHz | All paths: slack ps | All paths: Fmax GHz | Area um2 | Status |',
       '|---|---|---:|---:|---:|---:|---:|---:|---|']
for r in rows:
    per = num(r['period_ps']); sa = num(r['slack_all']); sr = num(r['slack_r2r'])
    out.append(f"| `{r['module']}` | {'clk' if r['has_clk'] == '1' else 'none (combinational)'} | {r['period_ps']} | {g(sr, 1)} | {g(fmax(per, sr), 2)} | {g(sa, 1)} | {g(fmax(per, sa), 2)} | {r['area_um2'] or '-'} | {r['status']} |")
out += ['', f'{len(rows)} modules.']
Path(sys.argv[2]).write_text('\n'.join(out) + '\n')
print('\n'.join(out))
