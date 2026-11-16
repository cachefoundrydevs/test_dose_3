#!/usr/bin/env bash
# equiv.sh MODULE GOLDEN.v CURRENT.v OUTDIR
# Formal equivalence (Yosys) of the golden and the current version of MODULE, each flattened with its children from rtl/.
# Each side is elaborated in its own Yosys run (the include guards and the module names are the same), renamed to gold /
# gate and written as RTLIL; a third run proves them equal: equiv_make, equiv_simple -seq 5, equiv_induct, equiv_status
# -assert. If that does not prove every point, a miter with sat -tempinduct (reset held in cycle 0) is the fallback.
set -uo pipefail
M=$1; GOLD=$2; CUR=$3; OUT=$4
mkdir -p "$OUT"
# children: the files of rtl.f whose module is instantiated (transitively) by MODULE
ALL=$(grep -v '^[-#]' filelists/rtl.f | sed '/^\s*$/d')
KIDS=""; QUEUE="$CUR"
while [[ -n "$QUEUE" ]]; do
    set -- $QUEUE; f=$1; shift; QUEUE="$*"
    for g in $ALL; do
        n=$(basename "$g" .v)
        [[ "$n" == "$M" || " $KIDS " == *" $g "* ]] && continue
        if grep -qE "^\s*$n\b" "$f"; then KIDS="$KIDS $g"; QUEUE="$QUEUE $g"; fi
    done
done
echo "children of $M:$KIDS"
elab() {   # $1 = source file of MODULE, $2 = new name, $3 = output .il
    cat > "$OUT/elab_$2.ys" <<YS
read_verilog -DSYNTHESIS -mem2reg -Irtl/include $KIDS $1
hierarchy -top $M
proc
flatten
opt_clean
rename $M $2
write_rtlil $3
YS
    yosys -q -l "$OUT/elab_$2.log" -s "$OUT/elab_$2.ys"
}
elab "$GOLD" gold "$OUT/gold.il" || { echo "EQUIV: FAIL (golden does not elaborate)"; exit 1; }
elab "$CUR"  gate "$OUT/gate.il" || { echo "EQUIV: FAIL (current does not elaborate)"; exit 1; }
cat > "$OUT/equiv.ys" <<YS
read_rtlil $OUT/gold.il
read_rtlil $OUT/gate.il
async2sync
equiv_make gold gate equiv
hierarchy -top equiv
equiv_simple -seq 5
equiv_induct -seq 5
equiv_status -assert
YS
if yosys -l "$OUT/equiv.log" -s "$OUT/equiv.ys" > /dev/null 2>&1; then
    grep -E 'Found [0-9]+ \$equiv cells|Of those cells|proven|unproven' "$OUT/equiv.log" | tail -5
    echo "EQUIV: PASS (equiv_make / equiv_simple / equiv_induct)"; exit 0
fi
grep -E 'Found [0-9]+ \$equiv|Of those|unproven' "$OUT/equiv.log" | tail -8
echo "equiv flow did not prove all points; fallback: miter + sat -tempinduct"
cat > "$OUT/miter.ys" <<YS
read_rtlil $OUT/gold.il
read_rtlil $OUT/gate.il
async2sync
miter -equiv -flatten -make_assert -ignore_gold_x gold gate miter
hierarchy -top miter
flatten
opt -fast
sat -verify -tempinduct -prove-asserts -set-init-zero -set-at 1 in_rst 1 -seq 1 -maxsteps 20 -show-ports miter
YS
if yosys -l "$OUT/miter.log" -s "$OUT/miter.ys" > /dev/null 2>&1; then
    grep -E 'Induction step proven|SUCCESS|reached' "$OUT/miter.log" | tail -3
    echo "EQUIV: PASS (miter, sat -tempinduct)"; exit 0
fi
grep -E 'failed|FAIL|counter|Induction' "$OUT/miter.log" | tail -10
echo "EQUIV: FAIL"; exit 1
