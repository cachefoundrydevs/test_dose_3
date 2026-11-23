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
# Assumption A1 (user decision 2026-11-18): only when golden/$M.assume_age_onehot exists, both sides use a copy of l1d_age_matrix whose
# pick is one-hot by construction (flow/age_assume_a1.pl). The proof then reads "equal for all states, given A1"; A1 is checked by simulation.
A1=0
if [[ -f golden/$M.assume_age_onehot && " $KIDS " == *" rtl/common/l1d_age_matrix.v "* ]]; then
    perl flow/age_assume_a1.pl rtl/common/l1d_age_matrix.v > "$OUT/l1d_age_matrix_a1.v" || { echo "EQUIV: FAIL (A1 patch)"; exit 1; }
    KIDS=$(echo " $KIDS " | sed "s#  *rtl/common/l1d_age_matrix.v # $OUT/l1d_age_matrix_a1.v #")
    A1=1
    echo "ASSUMPTION A1: age matrix pick is one-hot or zero (proof is equal for all states given A1)"
fi
# the gold side uses golden/<child>.v for every child that has a golden copy, so an edit of a child is really compared (after the A1 patch of KIDS)
GKIDS=""
for g in $KIDS; do gb=golden/$(basename "$g"); if [[ -f "$gb" ]]; then GKIDS="$GKIDS $gb"; else GKIDS="$GKIDS $g"; fi; done
elab() {   # $1 = source file of MODULE, $2 = new name, $3 = output .il, $4 = child files
    cat > "$OUT/elab_$2.ys" <<YS
read_verilog -DSYNTHESIS -mem2reg -Irtl/include $4 $1
hierarchy -top $M
proc
flatten
opt_clean
rename $M $2
write_rtlil $3
YS
    yosys -q -l "$OUT/elab_$2.log" -s "$OUT/elab_$2.ys"
}
elab "$GOLD" gold "$OUT/gold.il" "$GKIDS" || { echo "EQUIV: FAIL (golden does not elaborate)"; exit 1; }
elab "$CUR"  gate "$OUT/gate.il" "$KIDS" || { echo "EQUIV: FAIL (current does not elaborate)"; exit 1; }
# optional golden/<M>.equiv_blacklist: base names of INTERNAL combinational signals whose encoding was changed on
# purpose (outputs and flops are never listed); they are not paired, everything after them is still proven
BL=""
if [[ -f golden/$M.equiv_blacklist ]]; then
    : > "$OUT/blacklist.txt"
    while read -r b; do
        b=${b%%#*}; b=$(echo "$b" | tr -d ' \r'); [[ -z "$b" ]] && continue
        # last word of every 'wire' line (an escaped name: first character is the escape mark), kept when its last
        # dotted part is the base name, with or without an index
        awk '$1 == "wire" {print $NF}' "$OUT/gold.il" | cut -c2- | grep -E "^([][A-Za-z0-9_.]*[.])?$b([[][0-9]+[]])?$" >> "$OUT/blacklist.txt" || true
    done < "golden/$M.equiv_blacklist"
    sort -u -o "$OUT/blacklist.txt" "$OUT/blacklist.txt"
    echo "blacklisted internal signals: $(tr '\n' ' ' < "$OUT/blacklist.txt")"
    BL="-blacklist $OUT/blacklist.txt"
fi
cat > "$OUT/equiv.ys" <<YS
read_rtlil $OUT/gold.il
read_rtlil $OUT/gate.il
async2sync
equiv_make $BL gold gate equiv
hierarchy -top equiv
equiv_simple -seq 5
equiv_induct -seq 5
equiv_status -assert
YS
if yosys -l "$OUT/equiv.log" -s "$OUT/equiv.ys" > /dev/null 2>&1; then
    grep -E 'Found [0-9]+ \$equiv cells|Of those cells|proven|unproven' "$OUT/equiv.log" | tail -5
    echo "EQUIV: PASS (equiv_make / equiv_simple / equiv_induct)"; [[ $A1 == 1 ]] && echo "EQUIV: PASS holds under assumption A1 (age pick one-hot)"; exit 0
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
# the fallback runs on the whole flattened miter (millions of variables): cap it, a timeout is a FAIL, not an endless job
if timeout "${MITER_TIMEOUT_S:-1500}" yosys -l "$OUT/miter.log" -s "$OUT/miter.ys" > /dev/null 2>&1; then
    grep -E 'Induction step proven|SUCCESS|reached' "$OUT/miter.log" | tail -3
    echo "EQUIV: PASS (miter, sat -tempinduct)"; exit 0
fi
grep -E 'failed|FAIL|counter|Induction' "$OUT/miter.log" | tail -10
[[ -f "$OUT/miter.log" ]] && ! grep -q "SUCCESS" "$OUT/miter.log" && echo "(miter stopped by the time cap or did not finish: not proven)"
echo "EQUIV: FAIL"; exit 1
