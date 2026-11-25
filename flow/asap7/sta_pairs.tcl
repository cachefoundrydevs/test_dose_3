# Crossbank check (added 2026-11-25): worst setup slack between GROUPS of registers (each bank, port front, each response merger,
# snoop control, prefetch unit, L2 port, rest), so a path that starts in one bank and ends in another can be seen. The stock reports list
# only the 10 to 12 worst paths of the whole design, so "no bank pair seen" there proves nothing.
# Outputs: <prefix>_pairs.csv (from,to,slack_ps for every group pair that has a path), <prefix>_pairs_paths.rpt (full paths:
# every bank a -> bank b pair that has a path, and the three worst paths inside each group, and the worst 12 group pairs).
foreach f $::env(LEF_FILES) { read_lef $f }
foreach f $::env(LIB_FILES) { read_liberty $f }
read_verilog $::env(NETLIST)
link_design $::env(TOP)
read_sdc $::env(SDC_FILE)
set prefix $::env(REPORT_PREFIX)

proc group_of {full} {
    set n [string map [list "\\" ""] $full]
    if {[regexp {^g_bank\[([0-9]+)\]} $n -> i]} { return bank$i }
    if {[regexp {^u_merge_([0-9]+)} $n -> i]}   { return merge$i }
    if {[string match u_front* $n]}             { return front }
    if {[string match u_snoop* $n]}             { return snoop }
    if {[string match g_prefetch* $n]}          { return prefetch }
    if {[string match *l2* $n]}                 { return l2port }
    return other
}
foreach c [all_registers -cells] {
    set g [group_of [get_full_name $c]]
    lappend G($g) $c
}
set names [lsort [array names G]]
foreach g $names { puts "GROUP $g registers=[llength $G($g)]" }

proc slack_of {from_cells to_cells} {
    sta::redirect_string_begin
    set code [catch {report_checks -path_delay max -from $from_cells -to $to_cells -format summary -digits 3} err]
    set txt [sta::redirect_string_end]
    if {$code} { return na }
    set s na
    foreach line [split $txt "\n"] { if {[regexp {(-?[0-9]+\.[0-9]+)\s*$} $line -> v]} { set s $v } }
    return $s
}
set csv [open ${prefix}_pairs.csv w]
puts $csv "from,to,slack_ps"
set rows {}
foreach a $names {
    foreach b $names {
        set s [slack_of $G($a) $G($b)]
        puts $csv "$a,$b,$s"
        puts "PAIR $a -> $b : $s"
        if {$s ne "na"} { lappend rows [list $a $b $s] }
    }
}
close $csv

set rpt [open ${prefix}_pairs_paths.rpt w]
proc dump {rpt title from_cells to_cells n} {
    puts $rpt "\n##### $title #####"
    sta::redirect_string_begin
    catch {report_checks -path_delay max -from $from_cells -to $to_cells -group_path_count $n -unique_paths_to_endpoint -fields {fanout} -digits 3}
    puts $rpt [sta::redirect_string_end]
}
# 1. every bank a -> bank b (a != b) that has a path: the crossbank paths
foreach r $rows {
    lassign $r a b s
    if {[regexp {^bank[0-9]$} $a] && [regexp {^bank[0-9]$} $b] && $a ne $b} { dump $rpt "CROSSBANK $a -> $b (slack $s)" $G($a) $G($b) 2 }
}
# 2. the three worst paths inside each group
foreach g $names { dump $rpt "INSIDE $g" $G($g) $G($g) 3 }
# 3. the 12 worst group pairs (different groups)
set cross {}
foreach r $rows { lassign $r a b s; if {$a ne $b} { lappend cross $r } }
set cross [lsort -real -index 2 $cross]
set k 0
foreach r $cross {
    if {$k >= 12} break
    lassign $r a b s
    dump $rpt "GROUP PAIR $a -> $b (slack $s)" $G($a) $G($b) 1
    incr k
}
close $rpt
exit
