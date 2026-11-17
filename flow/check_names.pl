#!/usr/bin/perl
# check_names.pl GOLDEN.v CURRENT.v
# Lists, for both files, the module header, every declaration (parameter, localparam, port, wire, reg, integer, genvar:
# name and width), every block label, every always block header and every for loop header (loop variable and bounds),
# in file order, and diffs the two lists. Any difference fails (exit 1). Expressions may change; names may not.
# Changes the user approved (new signals, a removed loop) are listed in golden/<M>.names_changes (see below).
use strict; use warnings;

sub items {
    my ($f) = @_;
    open(my $in, '<', $f) or die "$f: $!"; local $/; my $t = <$in>; close $in;
    $t =~ s/\r//g;
    $t =~ s{("(?:\\.|[^"\\])*")|//[^\n]*|/\*.*?\*/}{defined $1 ? $1 : ""}gse;
    my @out;
    # module header up to the end of the port list
    if ($t =~ /(\bmodule\b.*?\)\s*;)/s) { (my $h = $1) =~ s/\s+/ /g; push @out, "MODULE $h"; }
    # statements split at ';' and at begin / end keywords, kept in order
    my @tok = split /(;|\bbegin\b\s*(?::\s*\w+)?|\bend\b|\balways\b\s*@\s*(?:\*|\([^)]*\))|\bfor\s*\((?:[^()]|\([^()]*\))*\))/, $t;
    my $stmt = '';
    for my $p (@tok) {
        next unless defined $p;
        if ($p =~ /^\s*begin\s*:\s*(\w+)/) { push @out, "LABEL $1"; next; }
        if ($p =~ /^\s*always/) { (my $a = $p) =~ s/\s+/ /g; push @out, "ALWAYS $a"; next; }
        if ($p =~ /^\s*for\s*\(/) { (my $a = $p) =~ s/\s+//g; push @out, "FOR $a"; next; }
        if ($p eq ';') {
            (my $s = $stmt) =~ s/\s+/ /g; $s =~ s/^ //; $s =~ s/ $//;
            if ($s =~ /^(?:.*\b)?(input|output|inout|wire|reg|integer|genvar|localparam|parameter)\b(.*)$/s
                && $s !~ /^\s*(assign|if|else)\b/ && $s !~ /<=/) {
                my $d = "$1$2";
                $d =~ s/\s*=.*$// if $d =~ /^(input|output|inout|wire|reg|integer|genvar)\b/;
                $d =~ s/\s*([\[\]:,])\s*/$1/g;
                push @out, "DECL $d";
            }
            $stmt = ''; next;
        }
        $stmt = ($p =~ /^\s*(begin|end)\b/) ? '' : $stmt . $p;
    }
    return @out;
}

if (@ARGV == 2 && $ARGV[0] eq '--list') { print "$_\n" for items($ARGV[1]); exit 0; }
my ($g, $c) = @ARGV; die "usage: check_names.pl GOLDEN.v CURRENT.v | --list FILE\n" unless defined $c;
my @a = items($g); my @b = items($c);
my $bad = 0;
# optional <golden>.names_changes: the approved changes only. "+ ITEM" = an item the current file may add,
# "- ITEM" = a golden item the current file may drop (ITEM exactly as --list prints it). Each line must match
# one item, else the check fails, so the list stays exact. Every other item must still match in order.
# The two lists are walked together; at a mismatch an approved added (current) or dropped (golden) item is skipped.
(my $chg = $g) =~ s/\.v$/.names_changes/;
if (-f $chg) {
    my (%add, %drop);
    open(my $ch, '<', $chg) or die "$chg: $!";
    while (my $l = <$ch>) {
        $l =~ s/\r?\n$//; next if $l =~ /^\s*(#|$)/;
        my ($op, $it) = $l =~ /^([+-]) (.*)$/ or do { print "BAD LINE in $chg: $l\n"; $bad++; next; };
        if ($op eq '+') { $add{$it}++; } else { $drop{$it}++; }
    }
    close $ch;
    my (@ga, @cb); my ($i, $j) = (0, 0);
    while ($i < @a || $j < @b) {
        if ($i < @a && $j < @b && $a[$i] eq $b[$j]) { push @ga, $a[$i++]; push @cb, $b[$j++]; next; }
        if ($j < @b && $add{$b[$j]})  { print "approved + $b[$j]\n"; $add{$b[$j]}--;  $j++; next; }
        if ($i < @a && $drop{$a[$i]}) { print "approved - $a[$i]\n"; $drop{$a[$i]}--; $i++; next; }
        push @ga, $a[$i] if $i < @a; push @cb, $b[$j] if $j < @b; $i++; $j++;
    }
    for my $k (grep { $add{$_} }  keys %add)  { print "NOT FOUND (+): $k\n"; $bad++; }
    for my $k (grep { $drop{$_} } keys %drop) { print "NOT FOUND (-): $k\n"; $bad++; }
    @a = @ga; @b = @cb;
}
my $n = @a > @b ? scalar @a : scalar @b;
for my $i (0 .. $n - 1) {
    my ($x, $y) = ($a[$i] // '(none)', $b[$i] // '(none)');
    next if $x eq $y;
    print "DIFF item $i\n  golden : $x\n  current: $y\n"; $bad++;
    last if $bad >= 40;
}
printf "names: %d items golden, %d items current, %d differences\n", scalar @a, scalar @b, $bad;
print $bad ? "NAMES: FAIL\n" : "NAMES: PASS\n";
exit($bad ? 1 : 0);
