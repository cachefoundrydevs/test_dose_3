#!/usr/bin/perl
# age_assume_a1.pl l1d_age_matrix.v > patched.v   (comment-stripped file of the public copy)
# Assumption A1 for the equivalence proof ONLY: the pick oldest_onehot of l1d_age_matrix is one-hot or zero (design premise of
# ONEHOT_ENC = 1; the simulation check in the module prints an ERROR otherwise). The patched copy forms the output from the raw
# pick as its lowest set bit, which is the identity under A1. Both sides of the proof use the same patched copy.
use strict; use warnings;
local $/; open(my $in, '<', $ARGV[0]) or die "$ARGV[0]: $!"; my $t = <$in>; close $in; $t =~ s/\r//g;
$t =~ s/output reg(\s+)\[N-1:0\](\s+)oldest_onehot,/output wire$1\[N-1:0\]$2oldest_onehot,/ or die "output decl not found";
my $n = ($t =~ s/\boldest_onehot\[a\](\s*)=/oldest_raw[a]$1=/g); die "pick assignments not found" unless $n == 2;
$t =~ s/(output wire\s+any_candidate\s*\);)/$1\n    reg [N-1:0] oldest_raw;\n    assign oldest_onehot = oldest_raw & ~(oldest_raw - 1'b1);/ or die "port list end not found";
print $t;
