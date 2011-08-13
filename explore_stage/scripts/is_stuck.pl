#!/usr/bin/env perl
#
# Take a portion of lines from poses_output and determine if robot is stuck
# or not
#

use strict;
use warnings;

my @poses_output_lines = `cat ~/.ros/poses_output`;

my $latest_time = -1;
my $latest_x = -1;
my $latest_y = -1;
foreach (reverse @poses_output_lines) {
  my ($time, $x, $y, $furthest) = split;
  if ($latest_time == -1) {
    $latest_time = $time;
    $latest_x = $x;
    $latest_y = $y;
  } else {
    if ($latest_x != $x || $latest_y != $y) {
      # Robot isn't stuck
      print "0";
      last;
    }

    # If pose hasn't changed for at least 30 min (ROS time), we're stuck
    if ($time + 1800 < $latest_time) {
    #if ($time + 60 < $latest_time) {
      # Robot is stuck
      print "1";
      last;
    }
  }
}
