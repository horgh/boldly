#!/usr/bin/gnuplot -persist
# set terminal postscript landscape noenhanced monochrome \
#              dashed defaultplex "Helvetica" 14
# set output 'output.ps'

RUN=6
MAP="hospital"
SUFFIX=".processed"

set ylabel "Distance Travelled (m)" 0.000000,0.000000
set xlabel "% of Maximum Distance Discovered" 0.000000,0.000000
set title "Robot Performance" 0.000000,0.000000
set xrange [ 0 : 95 ]
set key left top

plot "data/".MAP."/".RUN."-0-".MAP.SUFFIX using 1:2 title "Run ".RUN.", A" with lines, \
    "data/".MAP."/".RUN."-1-".MAP.SUFFIX using 1:2 title "Run ".RUN.", B" with lines, \
    "data/".MAP."/".RUN."-2-".MAP.SUFFIX using 1:2 title "Run ".RUN.", C" with lines
