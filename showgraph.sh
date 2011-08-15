#!/usr/bin/gnuplot -persist
# set terminal postscript landscape noenhanced monochrome \
#              dashed defaultplex "Helvetica" 14
# set output 'output.ps'

RUN=8
MAP="hospital"
SUFFIX=".processed"

set xlabel "Distance Travelled" 0.000000,0.000000
set ylabel "Maximum Distance Discovered" 0.000000,0.000000
set title "Robot Performance" 0.000000,0.000000

plot "data/".MAP."/".RUN."-0-".MAP.SUFFIX using 1:2 title "Run ".RUN.", A" with lines, \
    "data/".MAP."/".RUN."-1-".MAP.SUFFIX using 1:2 title "Run ".RUN.", B" with lines, \
    "data/".MAP."/".RUN."-2-".MAP.SUFFIX using 1:2 title "Run ".RUN.", C" with lines
