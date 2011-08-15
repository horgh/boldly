#!/usr/bin/gnuplot -persist
# set terminal postscript landscape noenhanced monochrome \
#              dashed defaultplex "Helvetica" 14
# set output 'output.ps'

RUN=0
MAP="hospital"

set ylabel "" 0.000000,0.000000
set xlabel "Distance Travelled to 95% Maximum Sight" 0.000000,0.000000
set title "Robot Performance Distributions" 0.000000,0.000000
set key left top

plot "data/".MAP."/0-".MAP.".histo" using 1:2 title "A", \
    "data/".MAP."/1-".MAP.".histo" using 1:2 title "B", \
    "data/".MAP."/2-".MAP.".histo" using 1:2 title "C"
