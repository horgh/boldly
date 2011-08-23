#!/usr/bin/gnuplot -persist
# set terminal postscript landscape noenhanced monochrome \
#              dashed defaultplex "Helvetica" 14
# set output 'output.ps'

RUN=0
MAP="hospital"
SUFFIX=".processed"

set ylabel "Distance Travelled (m)" 0.000000,0.000000
set xlabel "% of Maximum Distance Discovered" 0.000000,0.000000
set title "Robot Performance" 0.000000,0.000000
set xrange [ 0 : 95 ]
set key left top

set terminal png enhanced size 1500, 900
# Set the output-file name.
set output "graph.png"

#plot "data/".MAP."/".RUN."-0-".MAP.SUFFIX using 2:1 title "Run ".RUN.", A" with lines, \
#    "data/".MAP."/".RUN."-1-".MAP.SUFFIX using 2:1 title "Run ".RUN.", B" with lines, \
#    "data/".MAP."/".RUN."-2-".MAP.SUFFIX using 2:1 title "Run ".RUN.", C" with lines

plot "data/".MAP."/0-0-".MAP.SUFFIX using 2:1 title "Run 0, A" with lines linecolor rgb "red", \
    "data/".MAP."/0-1-".MAP.SUFFIX using 2:1 title "Run 0, B" with lines linecolor rgb "green", \
    "data/".MAP."/0-2-".MAP.SUFFIX using 2:1 title "Run 0, C" with lines linecolor rgb "blue", \
    "data/".MAP."/1-0-".MAP.SUFFIX using 2:1 title "Run 1, A" with lines linecolor rgb "red", \
    "data/".MAP."/1-1-".MAP.SUFFIX using 2:1 title "Run 1, B" with lines linecolor rgb "green", \
    "data/".MAP."/1-2-".MAP.SUFFIX using 2:1 title "Run 1, C" with lines linecolor rgb "blue", \
    "data/".MAP."/2-0-".MAP.SUFFIX using 2:1 title "Run 2, A" with lines linecolor rgb "red", \
    "data/".MAP."/2-1-".MAP.SUFFIX using 2:1 title "Run 2, B" with lines linecolor rgb "green", \
    "data/".MAP."/2-2-".MAP.SUFFIX using 2:1 title "Run 2, C" with lines linecolor rgb "blue", \
    "data/".MAP."/3-0-".MAP.SUFFIX using 2:1 title "Run 3, A" with lines linecolor rgb "red", \
    "data/".MAP."/3-1-".MAP.SUFFIX using 2:1 title "Run 3, B" with lines linecolor rgb "green", \
    "data/".MAP."/3-2-".MAP.SUFFIX using 2:1 title "Run 3, C" with lines linecolor rgb "blue", \
    "data/".MAP."/4-0-".MAP.SUFFIX using 2:1 title "Run 4, A" with lines linecolor rgb "red", \
    "data/".MAP."/4-1-".MAP.SUFFIX using 2:1 title "Run 4, B" with lines linecolor rgb "green", \
    "data/".MAP."/4-2-".MAP.SUFFIX using 2:1 title "Run 4, C" with lines linecolor rgb "blue", \
    "data/".MAP."/5-0-".MAP.SUFFIX using 2:1 title "Run 5, A" with lines linecolor rgb "red", \
    "data/".MAP."/5-1-".MAP.SUFFIX using 2:1 title "Run 5, B" with lines linecolor rgb "green", \
    "data/".MAP."/5-2-".MAP.SUFFIX using 2:1 title "Run 5, C" with lines linecolor rgb "blue", \
    "data/".MAP."/6-0-".MAP.SUFFIX using 2:1 title "Run 6, A" with lines linecolor rgb "red", \
    "data/".MAP."/6-1-".MAP.SUFFIX using 2:1 title "Run 6, B" with lines linecolor rgb "green", \
    "data/".MAP."/6-2-".MAP.SUFFIX using 2:1 title "Run 6, C" with lines linecolor rgb "blue", \
    "data/".MAP."/7-0-".MAP.SUFFIX using 2:1 title "Run 7, A" with lines linecolor rgb "red", \
    "data/".MAP."/7-1-".MAP.SUFFIX using 2:1 title "Run 7, B" with lines linecolor rgb "green", \
    "data/".MAP."/7-2-".MAP.SUFFIX using 2:1 title "Run 7, C" with lines linecolor rgb "blue", \
    "data/".MAP."/8-0-".MAP.SUFFIX using 2:1 title "Run 8, A" with lines linecolor rgb "red", \
    "data/".MAP."/8-1-".MAP.SUFFIX using 2:1 title "Run 8, B" with lines linecolor rgb "green", \
    "data/".MAP."/8-2-".MAP.SUFFIX using 2:1 title "Run 8, C" with lines linecolor rgb "blue", \
    "data/".MAP."/9-0-".MAP.SUFFIX using 2:1 title "Run 9, A" with lines linecolor rgb "red", \
    "data/".MAP."/9-1-".MAP.SUFFIX using 2:1 title "Run 9, B" with lines linecolor rgb "green", \
    "data/".MAP."/9-2-".MAP.SUFFIX using 2:1 title "Run 9, C" with lines linecolor rgb "blue"
