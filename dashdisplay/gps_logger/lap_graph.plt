#!/usr/bin/gnuplot

set terminal png background rgb "black" size 400,400
set output pngout
set size ratio -1
unset xtics
unset ytics
unset border
unset colorbox
set palette defined ( 0 "red", 1 "grey80", 2 "green" )

plot tracein u 2:3:4 notitle w l lw 8 palette
