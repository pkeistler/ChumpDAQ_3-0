#!/bin/bash
#!/usr/bin/gnuplot

folder=$1
imageout="/media/chump_thumb/chump_logs/$1/lap_plot.png"
datain="/media/chump_thumb/chump_logs/$1/deltav_trace.dat"

tail -n +2 $datain | tail -n 420 > $datain.mod
tail -n 1 $datain > $datain.mod2
#bounds=$(tail -n +2 $datain | tail -n 420 | awk 'BEGIN{xmax=-99999;xmin=99999;ymax=-99999;ymin=99999}\
#	{if ($2 > xmax) xmax=$2; if ($2 < xmin) xmin = $2; if ($3 > ymax) ymax = $3; if ($3 < ymin) ymin = $3}\
#	END{print xmin,xmax,ymin,ymax}')
#
#xmin=$(echo $bounds | awk '{print $1-100}')
#xmax=$(echo $bounds | awk '{print $2+100}')
#ymin=$(echo $bounds | awk '{print $3-100}')
#ymax=$(echo $bounds | awk '{print $4+100}')

/usr/bin/gnuplot << EOF
set terminal pngcairo round background rgb "black" size 600,600
set output "$imageout"
set size ratio -1
unset xtics
unset ytics
unset border
unset colorbox
set palette defined ( 0 "red", 1 "grey100", 2 "grey100", 3 "grey100", 5 "green" )

set cbrange [-5:5]

plot "$datain.mod" u 2:3:4 notitle w l lw 18 palette,\
     "$datain.mod2" u 2:3:(100) notitle w circles linecolor rgb "#000000" lw 12 fill solid border lc rgb "#FFCC00"
EOF
#     "< tail -n 1 $datain" u 2:3 notitle w p pt 6 lc rgb "#2222FF" lw 15
#set xrange [$xmin:$xmax]
#set yrange [$ymin:$ymax]
