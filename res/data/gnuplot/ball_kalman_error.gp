load 'common.gp.in'
set key autotitle columnhead

fn = '../22-05-15_22-26.csv'

set xlabel 'time (s)'
set ylabel 'distance'

unset colorbox

set palette model HSV defined ( 0 0 1 1, 1 1 1 1 )

plot fn every ::320::400 using 1:36:1 with linespoints palette pointtype 7
