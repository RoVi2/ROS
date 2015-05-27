load 'common.gp.in'

fn = '../22-05-15_22-26.csv'

set xlabel 'x'
set ylabel 'y'
set zlabel 'z'
set cblabel 'time (s)'

set palette model HSV defined ( 0 0 1 1, 1 1 1 1 )

splot fn every ::320::400 using 2:3:4:1 with linespoints pointtype 19 palette title 'detected ball position',\
fn every ::320::400 using 5:6:7:1 with linespoints pointtype 2 palette title 'ball pos. kalman estimate'
