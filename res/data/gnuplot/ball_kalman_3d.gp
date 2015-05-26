set datafile separator ','

fn = '../22-05-15_22-22.csv'

set xlabel 'x'
set ylabel 'y'
set zlabel 'z'

splot fn using 2:3:4 with linespoints title 'ball',\
fn using 5:6:7 with linespoints title 'kalman'
