set datafile separator ','

fn = '../22-05-15_22-22.csv'

set xlabel 'x'
set ylabel 'y'
set zlabel 'z'

splot fn using 1:2:3 with linespoints title 'ball',\
fn using 4:5:6 with linespoints title 'kalman'
