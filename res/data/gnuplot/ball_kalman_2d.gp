set datafile separator ','
set key autotitle columnhead

fn = '../22-05-15_22-22.csv'

set xlabel 'time (s)'

plot fn using 1 with linespoints,\
fn using 2 with linespoints,\
fn using 3 with linespoints,\
fn using 4 with linespoints,\
fn using 5 with linespoints,\
fn using 6 with linespoints
