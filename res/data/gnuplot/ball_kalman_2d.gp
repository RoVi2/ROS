set datafile separator ','
set key autotitle columnhead

fn = '../22-05-15_22-22.csv'

set xlabel 'time (s)'

plot fn using 1:2 with linespoints,\
fn using 1:3 with linespoints,\
fn using 1:4 with linespoints,\
fn using 1:5 with linespoints,\
fn using 1:6 with linespoints,\
fn using 1:7 with linespoints
