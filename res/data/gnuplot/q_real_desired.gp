set datafile separator ','
set key autotitle columnhead

fn = '../22-05-15_22-22.csv'

set xlabel 'time (s)'
set ylabel 'angle (rad)'

plot fn using 22 with linespoints,\
fn using 29 with linespoints,\
fn using 23 with linespoints,\
fn using 30 with linespoints,\
fn using 24 with linespoints,\
fn using 31 with linespoints
