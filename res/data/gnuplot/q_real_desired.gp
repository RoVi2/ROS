load 'common.gp.in'
set key autotitle columnhead

fn = '../22-05-15_22-22.csv'

set xlabel 'time (s)'
set ylabel 'angle (rad)'

plot fn using 22 with lines,\
fn using 29 with lines,\
fn using 23 with lines,\
fn using 30 with lines,\
fn using 24 with lines,\
fn using 31 with lines
