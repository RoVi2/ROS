#!/bin/bash

font=",9"

opts="set terminal tikz; \
set xtics font '${font}'; \
set ytics font '${font}'; \
set xlabel font '${font}'; \
set ylabel font '${font}'; \
set key font '${font}'; \
set label font '${font}'; \
set title font '${font}'"

for f in ./*.gp
do
    y=${f%.gp}
    output=${y##*/}_tikz.tex
    echo "Running ${f} -> ${output}"
    gnuplot -e "set output '${output}'; ${opts}" "$f"
done
