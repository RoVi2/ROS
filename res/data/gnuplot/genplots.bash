#!/bin/bash

for f in ./*.gp
do
    y=${f%.gp}
    output=${y##*/}_tikz.tex
    echo "Running ${f} -> ${output}"
    gnuplot -e "set output '${output}'" "$f"
done
