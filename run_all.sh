#!/bin/bash
for task in 5 6 8
do
    echo "---------------------------task=${task}-------------------------"
    for ((i=1; i<=20; i++))
    do
        echo "---------------------------i=${i}-------------------------"
        python hierarchical_LTL.py --task=nav --case=${task}
    done
done
