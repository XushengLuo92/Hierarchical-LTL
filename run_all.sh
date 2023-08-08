#!/bin/bash
for ((i=1; i<=20; i++))
do
    echo "---------------------------i=${i}-------------------------"
    python hierarchical_LTL.py --task=nav --case=5
done