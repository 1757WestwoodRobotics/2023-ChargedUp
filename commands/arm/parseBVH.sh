#!/bin/sh 
# takes in two arguments, 1 which is the BVH file and a second for the output, converts only the rotation on each linkage that we want
tail -n +38 "$1" | awk '{ print $4,$7,$10; }' > "$2"
