#!/bin/bash

if [ -n "${1}" ]; then
	raw_file="${1}"
else
	echo "sh ${0} [raw_file]"
	exit 1
fi

filename=$(basename "$raw_file")
#extension="${filename##*.}"
filename="${filename%.*}"
#echo $filename

rosrun tictactoe trajectory_from_raw_to_xml $raw_file ${filename}.xml left_e0 left_e1 left_s0 left_s1 left_w0 left_w1 left_w2
