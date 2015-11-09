#!/bin/bash

if [ -n "${1}" ]; then
	reg_exp="${1}"
else
	echo "Regular expression required to convert raw files whose names match the missing regular expression"
	exit 1
fi

for rawfile in `ls -tr ${reg_exp}` ; 
do 
	echo "Converting ${rawfile} into xml trajectory file ( trajectories.xml )"; 
	rosrun tictactoe trajectory_from_raw_to_xml $rawfile trajectories.xml left_e0 left_e1 left_s0 left_s1 left_w0 left_w1 left_w2
	
done

