#!/bin/bash

if [ -n "${1}" ]; then
	file="${1}"
else
	echo "sh ${0} [file.raw]"
	exit 1
fi

rostopic echo /robot/limb/left/joint_states -n 1 | grep position >> $file

