#!/bin/bash
while :
do
    for VARIABLE in 7 8 9
    do
        echo $VARIABLE
        rostopic pub --once /uav/calibration std_msgs/UInt8 "data: $VARIABLE"
    	sleep 1
    done
	echo "Press [CTRL+C] to stop.."
done