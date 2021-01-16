#!/bin/bash
# This script creates virtual CAN port using slcan
# You can use it in 2 ways:
# ./scripts/create_slcan.sh - use automatic device path search
# ./scripts/create_slcan.sh /dev/ttyACMx - use user device path

# 1. Set tty settings
if [ $# == 1 ]
then
    DEV_PATH=$1
else
    EXPECTED_VID=0483
    EXPECTED_PID=374b
    EXPECTED_DEV_PATH="/dev/ttyACM*"
    for dev_path in $EXPECTED_DEV_PATH; do
        check_vid_and_pid=$(udevadm info $dev_path |
                            grep -E "(ID_MODEL_ID=$EXPECTED_PID|ID_VENDOR_ID=$EXPECTED_VID)" -wc)
        if [ "$check_vid_and_pid" == 2 ]
        then
            DEV_PATH=$dev_path
        fi
    done
fi
if [ -z $DEV_PATH ]
then
    echo "Can't find expected tty device."
    exit
fi
BAUD_RATE=1000000

# 2. Add module to the Linux kernel
sudo modprobe slcan

# 3. Run daemon slcand from can-utils - link serial interface with a virtual CAN device
# It will get name slcan name base
#   -o              option means open command
#   -s8             option means 1000 Kbit/s CAN bitrate
#   -t hw           option means UART flow control
#   -S $BAUD_RATE   option means uart baud rate
#   $DEV_PATH       position argument means port name
# sudo slcand -o -s8 -t hw -S $BAUD_RATE $DEV_PATH
sudo slcand -o -c -f -s8 -t hw -S $BAUD_RATE $DEV_PATH


sudo ip link set up slcan0
slcan_attach $DEV_PATH
sudo ip link set slcan0 txqueuelen 100