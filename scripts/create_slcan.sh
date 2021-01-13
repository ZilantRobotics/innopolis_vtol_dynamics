#!/bin/bash
# This script creates virtual CAN port using slcan
# You can use it in 2 ways:
# ./scripts/create_slcan.sh - use default tty port - /dev/ttyACM1
# ./scripts/create_slcan.sh /dev/ttyACMx - use custom

# 1. tty settings
if [ $# == 1 ]
then
    TTY_PORT=$1
else
    TTY_PORT=/dev/ttyACM0
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
#   $TTY_PORT       position argument means port name
sudo slcand -o -s8 -t hw -S $BAUD_RATE $TTY_PORT


sudo ip link set up slcan0
slcan_attach $TTY_PORT
sudo ip link set slcan0 txqueuelen 100