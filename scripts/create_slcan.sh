#!/bin/bash
# This script creates virtual CAN port using slcan

TTY_PORT=/dev/ttyACM0
BAUD_RATE=1000000

sudo modprobe slcan
sudo slcand -o -s8 -t hw -S $BAUD_RATE $TTY_PORT
sudo ip link set up slcan0
slcan_attach $TTY_PORT
sudo ip link set slcan0 txqueuelen 100