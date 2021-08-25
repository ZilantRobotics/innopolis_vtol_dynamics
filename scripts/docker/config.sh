#!/bin/bash

TAG_NAME=v0.3.0
DOCKERHUB_REPOSITOTY=ponomarevda/uavcan_hitl_dynamics_simulator
DEV_NAME=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF545151717867185622-if02

if uname -m | grep -q 'aarch64'; then
   TAG_NAME="$TAG_NAME""arm64"
elif uname -m | grep -q 'x86_64'; then
   TAG_NAME="$TAG_NAME""amd64"
else
    echo "unknown architecture"
    exit
fi
FLAGS_FOR_DEV="--privileged -v $DEV_NAME:$DEV_NAME --net=host"
