#!/bin/bash

TAG_NAME=v0.3.0
DOCKERHUB_REPOSITOTY=ponomarevda/uavcan_hitl_dynamics_simulator

if uname -m | grep -q 'aarch64'; then
   TAG_NAME="$TAG_NAME""arm64"
elif uname -m | grep -q 'x86_64'; then
   TAG_NAME="$TAG_NAME""amd64"
else
   echo "unknown architecture"
   exit
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source $SCRIPT_DIR/../uavcan_tools/get_sniffer_symlink.sh
FLAGS_FOR_DEV="--privileged -v $DEV_PATH_SYMLINK:$DEV_PATH_SYMLINK --net=host"
FLAGS_FOR_ENV_VARS="-e DEV_PATH_SYMLINK=$DEV_PATH_SYMLINK"

echo "docker/config.sh: TAG_NAME is" $TAG_NAME
echo "docker/config.sh: DOCKERHUB_REPOSITOTY is" $DOCKERHUB_REPOSITOTY
echo "docker/config.sh: DEV_PATH_SYMLINK is" $DEV_PATH_SYMLINK
