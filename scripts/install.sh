#!/bin/bash
# Install everything required for UAV HITL Simulator

set -e

is_root="$(whoami)"
if [[ $is_root =~ "root" ]]; then
    SUDO=""
else
    SUDO="sudo"
fi

# Required for Github Action CI
if [[ $1 == "--yes" ]]; then
    FORCE_APT_INSTALL="-y"
fi

$SUDO apt-get install $FORCE_APT_INSTALL \
    git \
    ros-$ROS_DISTRO-catkin \
    python3-pip
