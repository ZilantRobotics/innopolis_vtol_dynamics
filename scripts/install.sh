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
    git                         \
    psmisc                      \
    can-utils                   \
    net-tools                   \
    iproute2                    \
    ros-$ROS_DISTRO-catkin      \
    ros-$ROS_DISTRO-rosauth     \
    ros-$ROS_DISTRO-mavros      \
    ros-$ROS_DISTRO-mavlink     \
    ros-$ROS_DISTRO-tf          \
    ros-$ROS_DISTRO-tf2         \
    ros-$ROS_DISTRO-tf2-ros     \
    ros-$ROS_DISTRO-mavros-msgs \
    ros-$ROS_DISTRO-rosbridge-server \
    python3-pip \
    python3-catkin-tools

pip install bson pymongo protobuf Pillow twisted

# For ROS Melodic only:
if [[ "$ROS_DISTRO" = "melodic" ]]; then
    apt-get install -y python-pip python-catkin-tools
fi

# Hack because there is an issue on >=71.0.0 https://github.com/pypa/setuptools/issues/4478
pip install setuptools==68.0.0 importlib-metadata==8.2.0

# Required for Github Action CI
kernel_release=$(uname -r)
if [[ $kernel_release == *azure ]]; then
    $SUDO apt-get install $FORCE_APT_INSTALL linux-modules-extra-$(uname -r)
fi
