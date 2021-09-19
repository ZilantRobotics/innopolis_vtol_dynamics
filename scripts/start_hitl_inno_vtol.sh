#!/bin/bash

cd "$(dirname "$0")"
source /opt/ros/$ROS_DISTRO/setup.bash
source /catkin_ws/devel/setup.bash

set -e

./uavcan_tools/create_slcan_from_serial.sh $DEV_PATH_SYMLINK
roslaunch innopolis_vtol_dynamics hitl.launch   \
    vehicle:=innopolis_vtol                     \
    airframe:=inno_standard_vtol                \
    dynamics:=inno_vtol