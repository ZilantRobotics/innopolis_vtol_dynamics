#!/bin/bash

cd "$(dirname "$0")"
source /opt/ros/$ROS_DISTRO/setup.bash
source /catkin_ws/devel/setup.bash

roslaunch innopolis_vtol_dynamics sitl.launch   \
    vehicle:=innopolis_vtol                     \
    airframe:=inno_standard_vtol                \
    dynamics:=inno_vtol