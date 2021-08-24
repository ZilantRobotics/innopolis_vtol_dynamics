#!/bin/bash

cd "$(dirname "$0")"
source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash

roslaunch innopolis_vtol_dynamics sitl.launch   \
    vehicle:=iris                               \
    airframe:=iris                              \
    dynamics:=flightgoggles_multicopter