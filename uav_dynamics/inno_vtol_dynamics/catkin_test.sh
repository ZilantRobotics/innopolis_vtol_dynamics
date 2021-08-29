#!/bin/bash
{
    source /opt/ros/$ROS_DISTRO/setup.bash
    source /catkin_ws/devel/setup.bash

    roscore &
    sleep 1

    roslaunch innopolis_vtol_dynamics load_parameters.launch
    catkin run_tests --no-deps innopolis_vtol_dynamics
}

killall -9 roscore
killall -9 rosmaster
