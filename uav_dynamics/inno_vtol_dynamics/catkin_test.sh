#!/bin/bash

set -e
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

# Setup ROS environment variables if inside a container
key="$1"
case $key in
    -d|--docker)
        source /opt/ros/$ROS_DISTRO/setup.bash
        source /catkin_ws/devel/setup.bash
        ;;
*)
    ;;
esac

# Run tests
roscore &
roslaunch innopolis_vtol_dynamics load_parameters.launch --wait
catkin run_tests --no-deps innopolis_vtol_dynamics

killall -9 roscore
killall -9 rosmaster
