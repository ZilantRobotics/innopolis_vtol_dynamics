#!/bin/bash

catkin build innopolis_vtol_dynamics    \
             timed_roslaunch            \
             rosbridge_suite            \
             inno_sim_interface         \
             geographiclib_conversions  \
             uavcan_communicator        \
             uavcan_msgs