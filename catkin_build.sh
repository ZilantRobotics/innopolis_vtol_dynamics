#!/bin/bash

catkin build \
    innopolis_vtol_dynamics \
    timed_roslaunch \
    mavlink mavros \
    rosbridge_suite \
    inno_sim_interface \
    drone_communicators