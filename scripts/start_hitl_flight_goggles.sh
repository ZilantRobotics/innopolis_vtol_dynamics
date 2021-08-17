#!/bin/bash

cd "$(dirname "$0")"

./uavcan_tools/create_slcan_from_serial.sh
roslaunch innopolis_vtol_dynamics hitl.launch   \
    vehicle:=iris                               \
    airframe:=iris                              \
    dynamics:=flightgoggles_multicopter