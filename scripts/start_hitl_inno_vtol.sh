#!/bin/bash

cd "$(dirname "$0")"

./uavcan_tools/create_slcan_from_serial.sh
roslaunch innopolis_vtol_dynamics hitl.launch   \
    vehicle:=innopolis_vtol                     \
    airframe:=inno_standard_vtol                \
    dynamics:=inno_vtol