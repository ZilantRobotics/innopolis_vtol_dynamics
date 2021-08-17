#!/bin/bash

cd "$(dirname "$0")"

roslaunch innopolis_vtol_dynamics sitl.launch   \
    vehicle:=iris                               \
    airframe:=iris                              \
    dynamics:=flightgoggles_multicopter