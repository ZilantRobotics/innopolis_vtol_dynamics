#!/bin/bash

cd "$(dirname "$0")"

roslaunch innopolis_vtol_dynamics sitl.launch   \
    vehicle:=innopolis_vtol                     \
    airframe:=inno_standard_vtol                \
    dynamics:=inno_vtol