#!/bin/bash

print_help() {
   echo "Wrapper under ROS API for Innopolis VTOL Dynamics simulator.
It automatically run all auxilliary scripts required to each specific mode and source necessary setup.bash files.
It supports all possible simulator modes.

https://github.com/RaccoonlabDev/innopolis_vtol_dynamics

Usage: run_sim.sh <command>

Commands:
  px4_v1_14_0_beta_dronecan_vtol          Run dynamics simulator in DroneCan HITL mode for px4 vtol 13000 airframe
  px4_v1_12_1_dronecan_vtol               Run dynamics simulator in DroneCan HITL mode for px4 vtol 13070 airframe
  dronecan_flight_goggles                 Run dynamics simulator in DroneCan HITL mode for flight_goggles airframe
  cyphal_quadrotor                        Cyphal HITL PX4 Quadrotor (4001)
  cyphal_octorotor                        Cyphal HITL PX4 Octorotor (12001)
  cyphal_standard_vtol                    Run dynamics simulator in Cyphal HITL mode for inno_vtol airframe.
  cyphal_and_dronecan_inno_vtol           Run dynamics simulator in DroneCan + Cyphal mode for inno_vtol airframe.
                                          This mode uses 2 serial ports and is in the alpha testing stage yet.
  sitl_inno_vtol                          Run dynamics simulator in MAVLink SITL mode for inno_vtol airframe
  sitl_flight_goggles                     Run dynamics simulator in MAVLink SITL mode for flight_goggles airframe
  sitl_inno_vtol_with_flight_stack        Run dynamics simulator in MAVLink SITL mode for inno_vtol airframe (with additional including px4.launch)
  sitl_flight_goggles_with_flight_stack   Run dynamics simulator in MAVLink SITL mode for flight_goggles airframe (with additional including px4.launch)

Auxilliary commands:
  ros                                     Source ROS and catkin_ws setup.bash files
  help                                    Print this message and exit"
}

setup_ros() {
    # /opt/ros/$ROS_DISTRO/setup.bash should be either run first, or not run at all
    # If catkin_ws is not found, do nothing.
    DOCKER_CATKIN_WS_SETUP_BASH_PATH=/catkin_ws/devel/setup.bash
    MOST_PROBABLE_CATKIN_WS_SETUP_BASH_PATH=~/catkin_ws/devel/setup.bash
    if [ -f "$DOCKER_CATKIN_WS_SETUP_BASH_PATH" ]; then
        source /opt/ros/$ROS_DISTRO/setup.bash
        source $DOCKER_CATKIN_WS_SETUP_BASH_PATH
    elif [ -f "$MOST_PROBABLE_CATKIN_WS_SETUP_BASH_PATH" ]; then
        source /opt/ros/$ROS_DISTRO/setup.bash
        source $MOST_PROBABLE_CATKIN_WS_SETUP_BASH_PATH
    fi
}

setup_sitl_px4_flight_stack() {
    source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo
}

setup_dronecan_hitl() {
    if [ ! -z $DRONECAN_DEV_PATH_SYMLINK ]; then
        echo "Trying to create slcan0 for dronecan..."
        $SCRIPT_DIR/tools/can/create_slcan.sh -d $DRONECAN_DEV_PATH_SYMLINK -i slcan0
    fi
    if [[ -z $(ifconfig | grep slcan0) ]]; then
        echo "HITL can't be started without CAN interface!"
        exit 0
    fi
}

setup_cyphal_hitl() {
    if [ ! -z $CYPHAL_DEV_PATH_SYMLINK ]; then
        echo "Trying to create slcan0 for cyphal/serial..."
        $SCRIPT_DIR/tools/can/create_slcan.sh -d $CYPHAL_DEV_PATH_SYMLINK -i slcan0
    fi
    if [[ -z $(ifconfig | grep slcan0) ]]; then
        echo "HITL can't be started without CAN interface!"
        exit 0
    fi
    source $SCRIPT_DIR/cyphal_config_slcan0.sh
}

setup_combined_hitl() {
    if [ ! -z $DRONECAN_DEV_PATH_SYMLINK ]; then
        echo "Trying to create slcan0 for dronecan..."
        $SCRIPT_DIR/tools/can/create_slcan.sh -d $DRONECAN_DEV_PATH_SYMLINK -i slcan0
    fi
    if [ ! -z $CYPHAL_DEV_PATH_SYMLINK ]; then
        echo "Trying to create slcan1 for cyphal..."
        $SCRIPT_DIR/tools/can/create_slcan.sh -d $CYPHAL_DEV_PATH_SYMLINK -i slcan1
        source $SCRIPT_DIR/cyphal_config_slcan1.sh
    fi
}

px4_v1_14_0_beta_dronecan_vtol() {
    setup_ros
    setup_dronecan_hitl
    $SCRIPT_DIR/airframe_printer.sh 13000
    roslaunch innopolis_vtol_dynamics hitl.launch   \
        run_dronecan_communicator:=true             \
        vehicle:=innopolis_vtol                     \
        vehicle_params:=vtol_params                 \
        mixer:=px4_v1_14_0_vtol_13000_mixer         \
        dynamics:=vtol_dynamics
}

px4_v1_12_1_dronecan_vtol() {
    setup_ros
    setup_dronecan_hitl
    $SCRIPT_DIR/airframe_printer.sh 13000
    roslaunch innopolis_vtol_dynamics hitl.launch   \
        run_dronecan_communicator:=true             \
        vehicle:=innopolis_vtol                     \
        vehicle_params:=vtol_params                 \
        mixer:=vtol_13070_mixer                     \
        dynamics:=vtol_dynamics
}

dronecan_flight_goggles() {
    setup_ros
    setup_dronecan_hitl
    $SCRIPT_DIR/airframe_printer.sh 4001
    roslaunch innopolis_vtol_dynamics hitl.launch   \
        run_dronecan_communicator:=true             \
        vehicle:=iris                               \
        vehicle_params:=quadrotor_params            \
        mixer:=direct_mixer                         \
        dynamics:=quadcopter
}

cyphal_quadrotor() {
    setup_ros
    setup_cyphal_hitl
    $SCRIPT_DIR/airframe_printer.sh 4001
    roslaunch innopolis_vtol_dynamics hitl.launch   \
        run_cyphal_communicator:=true               \
        vehicle:=iris                               \
        vehicle_params:=quadrotor_params            \
        mixer:=direct_mixer                         \
        dynamics:=quadcopter
}

cyphal_octorotor() {
    setup_ros
    setup_cyphal_hitl
    $SCRIPT_DIR/airframe_printer.sh 12001
    roslaunch innopolis_vtol_dynamics hitl.launch   \
        run_cyphal_communicator:=true               \
        vehicle:=iris                               \
        vehicle_params:=octorotor_params            \
        mixer:=direct_mixer                         \
        dynamics:=octorotor
}

cyphal_standard_vtol() {
    setup_ros
    setup_cyphal_hitl
    $SCRIPT_DIR/airframe_printer.sh 13000
    roslaunch innopolis_vtol_dynamics hitl.launch   \
        run_cyphal_communicator:=true               \
        vehicle:=innopolis_vtol                     \
        vehicle_params:=vtol_params                 \
        mixer:=direct_mixer                         \
        dynamics:=vtol_dynamics
}

cyphal_and_dronecan_inno_vtol() {
    setup_ros
    setup_combined_hitl
    $SCRIPT_DIR/airframe_printer.sh 4001
    roslaunch innopolis_vtol_dynamics hitl.launch   \
        run_cyphal_communicator:=true               \
        run_dronecan_communicator:=true             \
        vehicle:=iris                               \
        vehicle_params:=quadrotor_params_ardupilot  \
        mixer:=direct_mixer                         \
        dynamics:=quadcopter
}

sitl_inno_vtol() {
    setup_ros
    roslaunch innopolis_vtol_dynamics sitl.launch   \
        vehicle:=innopolis_vtol                     \
        vehicle_params:=vtol_params                 \
        mixer:=vtol_13070_mixer                     \
        dynamics:=vtol_dynamics                     \
        run_sitl_flight_stack:="false"
}

sitl_flight_goggles() {
    setup_ros
    roslaunch innopolis_vtol_dynamics sitl.launch   \
        vehicle:=iris                               \
        vehicle_params:=quadrotor_params            \
        mixer:=direct_mixer                         \
        dynamics:=quadcopter                        \
        run_sitl_flight_stack:="false"
}

sitl_inno_vtol_with_flight_stack() {
    setup_ros
    setup_sitl_px4_flight_stack
    roslaunch innopolis_vtol_dynamics sitl.launch   \
        vehicle:=innopolis_vtol                     \
        vehicle_params:=quadrotor_params            \
        mixer:=vtol_13070_mixer                     \
        dynamics:=vtol_dynamics                     \
        run_sitl_flight_stack:="true"
}

sitl_flight_goggles_with_flight_stack() {
    setup_ros
    setup_sitl_px4_flight_stack
    roslaunch innopolis_vtol_dynamics sitl.launch   \
        vehicle:=iris                               \
        vehicle_params:=quadrotor_params            \
        mixer:=direct_mixer                         \
        dynamics:=quadcopter                        \
        run_sitl_flight_stack:="true"
}


# Entry point
if [ "${BASH_SOURCE[0]}" -ef "$0" ]; then
    set -e
fi
SCRIPT_DIR="$(dirname "$0")"

if [ "$1" = "px4_v1_14_0_beta_dronecan_vtol" ]; then
    px4_v1_14_0_beta_dronecan_vtol
elif [ "$1" = "px4_v1_12_1_dronecan_vtol" ]; then
    px4_v1_12_1_dronecan_vtol
elif [ "$1" = "dronecan_flight_goggles" ]; then
    dronecan_flight_goggles
elif [ "$1" = "cyphal_quadrotor" ]; then
    cyphal_quadrotor
elif [ "$1" = "cyphal_octorotor" ]; then
    cyphal_octorotor
elif [ "$1" = "cyphal_standard_vtol" ]; then
    cyphal_standard_vtol
elif [ "$1" = "cyphal_and_dronecan_inno_vtol" ]; then
    cyphal_and_dronecan_inno_vtol
elif [ "$1" = "sitl_inno_vtol" ]; then
    sitl_inno_vtol
elif [ "$1" = "sitl_flight_goggles" ]; then
    sitl_flight_goggles
elif [ "$1" = "sitl_inno_vtol_with_flight_stack" ]; then
    sitl_inno_vtol_with_flight_stack
elif [ "$1" = "sitl_flight_goggles_with_flight_stack" ]; then
    sitl_flight_goggles_with_flight_stack
elif [ "$1" = "ros" ]; then
    setup_ros
else
    print_help
fi
