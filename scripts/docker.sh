#!/bin/bash

print_help() {
   echo "usage: docker.sh COMMAND [OPTIONS]

Wrapper under docker API for Innopolis VTOL Dynamics simulator.
It encapsulates all necessary docker flags and properly handles image versions.
https://github.com/ZilantRobotics/innopolis_vtol_dynamics

Options:
  --force                       Upload the required firmware and update parameters.
                                This option has effect only in HITL mode.

Supported modes (with aliases):
-------------------------------------------------------------------------------
  Command                                   | Protocol    Autopilot SW    Airframe
-------------------------------------------------------------------------------
  px4_v1_15_0_cyphal_quadcopter,cq          | Cyphal      PX4 v1.15-alpha Quadrotor x (4001)
  px4_v1_14_0_cyphal_quadplane_vtol,csv     | Cyphal      PX4 v1.14-beta  Standard VTOL (13000)
  px4_v1_14_0_cyphal_octoplane_vtol,cv8     | Cyphal      PX4 v1.14-beta  VTOL 8 motors (13050)
  px4_v1_14_0_dronecan_quadrotor,dq         | DroneCAN    PX4 v1.14-beta  Quadrotor (4001)
  px4_v1_14_0_dronecan_quadplane_vtol,dv    | DroneCAN    PX4 v1.14-beta  Standard VTOL (13000)
  px4_v1_13_0_dronecan_vtol,dv1130          | DroneCAN    PX4 v1.13       vtol 13070
-------------------------------------------------------------------------------
  px4_v1_14_0_cyphal_octorotor,co           | Cyphal      PX4 v1.14-beta  Octorotor Coaxial (12001)
  sitl_inno_vtol                            | MAVLink     PX4 v1.12       vtol 13070
  sitl_flight_goggles                       | MAVLink     PX4 v1.12       Quadrotor (4001)
  cyphal_and_dronecan                       | 2 CAN       AP  v4.4.0      Copter
-------------------------------------------------------------------------------

Commands (with aliases):
  build,b                       Build docker image
  pull                          Pull docker image
  push                          Push docker image
  interactive,i                 Run container in interactive mode
  test                          Run tests
  kill                          Kill all containers
  help                          Print this message and exit"
}

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPOSITORY_DIR="$(dirname "$SCRIPT_DIR")"

# Kill the container if sniffer is disconnected
slcan_checker() {
    sleep 5
    while :; do
        sleep 1

        containers=$(docker ps -q --filter ancestor=$IMAGE_NAME)
        if [ -z "${containers}" ]; then
            exit
        fi

        if [[ -z $(ifconfig | grep slcan) ]]; then
            $SCRIPT_DIR/docker.sh kill
            break
        fi
    done
}

setup_image_name_and_version() {
    TAG_NAME=$(git describe --tags --abbrev=0)
    DOCKERHUB_REPOSITOTY=ponomarevda/uavcan_hitl_dynamics_simulator

    if uname -m | grep -q 'aarch64'; then
        TAG_NAME="$TAG_NAME""arm64"
    elif uname -m | grep -q 'x86_64'; then
        TAG_NAME="$TAG_NAME""amd64"
    else
        echo "Unknown architecture"
        exit
    fi
    IMAGE_NAME=$DOCKERHUB_REPOSITOTY:$TAG_NAME
}

setup_mavlink_sitl_config() {
    DOCKER_FLAGS="--net=host"
    DOCKER_FLAGS="$DOCKER_FLAGS -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1)"
}

setup_dronecan_hitl_config() {
    setup_mavlink_sitl_config
    if [ -z "$SNIFFER" ]; then
        source ./tools/can/create_slcan.sh --only-find
        DRONECAN_DEV_PATH_SYMLINK=$DEV_PATH
    else
        DRONECAN_DEV_PATH_SYMLINK=$SNIFFER
    fi

    if [ ! -z $DRONECAN_DEV_PATH_SYMLINK ]; then
        DOCKER_FLAGS="$DOCKER_FLAGS --privileged -v $DRONECAN_DEV_PATH_SYMLINK:$DRONECAN_DEV_PATH_SYMLINK"
        DOCKER_FLAGS="$DOCKER_FLAGS -e DRONECAN_DEV_PATH_SYMLINK=$DRONECAN_DEV_PATH_SYMLINK"
    fi

    echo "Docker Dronecan HITL settings:"
    echo "- IMAGE_NAME is" $IMAGE_NAME
    echo "- DRONECAN_DEV_PATH_SYMLINK is" $DRONECAN_DEV_PATH_SYMLINK
}

setup_cyphal_hitl_config() {
    setup_mavlink_sitl_config
    if [ -z "$SNIFFER" ]; then
        source ./tools/can/create_slcan.sh --only-find
        CYPHAL_DEV_PATH_SYMLINK=$DEV_PATH
    else
        CYPHAL_DEV_PATH_SYMLINK=$SNIFFER
    fi

    if [ ! -z $CYPHAL_DEV_PATH_SYMLINK ]; then
        DOCKER_FLAGS="$DOCKER_FLAGS --privileged -v $CYPHAL_DEV_PATH_SYMLINK:$CYPHAL_DEV_PATH_SYMLINK"
        DOCKER_FLAGS="$DOCKER_FLAGS -e CYPHAL_DEV_PATH_SYMLINK=$CYPHAL_DEV_PATH_SYMLINK"
    fi

    echo "Docker Cyphal HITL settings:"
    echo "- IMAGE_NAME is" $IMAGE_NAME
    echo "- CYPHAL_DEV_PATH_SYMLINK is" $CYPHAL_DEV_PATH_SYMLINK
}

setup_cyphal_and_dronecan_hitl_config() {
    setup_mavlink_sitl_config

    DRONECAN_DEV_PATH_SYMLINK="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0674FF524957778667133858-if02"
    CYPHAL_DEV_PATH_SYMLINK="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066AFF524957778667134207-if02"
    
    if [ ! -z $DRONECAN_DEV_PATH_SYMLINK ]; then
        DOCKER_FLAGS="$DOCKER_FLAGS --privileged -v $DRONECAN_DEV_PATH_SYMLINK:$DRONECAN_DEV_PATH_SYMLINK"
        DOCKER_FLAGS="$DOCKER_FLAGS -e DRONECAN_DEV_PATH_SYMLINK=$DRONECAN_DEV_PATH_SYMLINK"
    fi
    if [ ! -z $CYPHAL_DEV_PATH_SYMLINK ]; then
        DOCKER_FLAGS="$DOCKER_FLAGS --privileged -v $CYPHAL_DEV_PATH_SYMLINK:$CYPHAL_DEV_PATH_SYMLINK"
        DOCKER_FLAGS="$DOCKER_FLAGS -e CYPHAL_DEV_PATH_SYMLINK=$CYPHAL_DEV_PATH_SYMLINK"
    fi

    echo "Docker Cyphal and Dronecan HITL settings:"
    echo "- IMAGE_NAME is" $IMAGE_NAME
    echo "- DRONECAN_DEV_PATH_SYMLINK is" $DRONECAN_DEV_PATH_SYMLINK
    echo "- CYPHAL_DEV_PATH_SYMLINK is" $CYPHAL_DEV_PATH_SYMLINK
}

docker_build_image() {
    docker build -t $IMAGE_NAME ..
}

docker_pull_image() {
    docker pull $IMAGE_NAME
}

docker_push_image() {
    docker push $IMAGE_NAME
}

docker_countainer_run_interactive() {
    setup_cyphal_hitl_config
    docker container run --rm -it $DOCKER_FLAGS $IMAGE_NAME /bin/bash
}

docker_container_run_test() {
    setup_mavlink_sitl_config
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./uav_dynamics/uav_hitl_dynamics/catkin_test.sh --docker
}

docker_kill_all_related_containers() {
    containers=$(docker ps -q --filter ancestor=$IMAGE_NAME)
    if [ ! -z "${containers}" ]; then
        printf "Killing the following containers: "
        docker kill $containers
    fi
}


docker_container_run_dronecan() {
    docker_kill_all_related_containers
    setup_dronecan_hitl_config
    slcan_checker&
    if [[ $OPTIONS == "--force" ]]; then
        ${REPOSITORY_DIR}/scripts/configurator.py ${REPOSITORY_DIR}/configs/vehicles/${vehicle}.yaml
    fi
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./scripts/run_sim.sh ${vehicle}
}

docker_container_run_cyphal() {
    docker_kill_all_related_containers
    setup_cyphal_hitl_config
    slcan_checker&
    if [[ $OPTIONS == "--force" ]]; then
        ${REPOSITORY_DIR}/scripts/configurator.py ${REPOSITORY_DIR}/configs/vehicles/${vehicle}.yaml
    fi
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./scripts/run_sim.sh ${vehicle}
}

docker_container_run_mavlink() {
    docker_kill_all_related_containers
    setup_mavlink_sitl_config
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./scripts/run_sim.sh ${vehicle}
}

docker_container_run_cyphal_and_dronecan_inno_vtol() {
    echo "Cyphal and DroneCAN mode is a special mode that uses:"
    echo "- slcan0 based on the 1-st sniffer for DroneCAN communication (sensors)"
    echo "- slcan1 based on the 2-nd sniffer for Cyphal   communication (Actuators only)"
    docker_kill_all_related_containers
    setup_cyphal_and_dronecan_hitl_config
    slcan_checker&
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./scripts/run_sim.sh cyphal_and_dronecan_inno_vtol
}

## Start from here
set -e
cd "$(dirname "$0")"
setup_image_name_and_version
OPTIONS=$2

## Handle aliases
declare -A aliases=(
    [b]="build"
    [i]="interactive"
    [dv1130]="px4_v1_13_0_dronecan_vtol"
    [dv]="px4_v1_14_0_dronecan_quadplane_vtol"
    [dq]="px4_v1_14_0_dronecan_quadrotor"
    [cq]="px4_v1_15_0_cyphal_quadcopter"
    [co]="px4_v1_14_0_cyphal_octorotor"
    [csv]="px4_v1_14_0_cyphal_quadplane_vtol"
    [cv8]="px4_v1_14_0_cyphal_octoplane_vtol"
)
if [ $# -eq 0 ]; then
    CMD="help"
elif [[ -n "${aliases[$1]}" ]]; then
    CMD="${aliases[$1]}"
else
    CMD=$1
fi


# Common commands:
if [ "$CMD" = "build" ]; then
    docker_build_image
elif [ "$CMD" = "pull" ]; then
    docker_pull_image
elif [ "$CMD" = "push" ]; then
    docker_push_image
elif [ "$CMD" = "interactive" ]; then
    docker_countainer_run_interactive
elif [ "$CMD" = "test" ]; then
    docker_container_run_test
elif [ "$CMD" = "kill" ]; then
    docker_kill_all_related_containers

# DroneCAN commands:
elif [ "$CMD" = "px4_v1_13_0_dronecan_vtol" ] || \
     [ "$CMD" = "px4_v1_14_0_dronecan_quadplane_vtol" ] || \
     [ "$CMD" = "px4_v1_14_0_dronecan_quadrotor" ] ; then
    vehicle=$CMD
    docker_container_run_dronecan

# Cyphal commands:
elif [ "$CMD" = "px4_v1_15_0_cyphal_quadcopter" ] || \
     [ "$CMD" = "px4_v1_14_0_cyphal_octorotor" ] || \
     [ "$CMD" = "px4_v1_14_0_cyphal_quadplane_vtol" ] || \
     [ "$CMD" = "px4_v1_14_0_cyphal_octoplane_vtol" ] ; then
    vehicle=$CMD
    docker_container_run_cyphal

# MAVLink commands:
elif [ "$CMD" = "sitl_inno_vtol" ] || \
     [ "$CMD" = "sitl_flight_goggles" ] ; then
    vehicle=$CMD
    docker_container_run_mavlink

# DroneCAN + Cyphal commands:
elif [ "$CMD" = "cyphal_and_dronecan" ]; then
    docker_container_run_cyphal_and_dronecan_inno_vtol

# Help:
else
    print_help
fi
