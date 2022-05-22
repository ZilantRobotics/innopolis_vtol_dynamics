#!/bin/bash

print_help() {
   echo "Wrapper under docker API for Innopolis VTOL Dynamics simulator.
It encapsulates all necessary docker flags and properly handles image versions.
https://github.com/InnopolisAero/innopolis_vtol_dynamics

usage: docker.sh [command]

Commands:
build                   Build docker image.
pull                    Pull docker image.
push                    Push docker image.
hitl_inno_vtol          Run dynamics simulator in HITL mode for inno_vtol airframe
hitl_flight_goggles     Run dynamics simulator in HITL mode for flight_goggles airframe
sitl_inno_vtol          Run dynamics simulator in SITL mode for inno_vtol airframe
sitl_flight_goggles     Run dynamics simulator in SITL mode for flight_goggles airframe
cyphal_inno_vtol        Run dynamics simulator in dronecan + cyphal mode for inno_vtol airframe.
                        This mode uses 2 serial ports and is in the alpha testing stage yet.
interactive             Run container in interactive mode.
test                    Run tests.
kill                    Kill all containers.
help                    Print this message and exit"
}

setup_image_name_and_version() {
    TAG_NAME=v0.4.0
    DOCKERHUB_REPOSITOTY=ponomarevda/uavcan_hitl_dynamics_simulator

    if uname -m | grep -q 'aarch64'; then
        TAG_NAME="$TAG_NAME""arm64"
    elif uname -m | grep -q 'x86_64'; then
        TAG_NAME="$TAG_NAME""amd64"
    else
        echo "unknown architecture"
        exit
    fi
    DOCKER_CONTAINER_NAME=$DOCKERHUB_REPOSITOTY:$TAG_NAME
}

setup_mavlink_sitl_config() {
    setup_image_name_and_version
    DOCKER_FLAGS="--net=host -i -t"
    DOCKER_FLAGS="$DOCKER_FLAGS -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1)"
}

setup_dronecan_hitl_config() {
    setup_mavlink_sitl_config
    source ./uavcan_tools/get_sniffer_symlink.sh
    DRONECAN_DEV_PATH_SYMLINK=$DEV_PATH_SYMLINK

    if [ ! -z $DRONECAN_DEV_PATH_SYMLINK ]; then
        DOCKER_FLAGS="$DOCKER_FLAGS --privileged -v $DRONECAN_DEV_PATH_SYMLINK:$DRONECAN_DEV_PATH_SYMLINK"
        DOCKER_FLAGS="$DOCKER_FLAGS -e DRONECAN_DEV_PATH_SYMLINK=$DRONECAN_DEV_PATH_SYMLINK"
    fi

    echo "Docker Dronecan HITL settings:"
    echo "- DOCKER_CONTAINER_NAME is" $DOCKER_CONTAINER_NAME
    echo "- DRONECAN_DEV_PATH_SYMLINK is" $DRONECAN_DEV_PATH_SYMLINK
}

setup_cyphal_hitl_config() {
    setup_mavlink_sitl_config

    DRONECAN_DEV_PATH_SYMLINK="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0669FF535151726687231340-if02"
    CYPHAL_DEV_PATH_SYMLINK="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF575654888667251342-if02"
    
    if [ ! -z $DRONECAN_DEV_PATH_SYMLINK ]; then
        DOCKER_FLAGS="$DOCKER_FLAGS --privileged -v $DRONECAN_DEV_PATH_SYMLINK:$DRONECAN_DEV_PATH_SYMLINK"
        DOCKER_FLAGS="$DOCKER_FLAGS -e DRONECAN_DEV_PATH_SYMLINK=$DRONECAN_DEV_PATH_SYMLINK"
    fi
    if [ ! -z $CYPHAL_DEV_PATH_SYMLINK ]; then
        DOCKER_FLAGS="$DOCKER_FLAGS --privileged -v $CYPHAL_DEV_PATH_SYMLINK:$CYPHAL_DEV_PATH_SYMLINK"
        DOCKER_FLAGS="$DOCKER_FLAGS -e CYPHAL_DEV_PATH_SYMLINK=$CYPHAL_DEV_PATH_SYMLINK"
    fi

    echo "Docker Cyphal HITL settings:"
    echo "- DOCKER_CONTAINER_NAME is" $DOCKER_CONTAINER_NAME
    echo "- DRONECAN_DEV_PATH_SYMLINK is" $DRONECAN_DEV_PATH_SYMLINK
    echo "- CYPHAL_DEV_PATH_SYMLINK is" $CYPHAL_DEV_PATH_SYMLINK
}

build_docker_image() {
    setup_image_name_and_version
    docker build -t $DOCKER_CONTAINER_NAME ..
}

pull_docker_image() {
    setup_image_name_and_version
    docker pull $DOCKER_CONTAINER_NAME
}

push_docker_image() {
    setup_image_name_and_version
    docker push $DOCKER_CONTAINER_NAME
}

hitl_inno_vtol() {
    setup_dronecan_hitl_config
    docker container run --rm $DOCKER_FLAGS $DOCKER_CONTAINER_NAME ./scripts/run_sim.sh hitl_inno_vtol
}

hitl_flight_goggles() {
    setup_dronecan_hitl_config
    docker container run --rm $DOCKER_FLAGS $DOCKER_CONTAINER_NAME ./scripts/run_sim.sh hitl_flight_goggles
}

sitl_inno_vtol() {
    setup_mavlink_sitl_config
    docker container run --rm $DOCKER_FLAGS $DOCKER_CONTAINER_NAME ./scripts/run_sim.sh sitl_inno_vtol
}

sitl_flight_goggles() {
    setup_mavlink_sitl_config
    docker container run --rm $DOCKER_FLAGS $DOCKER_CONTAINER_NAME ./scripts/run_sim.sh sitl_flight_goggles
}

cyphal_inno_vtol() {
    setup_cyphal_hitl_config
    docker container run --rm $DOCKER_FLAGS $DOCKER_CONTAINER_NAME ./scripts/run_sim.sh cyphal_inno_vtol
}

run_interactive() {
    setup_dronecan_hitl_config
    docker container run --rm -it $DOCKER_FLAGS $DOCKER_CONTAINER_NAME /bin/bash
}

kill_all_containers() {
    docker kill $(docker ps -q)
}

test() {
    setup_mavlink_sitl_config
    docker container run --rm $DOCKER_FLAGS $DOCKER_CONTAINER_NAME ./uav_dynamics/inno_vtol_dynamics/catkin_test.sh --docker
}

cd "$(dirname "$0")"

if [ "$1" = "build" ]; then
    build_docker_image
elif [ "$1" = "pull" ]; then
    pull_docker_image
elif [ "$1" = "push" ]; then
    push_docker_image
elif [ "$1" = "hitl_inno_vtol" ]; then
    hitl_inno_vtol
elif [ "$1" = "hitl_flight_goggles" ]; then
    hitl_flight_goggles
elif [ "$1" = "sitl_inno_vtol" ]; then
    sitl_inno_vtol
elif [ "$1" = "sitl_flight_goggles" ]; then
    sitl_flight_goggles
elif [ "$1" = "cyphal_inno_vtol" ]; then
    cyphal_inno_vtol
elif [ "$1" = "interactive" ]; then
    run_interactive
elif [ "$1" = "test" ]; then
    test
elif [ "$1" = "kill" ]; then
    kill_all_containers
else
    print_help
fi
