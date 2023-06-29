#!/bin/bash

print_help() {
   echo "Wrapper under docker API for Innopolis VTOL Dynamics simulator.
It encapsulates all necessary docker flags and properly handles image versions.
https://github.com/RaccoonlabDev/innopolis_vtol_dynamics

usage: docker.sh [command]

Commands:
build (b)                       Build docker image.
pull                            Pull docker image.
push                            Push docker image.
dronecan_vtol (dv)              Run dynamics simulator in DroneCAN HITL mode for inno_vtol airframe
dronecan_iris                   Run dynamics simulator in DroneCAN HITL mode for flight_goggles airframe
sitl_inno_vtol                  Run dynamics simulator in MAVLink SITL mode for inno_vtol airframe
sitl_flight_goggles             Run dynamics simulator in MAVLink SITL mode for flight_goggles airframe
cyphal_inno_vtol (cv)           Run dynamics simulator in Cyphal HITL mode for inno_vtol airframe.
cyphal_and_dronecan_inno_vtol   Run dynamics simulator in DroneCAN + Cyphal HITL mode for inno_vtol airframe.
interactive (i)                 Run container in interactive mode.
test                            Run tests.
kill                            Kill all containers.
help                            Print this message and exit"
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
    source ./uavcan_tools/scripts/get_sniffer_symlink.sh
    DRONECAN_DEV_PATH_SYMLINK=$DEV_PATH_SYMLINK

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
    source ./uavcan_tools/scripts/get_sniffer_symlink.sh
    CYPHAL_DEV_PATH_SYMLINK=$DEV_PATH_SYMLINK

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

    echo "Docker Cyphal and Dronecan HITL settings:"
    echo "- IMAGE_NAME is" $IMAGE_NAME
    echo "- DRONECAN_DEV_PATH_SYMLINK is" $DRONECAN_DEV_PATH_SYMLINK
    echo "- CYPHAL_DEV_PATH_SYMLINK is" $CYPHAL_DEV_PATH_SYMLINK
}

build_docker_image() {
    docker build -t $IMAGE_NAME ..
}

pull_docker_image() {
    docker pull $IMAGE_NAME
}

push_docker_image() {
    docker push $IMAGE_NAME
}

dronecan_vtol() {
    kill_all_related_containers
    setup_dronecan_hitl_config
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./scripts/run_sim.sh dronecan_inno_vtol
}

dronecan_iris() {
    kill_all_related_containers
    setup_dronecan_hitl_config
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./scripts/run_sim.sh dronecan_flight_goggles
}

sitl_inno_vtol() {
    kill_all_related_containers
    setup_mavlink_sitl_config
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./scripts/run_sim.sh sitl_inno_vtol
}

sitl_flight_goggles() {
    kill_all_related_containers
    setup_mavlink_sitl_config
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./scripts/run_sim.sh sitl_flight_goggles
}

cyphal_inno_vtol() {
    kill_all_related_containers
    setup_cyphal_hitl_config
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./scripts/run_sim.sh cyphal_inno_vtol
}

cyphal_and_dronecan_inno_vtol() {
    kill_all_related_containers
    setup_cyphal_and_dronecan_hitl_config
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./scripts/run_sim.sh cyphal_and_dronecan_inno_vtol
}

run_interactive() {
    setup_cyphal_hitl_config
    docker container run --rm -it $DOCKER_FLAGS $IMAGE_NAME /bin/bash
}

kill_all_related_containers() {
    containers=$(docker ps -q --filter ancestor=$IMAGE_NAME)
    if [ ! -z "${containers}" ]; then
        printf "Killing the following containers: "
        docker kill $containers
    fi
}

test() {
    setup_mavlink_sitl_config
    docker container run --rm $DOCKER_FLAGS $IMAGE_NAME ./uav_dynamics/inno_vtol_dynamics/catkin_test.sh --docker
}

## Start from here
cd "$(dirname "$0")"
setup_image_name_and_version

if [ "$1" = "build" ] || [ "$1" = "b" ]; then
    build_docker_image
elif [ "$1" = "pull" ]; then
    pull_docker_image
elif [ "$1" = "push" ]; then
    push_docker_image
elif [ "$1" = "dronecan_vtol" ] || [ "$1" = "dv" ]; then
    dronecan_vtol
elif [ "$1" = "dronecan_iris" ]; then
    dronecan_iris
elif [ "$1" = "sitl_inno_vtol" ]; then
    sitl_inno_vtol
elif [ "$1" = "sitl_flight_goggles" ]; then
    sitl_flight_goggles
elif [ "$1" = "cyphal_inno_vtol" ] || [ "$1" = "cv" ]; then
    cyphal_inno_vtol
elif [ "$1" = "cyphal_and_dronecan_inno_vtol" ]; then
    cyphal_and_dronecan_inno_vtol
elif [ "$1" = "interactive" ] || [ "$1" = "i" ]; then
    run_interactive
elif [ "$1" = "test" ]; then
    test
elif [ "$1" = "kill" ]; then
    kill_all_related_containers
else
    print_help
fi
