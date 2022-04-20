#!/bin/bash

print_help() {
   echo "Wrapper under docker API for Innopolis VTOL Dynamics simulator.
It encapsulates all necessary docker flags for working with the package and properly handles image versions.
https://github.com/InnopolisAero/innopolis_vtol_dynamics

usage: docker.sh [build | pull | push | hitl_inno_vtol | interactive | kill | help]

Commands:
build                   Build docker image.
pull                    Pull docker image.
push                    Push docker image.
hitl_inno_vtol          Run dynamics simulator in HITL mode for inno_vtol airframe (VTOL)
hitl_flight_goggles     Run dynamics simulator in HITL mode for flight_goggles airframe (MR)
interactive             Run container in interactive mode.
test                    Run tests.
kill                    Kill all containers.
help                    Print this message and exit"
}

setup_config() {
    TAG_NAME=v0.3.1
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

    source ./uavcan_tools/get_sniffer_symlink.sh

    DOCKER_FLAGS="--privileged -v $DEV_PATH_SYMLINK:$DEV_PATH_SYMLINK       \
                 --net=host                                                 \
                 -e DEV_PATH_SYMLINK=$DEV_PATH_SYMLINK                      \
                 -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"                      \
                 -e DISPLAY=$DISPLAY                                        \
                 -e QT_X11_NO_MITSHM=1)"

    echo "TAG_NAME is" $TAG_NAME
    echo "DOCKER_CONTAINER_NAME is" $DOCKER_CONTAINER_NAME
    echo "DEV_PATH_SYMLINK is" $DEV_PATH_SYMLINK
}

build_docker_image() {
    setup_config
    docker build -t $DOCKER_CONTAINER_NAME ..
}

pull_docker_image() {
    setup_config
    docker pull $DOCKER_CONTAINER_NAME
}

push_docker_image() {
    setup_config
    docker push $DOCKER_CONTAINER_NAME
}

hitl_inno_vtol() {
    setup_config
    docker container run --rm $DOCKER_FLAGS $DOCKER_CONTAINER_NAME ./scripts/start_hitl_inno_vtol.sh
}

hitl_flight_goggles() {
    setup_config
    docker container run --rm $DOCKER_FLAGS $DOCKER_CONTAINER_NAME ./scripts/start_hitl_flight_goggles.sh
}

sitl_inno_vtol() {
    echo "Not implemented yet..."
}

sitl_flight_goggles() {
    echo "Not implemented yet..."
}

run_interactive() {
    setup_config
    xhost +local:docker
    docker container run --rm -it $DOCKER_FLAGS $DOCKER_CONTAINER_NAME /bin/bash
}

kill_all_containers() {
    docker kill $(docker ps -q)
}

test() {
    setup_config
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
elif [ "$1" = "interactive" ]; then
    run_interactive
elif [ "$1" = "test" ]; then
    test
elif [ "$1" = "kill" ]; then
    kill_all_containers
else
    print_help
fi
