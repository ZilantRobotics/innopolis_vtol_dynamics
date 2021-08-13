#!/bin/bash
cd "$(dirname "$0")"
source config.sh
sudo docker container run --privileged -v /dev/bus/usb:/dev/bus/usb \
                          --net=host                                \
                          $DOCKERHUB_REPOSITOTY:$TAG_NAME