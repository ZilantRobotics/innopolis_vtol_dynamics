#!/bin/bash

# Description:
# -p 11311:11311/tcp
# -p 14552:14552/udp
# -p 12345:12345/udp
# -p 12346:12346/udp
# Additionaly:
# --device=/dev/ttyACM0

cd "$(dirname "$0")"
source config.sh
sudo docker container run --privileged -v /dev/bus/usb:/dev/bus/usb \
                          -it                                       \
                          --net=host                                \
                          $DOCKERHUB_REPOSITOTY:$TAG_NAME           \
                          /bin/bash
