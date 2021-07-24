#!/bin/bash
sudo docker container run -p 14552:14552/udp                        \
                          -p 11311:11311/tcp                        \
                          --privileged -v /dev/bus/usb:/dev/bus/usb \
                          $DOCKERHUB_REPOSITOTY:$TAG_NAME
