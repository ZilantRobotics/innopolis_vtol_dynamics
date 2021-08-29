#!/bin/bash
cd "$(dirname "$0")"
source config.sh

sudo docker container run --rm                                      \
                          $FLAGS_FOR_DEV                            \
                          $DOCKERHUB_REPOSITOTY:$TAG_NAME           \
                          /bin/bash uav_dynamics/inno_vtol_dynamics/catkin_test.sh