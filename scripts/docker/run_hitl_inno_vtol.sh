#!/bin/bash
cd "$(dirname "$0")"
source config.sh

sudo docker container run --rm                                      \
                          $FLAGS_FOR_DEV                            \
                          $FLAGS_FOR_ENV_VARS                       \
                          $DOCKERHUB_REPOSITOTY:$TAG_NAME           \
                          /bin/bash scripts/start_hitl_inno_vtol.sh