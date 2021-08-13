#!/bin/bash
cd "$(dirname "$0")"
source config.sh
sudo docker pull $DOCKERHUB_REPOSITOTY:$TAG_NAME
