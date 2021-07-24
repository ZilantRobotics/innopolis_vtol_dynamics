#!/bin/bash
cd "$(dirname "$0")"
source config.sh
sudo docker push $DOCKERHUB_REPOSITOTY:$TAG_NAME
