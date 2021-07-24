#!/bin/bash
cd "$(dirname "$0")"
source config.sh

sudo docker build -t $DOCKERHUB_REPOSITOTY:$TAG_NAME ../..
