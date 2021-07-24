#!/bin/bash
sudo docker save -o ../landing_station_image.tar $DOCKERHUB_REPOSITOTY:$TAG_NAME
