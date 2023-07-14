#!/bin/bash

echo "Automatical configuration:"
echo "1. Download firmware"
echo "2. Upload firmware to the autopilot"
echo "3. Configure parameters"

./px4/upload.sh px4_fmu-v5_cyphal.px4

if [[ $1 == 4001 ]]; then
  echo $1
elif [[ $1 == 12001 ]]; then
  echo $1
elif [[ $1 == 13000 ]]; then
  echo $1
elif [[ $1 == 13050 ]]; then
  echo $1
fi
