#!/bin/bash

echo ""
echo "Automatical configuration:"
echo "1. Download firmware"
echo "2. Upload firmware to the autopilot"
echo "3. Configure parameters"

./px4/upload.sh px4_fmu-v5_cyphal.px4

if [[ $1 == 4001 ]]; then
  ./parameters_configurator.py px4_v1_14_cyphal_quadcopter_configs
elif [[ $1 == 12001 ]]; then
  ./parameters_configurator.py px4_v1_14_cyphal_octorotor_configs
elif [[ $1 == 13000 ]]; then
  ./parameters_configurator.py px4_v1_14_cyphal_inno_quadcopter_configs
elif [[ $1 == 13050 ]]; then
  echo $1
fi
