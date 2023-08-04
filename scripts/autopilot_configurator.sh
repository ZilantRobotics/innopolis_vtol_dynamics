#!/bin/bash

echo ""
echo "Automatical configuration:"
echo "1. Download firmware"
echo "2. Upload firmware to the autopilot"
echo "3. Configure parameters"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPOSITORY_DIR="$(dirname "$SCRIPT_DIR")"
DOWNLOADS_DIR=$REPOSITORY_DIR/downloads
CYPHAL_BINARY_PATH=$DOWNLOADS_DIR/px4_fmu-v5_cyphal.px4
CYPHAL_BINARY_LINK=https://github.com/ZilantRobotics/PX4-Autopilot/releases/download/v1.14.0-0.3.0-beta1/px4_fmu-v5_cyphal.px4

download_cyphal_px4_firmware_and_load_it_to_autopilot() {
  mkdir $DOWNLOADS_DIR
  wget -O $CYPHAL_BINARY_PATH $CYPHAL_BINARY_LINK
  ./px4/upload.sh $CYPHAL_BINARY_PATH
}


if [[ $1 == 4001 ]]; then
  download_cyphal_px4_firmware_and_load_it_to_autopilot
  ./parameters_configurator.py px4_v1_14_cyphal_quadcopter_configs
elif [[ $1 == 12001 ]]; then
  download_cyphal_px4_firmware_and_load_it_to_autopilot
  ./parameters_configurator.py px4_v1_14_cyphal_octorotor_configs
elif [[ $1 == 13000 ]]; then
  download_cyphal_px4_firmware_and_load_it_to_autopilot
  ./parameters_configurator.py px4_v1_14_cyphal_inno_quadcopter_configs
elif [[ $1 == 13050 ]]; then
  echo $1
fi
