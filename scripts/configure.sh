#!/bin/bash
SCRIPT_NAME=$(basename $BASH_SOURCE)
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPOSITORY_DIR="$(dirname "$SCRIPT_DIR")"
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color
print_help() {
   echo "usage: docker.sh COMMAND [OPTIONS]

Automatical configuration:
1. Download firmware
2. Upload firmware to the autopilot
3. Configure parameters"
}

px4_v1_12_1_dronecan_vtol() {
    wget -O $DOWNLOADED_BINARY_PATH $PX4_V1_12_1_DRONECAN_URL
    autopilot-configurator -v --firmware $DOWNLOADED_BINARY_PATH -f --config \
        ${REPOSITORY_DIR}/configs/px4/v1.13/vtol_13070/airframe.yaml \
        ${REPOSITORY_DIR}/configs/px4/dronecan.yaml \
        ${REPOSITORY_DIR}/configs/px4/common.yaml
}

if [ -z $1 ]; then
    printf "$RED$SCRIPT_NAME ERROR (line ${LINENO}): Argument is not specified!$NC\n"
    exit 1
fi

# Check autopilot_tools version
autopilot_tools_version=$(pip show autopilot_tools | grep Version)
if [ ! -z "$autopilot_tools_version" ]; then
    IFS=':' read -ra version <<< "$autopilot_tools_version"
    IFS='.' read -ra version <<< ${version[1]}
    if (( ${version[1]} >= 4 )); then
        printf "$SCRIPT_NAME INFO: autopilot-tools: $autopilot_tools_version\n"
    else
        printf "$YELLOW$SCRIPT_NAME WARN: autopilot-tools: $autopilot_tools_version (Consider updating)$NC\n"
    fi
else
    printf "$RED$SCRIPT_NAME ERROR (line ${LINENO}): autopilot-tools is not installed!$NC\n"
    exit 1
fi


DOWNLOADS_DIR=$REPOSITORY_DIR/downloads
DOWNLOADED_BINARY_PATH=$DOWNLOADS_DIR/px4_fmu-v5_latest_downloaded_firmware.px4
PX4_V1_12_1_DRONECAN_URL=https://github.com/ZilantRobotics/PX4-Autopilot/releases/download/v1.12.1_hitl/px4_fmu-v5_default.px4


mkdir -p $DOWNLOADS_DIR
rm -f $DOWNLOADED_BINARY_PATH

if [[ $1 == "px4_v1_12_1_dronecan_vtol" ]]; then
    px4_v1_12_1_dronecan_vtol
else
    printf "$RED$SCRIPT_NAME ERROR (line ${LINENO}): Unknown argument: '$1' $NC\n"
    exit 1
fi
