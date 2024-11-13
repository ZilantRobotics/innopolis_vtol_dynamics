#!/bin/bash
# This software is distributed under the terms of the MIT License.
# Copyright (c) 2023 Dmitry Ponomarev.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
SCRIPT_NAME=$(basename $BASH_SOURCE)
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color
REPOSITORY_DIR="$(dirname "$SCRIPT_DIR")"
MISSIONS_DIR=$REPOSITORY_DIR/test_scenario

HELP="usage: $SCRIPT_NAME [--help] [--mission <mission_path>].

The utility runs a mission.

options:
    -m, --mission MISSION_PATH      Specify mission path
    -h, --help                      Show this help message and exit."

MISSION_PATH=$MISSIONS_DIR/ci/square.plan

function log_error() {
    lineno=($(caller))
    printf "$RED$SCRIPT_NAME ERROR on line ${lineno}: $1!$NC\n"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -m|--mission)
        MISSION_PATH="$2"
        shift
        ;;

        -h|--help)
        echo "$HELP"
        [[ "${BASH_SOURCE[0]}" -ef "$0" ]] && exit 0 || return 0
        ;;

        *)
        log_error "Unknown option: $1"
        echo "$HELP"
        [[ "${BASH_SOURCE[0]}" -ef "$0" ]] && exit 0 || return 1
        ;;
    esac
    shift
done


autopilot_tools_version=$(pip show autopilot_tools | grep Version)
if [ ! -z "$autopilot_tools_version" ]; then
    IFS=':' read -ra version <<< "$autopilot_tools_version"
    IFS='.' read -ra version <<< ${version[1]}
    echo -n "autopilot-tools: $autopilot_tools_version"
    if (( ${version[1]} >= 4 )); then
        echo ""
    else
        echo " (Consider updating)"
    fi
else
    log_error "autopilot-tools is not installed."
    exit
fi

log=$REPOSITORY_DIR/latest_log.ulg

rm -f $log
test-scenario --verbose $MISSION_PATH --device serial --timeout 500 --output $log --verbose
