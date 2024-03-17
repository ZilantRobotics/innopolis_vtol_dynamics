#!/bin/bash
CURRENT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPOSITORY_DIR="$(dirname "$(dirname "$CURRENT_DIR")")"
SCRIPTS_DIR=$REPOSITORY_DIR/scripts

# Kill previous runned containers, just in case
$SCRIPTS_DIR/docker.sh kill
sleep 1

$SCRIPTS_DIR/configure.sh px4_v1_14_0_cyphal_quadplane_vtol
$SCRIPTS_DIR/docker.sh csv &
sleep 15 # px4 needs time after booting up to initialize everything
$SCRIPTS_DIR/test_scenario.sh --mission $CURRENT_DIR/sviyazhsk_vtol.plan
$SCRIPTS_DIR/docker.sh kill
