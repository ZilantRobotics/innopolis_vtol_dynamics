#!/bin/bash

cd "$(dirname "$0")"

BUILD_DIR=/home/nex/Firmware/build/px4_fmu-v5_default
UPLOADER_DIR=/home/nex/Firmware/Tools/px_uploader.py
PORT_NAME="/dev/serial/by-id/*-3D_Robotics_PX4_FMU_v5.x*,/dev/serial/by-id/*PX4_BL*,/dev/serial/by-id/*BL_FMU*,/dev/serial/by-id/usb-The_Autopilot*,/dev/serial/by-id/usb-Bitcraze*,/dev/serial/by-id/pci-Bitcraze*,/dev/serial/by-id/usb-Gumstix*,/dev/serial/by-id/usb-Hex_ProfiCNC*,/dev/serial/by-id/usb-UVify*,/dev/serial/by-id/usb-ArduPilot*"
BINARY_PATH=/home/nex/Firmware/build/px4_fmu-v5_default/px4_fmu-v5_default.px4

cd $BUILD_DIR && /usr/bin/python3 $UPLOADER_DIR --force --port $PORT_NAME $BINARY_PATH