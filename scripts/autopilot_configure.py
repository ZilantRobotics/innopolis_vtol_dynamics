#!/usr/bin/env python3
import sys
import time
from os.path import dirname, abspath

import mavlink_tools
from mavlink_tools.configurator.vehicle import Vehicle

ardupilot_cyphal_quadcopter_configs = [
    "ardupilot_cyphal_common_params.yaml",
    "ardupilot_cyphal_gazebo_registers.yaml",
]

px4_v1_13_dronecan_inno_vtol_configs = [
    "px4_v1.13_inno_vtol_airframe.yaml",
    "px4_dronecan.yaml",
    "px4_common.yaml"
]

px4_v1_14_cyphal_quadcopter_configs = [
    "px4_v1.14_quadcopter_airframe.yaml",
    "px4_cyphal.yaml",
    "px4_common.yaml"
]

supported_modes = {
    "px4_v1_13_dronecan_inno_vtol_configs" : px4_v1_13_dronecan_inno_vtol_configs,
    "px4_v1_14_cyphal_quadcopter_configs"  : px4_v1_14_cyphal_quadcopter_configs,
}

def print_help():
    print("Usage: `python3 configurate_autopilot.py <mode>`")
    print(f"Supported modes: {list(supported_modes.keys())}")

if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] not in supported_modes:
        print_help()
        exit()

    configs = supported_modes[sys.argv[1]]
    path = dirname(dirname(abspath(__file__))) + "/configs/"

    vehicle = Vehicle()
    vehicle.connect()
    vehicle.reset_params_to_default()
    for config in configs:
        print("")
        print(f"Config {config}:")
        vehicle.configure(path + config, reboot=True)

    # Reserved for ardupilot:
    # time.sleep(5)
    # vehicle.force_calibrate()
    # vehicle.reboot()
