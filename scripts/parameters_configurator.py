#!/usr/bin/env python3
import sys
import time
from os.path import dirname, abspath
from pathlib import Path

SCRIPT_DIR = Path(dirname(abspath(__file__)))
SRC_DIR = SCRIPT_DIR.parent.parent / "autopilot_tools" / "src"
sys.path.append(SRC_DIR.absolute().as_posix())
from autopilot_tools.vehicle import Vehicle

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

px4_v1_14_cyphal_octorotor_configs = [
    "px4_v1.14_octorotor_airframe.yaml",
    "px4_cyphal.yaml",
    "px4_common.yaml"
]

px4_v1_14_cyphal_inno_quadcopter_configs = [
    "px4_v1.14_inno_quadcopter_airframe.yaml",
    "px4_cyphal.yaml",
    "px4_common.yaml"
]

supported_modes = {
    "px4_v1_13_dronecan_inno_vtol_configs"      : px4_v1_13_dronecan_inno_vtol_configs,
    "px4_v1_14_cyphal_quadcopter_configs"       : px4_v1_14_cyphal_quadcopter_configs,
    "px4_v1_14_cyphal_octorotor_configs"        : px4_v1_14_cyphal_octorotor_configs,
    "px4_v1_14_cyphal_inno_quadcopter_configs"  : px4_v1_14_cyphal_inno_quadcopter_configs,
}

def print_help():
    print("Usage: `python3 configurate_autopilot.py <mode>`")
    print(f"Supported modes: {list(supported_modes.keys())}")

if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] not in supported_modes:
        print_help()
        exit()

    mode = sys.argv[1]
    configs = supported_modes[mode]
    path = dirname(dirname(abspath(__file__))) + "/configs/"

    vehicle = Vehicle()
    vehicle.connect()
    vehicle.reset_params_to_default()
    for config in configs:
        print("")
        print(f"Config {config}:")
        vehicle.configure(path + config, reboot=True)

    if mode == "ardupilot_lua_configs":
        time.sleep(5)
        vehicle.force_calibrate()
        vehicle.reboot()
