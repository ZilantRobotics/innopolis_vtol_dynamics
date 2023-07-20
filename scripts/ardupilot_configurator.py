#!/usr/bin/env python3
import time
from os.path import dirname, abspath

from vehicle import Vehicle

ardupilot_configs = [
    "ardupilot_cyphal_lua/cuav_v5.yaml",
    "ardupilot_cyphal_lua/lua_parameters.yaml",
]

if __name__ == '__main__':
    configs = ardupilot_configs
    path = dirname(dirname(abspath(__file__))) + "/configs/"

    vehicle = Vehicle("ArduPilot")
    vehicle.connect()
    vehicle.reset_params_to_default()
    for config in configs:
        print("")
        print(f"Config {config}:")
        vehicle.configure(path + config) #, reboot=True)

    # # For ardupilot only:
    time.sleep(5)
    vehicle.force_calibrate()
    print("force calibrated")
    vehicle.reboot()
