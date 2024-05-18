#!/usr/bin/env python3
from enum import Enum

class SimMode(Enum):
    MONITOR = 0
    CYPHAL_HITL = 1
    DRONECAN_HITL = 2
    MAVLINK_SITL = 3

    @staticmethod
    def create_from_string(string: str):
        string_to_mode = {
            "cyphal": SimMode.CYPHAL_HITL,
            "dronecan": SimMode.DRONECAN_HITL,
            "mavlink": SimMode.MAVLINK_SITL,
            "monitor": SimMode.MONITOR,
        }
        return string_to_mode.get(string)
