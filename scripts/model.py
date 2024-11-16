#!/usr/bin/env python3
import os
import sys
import time
import glob
import platform
import subprocess
from typing import Optional
from dataclasses import dataclass
from raccoonlab_tools.common.device_manager import DeviceManager
from docker_wrapper import DockerWrapper
from colors import Colors

@dataclass
class ContainerInfo:
    version: str
    architecture: str
    id: Optional[int] = None
    cpu_load_pct: Optional[int] = 0

    @property
    def full_name(self) -> str:
        return f"ponomarevda/uavcan_hitl_dynamics_simulator:{self.version}{self.architecture}"

    def update(self) -> bool:
        self.id = DockerWrapper.get_container_id_by_image_name(self.full_name)
        return False

    def __str__(self) -> str:
        if self.id is None:
            string = f"{Colors.WARNING}None{Colors.ENDC}"
        else:
            string = f"{Colors.OKGREEN}{self.id}{Colors.ENDC}"
        return string


class AutopilotInterface:
    KNOWN_AUTOPILOTS = {
        "/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5"      : "px4_fmu-v5",
        "/dev/serial/by-id/usb-3D_Robotics_PX4_BL_FMU_v5"   : "px4_fmu-v5",

        "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X"        : "px4_fmu-v6x",
        "/dev/serial/by-id/usb-Auterion_PX4_BL_FMU_v6X"     : "px4_fmu-v6x",

        "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C"        : "px4_fmu-v6c",
        "/dev/serial/by-id/usb-Auterion_PX4_BL_FMU_v6C"     : "px4_fmu-v6c",
    }
    def __init__(self) -> None:
        self._autopilot = None

    def update(self) -> bool:
        autopilot = None
        for device_path, autopilot_type in AutopilotInterface.KNOWN_AUTOPILOTS.items():
            connected_devices = glob.glob(device_path + '*')
            for connected_device in connected_devices:
                if os.path.exists(connected_device):
                    autopilot = autopilot_type
                    break

        updated = autopilot != self._autopilot
        self._autopilot = autopilot
        return updated

    @property
    def autopilot_name(self) -> Optional[str]:
        """
        Known autopilots: px4_fmu-v5, px4_fmu-v6x, px4_fmu-v6c
        """
        return self._autopilot

    def __str__(self) -> str:
        if self._autopilot is None:
            string = f"{Colors.WARNING}Unknown{Colors.ENDC}"
        else:
            string = f"{Colors.OKGREEN}{self._autopilot}{Colors.ENDC}"
        return string

class SnifferInterface:
    def __init__(self) -> None:
        self._transports = []

    def update(self) -> bool:
        try:
            transports = DeviceManager.find_transports()
        except (TypeError, IndexError):
            transports = None

        updated = transports != self._transports
        self._transports = transports

        return updated

    @property
    def sniffer(self) -> str:
        if self._transports is None or len(self._transports) == 0:
            transport = "Undefined"
        else:
            transport = self._transports[0].port
        return transport

    def __str__(self) -> str:
        if self._transports is None or len(self._transports) == 0:
            string = f"{Colors.WARNING}Not connected{Colors.ENDC}"
        else:
            string = f"{Colors.OKGREEN}{self._transports[0].port}{Colors.ENDC}"
        return string

class SimModel:
    LOG_BUFFER_MAX_SIZE = 50

    def __init__(self) -> None:
        self._docker_info = ContainerInfo(
            version=self._get_latest_tag(),
            architecture=self._get_docker_architecture(),
        )
        self._elapsed_seconds = 0
        self._start_time = time.time()
        self._log_buffer = []
        self._process = None
        self._autopilot_interface = AutopilotInterface()
        self._sniffer_interface = SnifferInterface()
        self.update()

    @property
    def docker_info(self) -> ContainerInfo:
        return self._docker_info

    @property
    def full_image_name(self) -> str:
        return self._docker_info.full_name

    @property
    def autopilot_name(self) -> Optional[str]:
        return self._autopilot_interface.autopilot_name

    @property
    def log_buffer(self) -> list:
        return self._log_buffer

    @property
    def transport(self) -> str:
        return self._sniffer_interface.sniffer

    @property
    def time(self) -> int:
        return self._elapsed_seconds

    def update(self) -> bool:
        """
        Return True if model is updated and View should be updated, othwerwise False
        """
        self._update_process()
        updated = False
        updated += self._update_elapsed_seconds()
        updated += self._sniffer_interface.update()
        updated += self._autopilot_interface.update()
        updated += self._docker_info.update()

        return updated

    def add_process(self, process: subprocess.Popen) -> None:
        assert isinstance(process, subprocess.Popen)
        self._process = process

    @staticmethod
    def _get_latest_tag() -> str:
        cmd = "git describe --tags --abbrev=0"
        return subprocess.check_output(cmd, shell=True, text=True).strip()

    @staticmethod
    def _get_docker_architecture() -> str:
        architecture = platform.machine()

        if 'aarch64' in architecture:
            docker_architecture = 'arm64'
        elif 'x86_64' in architecture:
            docker_architecture = 'amd64'
        else:
            print("Unknown architecture")
            sys.exit()

        return docker_architecture

    def _update_elapsed_seconds(self) -> bool:
        elapsed_seconds = int(time.time() - self._start_time)
        updated = self._elapsed_seconds != elapsed_seconds
        self._elapsed_seconds = elapsed_seconds
        return updated

    def _update_process(self) -> bool:
        if self._process is None:
            return False

        output = self._process.stdout.readline()
        if output == b'' and self._process.poll() is not None:
            return False

        if output:
            line = output.strip().decode()
            self.log(line)

        return True

    def log(self, line: str):
        self._log_buffer.append(line)
        if len(self._log_buffer) > SimModel.LOG_BUFFER_MAX_SIZE:
            self._log_buffer.pop(0)

    def __str__(self) -> str:
        _, columns = os.popen('stty size', 'r').read().split()
        separator = "-" * int(columns)

        return (
            f"UAV HITL Simulator {self.docker_info.version} ({self.docker_info.architecture})\n"
            f"{separator}\n"
            f"Sniffer      : {self._sniffer_interface}\n"
            f"Autopilot    : {self._autopilot_interface}\n"
            f"Container    : {self.docker_info}\n"
            f"Command      : ?\n"
            f"Time         : {self.time}\n"
            f"{separator}\n"
        )

def main():
    model = SimModel()
    while model.time < 60:
        if not model.update():
            continue
        print("\033c" + str(model))

if __name__ == "__main__":
    main()
