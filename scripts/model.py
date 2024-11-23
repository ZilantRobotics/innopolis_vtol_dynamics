#!/usr/bin/env python3
import os
import sys
import time
import glob
import logging
import platform
import subprocess
from typing import Optional
from dataclasses import dataclass
from raccoonlab_tools.common.device_manager import DeviceManager
from docker_wrapper import DockerWrapper
from colors import Colors

logger = logging.getLogger(__name__)

@dataclass
class ContainerInfo:
    tag: str
    architecture: str
    id: Optional[int] = None
    cpu_load_pct: Optional[int] = 0

    @property
    def full_name(self) -> str:
        return f"ponomarevda/uavcan_hitl_dynamics_simulator:{self.tag}{self.architecture}"

    @property
    def version(self) -> str:
        return f"{self.tag} ({self.architecture})"

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

        if autopilot == self._autopilot:
            return False

        logger.debug(f"Autopilot connection status has been updated: {autopilot}")
        self._autopilot = autopilot

        return True

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

        if transports == self._transports:
            return False

        logger.debug(f"Avaliable transports have been updated: {transports}")
        self._transports = transports

        return True

    @property
    def sniffer(self) -> str:
        if self._transports is None or len(self._transports) == 0:
            transport = None
        else:
            transport = self._transports[0].port
        return transport

    def __str__(self) -> str:
        if self._transports is None or len(self._transports) == 0:
            string = f"{Colors.WARNING}Not connected{Colors.ENDC}"
        else:
            string = f"{Colors.OKGREEN}{self._transports[0].port}{Colors.ENDC}"
        return string

class GitInterface:
    @staticmethod
    def get_latest_tag() -> str:
        cmd = "git describe --tags --abbrev=0"
        return subprocess.check_output(cmd, shell=True, text=True).strip()

    @staticmethod
    def get_current_commit() -> str:
        cmd = "git rev-parse HEAD"
        return subprocess.check_output(cmd, shell=True, text=True).strip()

    @staticmethod
    def get_branch_name() -> str:
        cmd = "git rev-parse --abbrev-ref HEAD"
        return subprocess.check_output(cmd, shell=True, text=True).strip()

    @staticmethod
    def get_dirty_state() -> str:
        cmd = "git diff --quiet && git diff --cached --quiet || echo 'dirty'"
        dirty_state = subprocess.check_output(cmd, shell=True, text=True).strip()
        return 'Yes' if dirty_state else 'No'

class SimModel:
    LOG_BUFFER_MAX_SIZE = 50

    def __init__(self) -> None:
        self._docker_info = ContainerInfo(
            tag=GitInterface.get_latest_tag(),
            architecture=self._get_docker_architecture(),
        )
        self._elapsed_seconds = 0
        self._start_time = time.time()
        self._log_buffer = []
        self._process = None
        self._command = " ".join(sys.argv[1:])
        self._autopilot_interface = AutopilotInterface()
        self._sniffer_interface = SnifferInterface()
        self.update()
        logger.debug("Git information:")
        logger.debug(f"  Latest tag: {self._docker_info.version}")
        logger.debug(f"  Current Commit: {GitInterface.get_current_commit()}")
        logger.debug(f"  Branch: {GitInterface.get_branch_name()}")
        logger.debug(f"  Working Directory Dirty: {GitInterface.get_dirty_state()}")

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
        logger.info(f"A process {process.pid} has been added to Model.")
        self._process = process

    @staticmethod
    def _get_docker_architecture() -> str:
        architecture = platform.machine()

        if architecture in ['aarch64', 'arm64']:
            docker_architecture = 'arm64'
        elif architecture in ['x86_64', 'amd64']:
            docker_architecture = 'amd64'
        else:
            logger.critical("Unknown architecture: {architecture}")
            sys.exit()

        return docker_architecture

    def _update_elapsed_seconds(self) -> bool:
        elapsed_seconds = int(time.time() - self._start_time)
        updated = self._elapsed_seconds != elapsed_seconds
        self._elapsed_seconds = elapsed_seconds
        return updated

    def _update_process(self) -> bool:
        """
        True the process state has been updated, otherwise False.
        """
        if self._process is None:
            return False

        returncode = self._process.poll()
        if returncode is not None:
            self.log(f"The process {self._process.pid} failed with code={returncode}.")
            stdout, stderr = self._process.communicate(timeout=0.1)

            stdout_decoded = stdout.decode()
            if stdout_decoded.endswith("\n"):
                stdout_decoded = stdout_decoded[:-1]
            self.log(f"STDOUT: {stdout_decoded}")

            stderr_decoded = stderr.decode()
            if stderr_decoded.endswith("\n"):
                stderr_decoded = stderr_decoded[:-1]
            self.log(f"STDERR: {stderr_decoded}")

            self._process = None
            return False

        output = self._process.stdout.readline()
        if output == b'':
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
            f"UAV HITL Simulator {self.docker_info.tag} ({self.docker_info.architecture})\n"
            f"{separator}\n"
            f"Sniffer      : {self._sniffer_interface}\n"
            f"Autopilot    : {self._autopilot_interface}\n"
            f"Container    : {self.docker_info}\n"
            f"Command      : {self._command}\n"
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
