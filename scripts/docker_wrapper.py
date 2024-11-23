#!/usr/bin/env python3
import os
import re
import time
import shlex
import logging
import platform
import subprocess
from typing import Optional
from pathlib import Path
from mode import SimMode

logger = logging.getLogger(__name__)

def is_valid_sniffer_path(sniffer_path: Optional[str]) -> bool:
    if not isinstance(sniffer_path, str):
        return False

    if platform.system() == 'Windows':
        return bool(re.match(r'^COM\d+$', sniffer_path, re.IGNORECASE))

    return os.path.exists(sniffer_path)

class DockerWrapper:
    COMMON_DOCKER_FLAGS = [
        '--rm',
        '--net=host',
        '-v', '/tmp/.X11-unix:/tmp/.X11-unix:rw',
        '-e', 'DISPLAY=:0',
        '-e', 'QT_X11_NO_MITSHM=1',
        '--privileged',
    ]
    REPOSITORY_DIR = Path(__file__).parent.parent.absolute()
    DOCKERFILE_DIR = REPOSITORY_DIR
    @staticmethod
    def build(full_image_name : str) -> None:
        cmd = ['docker', 'build', '-t', full_image_name, DockerWrapper.DOCKERFILE_DIR]
        subprocess.run(cmd, check=True)

    @staticmethod
    def get_container_id_by_image_name(full_image_name: str) -> Optional[str]:
        command = f"docker ps -q --filter ancestor={full_image_name}"
        list_of_identifiers = subprocess.check_output(command, shell=True, text=True).splitlines()
        if len(list_of_identifiers) == 0:
            return None

        return list_of_identifiers[0]

    @staticmethod
    def kill_container_by_id(container_identifier : str) -> None:
        if container_identifier is None:
            return

        cmd = ["docker", "kill", container_identifier]
        logger.debug(' '.join(cmd))
        subprocess.run(cmd, check=True)

    @staticmethod
    def run_container(sniffer_path: str,
                      image_name: str,
                      sim_config: str,
                      mode: SimMode) -> Optional[subprocess.Popen]:
        if not is_valid_sniffer_path(sniffer_path):
            raise RuntimeError(f"CAN-Sniffer has not been found (sniffer_path={sniffer_path})")

        if mode == SimMode.CYPHAL_HITL:
            flags = [
                *DockerWrapper.COMMON_DOCKER_FLAGS,
                '-v', f'{sniffer_path}:{sniffer_path}',
                '-e', f'CYPHAL_DEV_PATH_SYMLINK={sniffer_path}',
            ]
        elif mode == SimMode.DRONECAN_HITL:
            flags = [
                *DockerWrapper.COMMON_DOCKER_FLAGS,
                '-v', f'{sniffer_path}:{sniffer_path}',
                '-e', f'DRONECAN_DEV_PATH_SYMLINK={sniffer_path}',
            ]
        elif mode == SimMode.MAVLINK_SITL:
            flags = [
                *DockerWrapper.COMMON_DOCKER_FLAGS,
            ]

        command = [
            "docker",
            "container",
            'run',
            *flags,
            image_name,
            './scripts/run_sim.sh',
            sim_config
        ]
        logger.info(" ".join([shlex.quote(arg) for arg in command]))

        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE) as process:
            time.sleep(0.3)
            returncode = process.poll()
            if returncode is not None:
                logger.critical("The process PID=%s exited with code=%s.", process.pid, returncode)
                stdout, stderr = process.communicate(timeout=0.1)

                stdout_decoded = stdout.decode()
                if stdout_decoded.endswith("\n"):
                    stdout_decoded = stdout_decoded[:-1]
                logger.critical("STDOUT: %s", stdout_decoded)

                stderr_decoded = stderr.decode()
                if stderr_decoded.endswith("\n"):
                    stderr_decoded = stderr_decoded[:-1]
                logger.critical("STDERR: %s", stderr_decoded)

                raise RuntimeError(stderr_decoded)

            logger.info("The container (PID=%s) ran successfully.", process.pid)
            return process
