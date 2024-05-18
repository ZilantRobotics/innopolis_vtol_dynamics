#!/usr/bin/env python3
import subprocess
from typing import Optional
from pathlib import Path
from mode import SimMode

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

        subprocess.run(["docker", "kill", container_identifier], check=True)

    @staticmethod
    def run_container(sniffer_path: str, image_name: str, sim_config: str, mode: SimMode):
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

        return subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
