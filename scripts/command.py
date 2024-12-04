#!/usr/bin/env python3
import os
from dataclasses import dataclass
from typing import Optional
import yaml
from mode import SimMode

REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
VEHICLES_DIR = os.path.join(REPO_DIR, "configs", "vehicles")

@dataclass
class SimCommand:
    name: str
    alias: Optional[str]
    mode: SimMode
    info: Optional[str]
    sim_config: Optional[str] = None
    args: Optional[list] = None
    need_upload_firmware: bool = False
    need_load_parameters: bool = False
    sniffer: Optional[str] = None

    def check(self, command: str) -> bool:
        if isinstance(command, str):
            name = command
        elif isinstance(command, list):
            name = command[0]
        else:
            raise ValueError("The 'name' argument must be a string or a list of strings.")

        res = name in [self.name, self.alias]
        if res:
            self.args = command[1:]
        return res

    @staticmethod
    def create_from_yaml_file(file_path: str):
        file_name = os.path.basename(file_path)
        if not file_name.endswith(".yaml"):
            return None

        with open(file_path, 'r', encoding='utf-8') as stream:
            yaml_data = yaml.safe_load(stream)
        cmd = SimCommand(
            name=file_name[:-5],
            alias=yaml_data.get("alias"),
            mode=SimMode.create_from_string(yaml_data.get("mode")),
            info=yaml_data.get("info"),
            sim_config=yaml_data.get("sim_config")
        )
        return cmd

    @staticmethod
    def create_list_from_directory(dir_with_yaml_files) -> list:
        commands = []
        all_files = sorted(os.listdir(dir_with_yaml_files), reverse=True)
        for file_name in all_files:
            file_path = os.path.join(dir_with_yaml_files, file_name)
            cmd = SimCommand.create_from_yaml_file(file_path)
            if cmd is not None:
                commands.append(cmd)
        return commands

class CommandsManager:
    COMMANDS = [
        SimCommand(name="build", alias='b', mode=None, info="Build the Docker image"),
        SimCommand(name="kill", alias='', mode=None, info="Kill the running Docker container"),
        SimCommand(name="monitor", alias='', mode=None, info="Just monitor"),
        SimCommand(name="rviz", alias='', mode=None, info="Run RVIZ"),
        SimCommand(name="px4-sitl", alias='', mode=None, info="Run PX4 SITL Flight Stack"),
        *SimCommand.create_list_from_directory(dir_with_yaml_files=VEHICLES_DIR)
    ]

    @staticmethod
    def get_all_commands():
        return CommandsManager.COMMANDS

    @staticmethod
    def get_help():
        header = f'{"Command":<36} {"Alias":<10} {"Info"}\n'

        commands_details = []
        for cmd in CommandsManager.COMMANDS:
            command_detail = f"{cmd.name:<36} {cmd.alias or '':<10} {cmd.info or ''}"
            commands_details.append(command_detail)

        return header + '\n'.join(commands_details)

    @staticmethod
    def parse_command(command: list) -> Optional[SimCommand]:
        command = next((cmd for cmd in CommandsManager.COMMANDS if cmd.check(command)), None)
        return command
