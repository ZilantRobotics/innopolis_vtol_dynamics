#!/usr/bin/env python3
import os
from dataclasses import dataclass
from typing import Optional
import yaml
from mode import SimMode

@dataclass
class SimCommand:
    name: str
    alias: Optional[str]
    mode: SimMode
    info: Optional[str]
    sim_config: Optional[str] = None

    def check(self, name: str) -> bool:
        return name in [self.name, self.alias]

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
