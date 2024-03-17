#!/usr/bin/env python3
"""
Autopilot configurator:
1. Download a firmware and upload to a target if it is specified in yaml file
2. Configure autopilot parameters if it is specified in yaml file
"""
import os
import subprocess
import wget
from argparse import ArgumentParser
import yaml

SCRIPTS_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_DIR = os.path.dirname(SCRIPTS_DIR)
BINARY_OUTPUT_PATH = f"{REPO_DIR}/firmware.bin"

def configure(yaml_path : str) -> None:
    with open(yaml_path, 'r') as file:
        config = yaml.safe_load(file)

    flags = []

    firmware = config.get('firmware')
    if firmware is not None:
        if os.path.exists(BINARY_OUTPUT_PATH):
            os.remove(BINARY_OUTPUT_PATH)
        wget.download(firmware, out=BINARY_OUTPUT_PATH)
        flags += ['--firmware', BINARY_OUTPUT_PATH]

    params = config.get('params')
    if params is not None and len(params) > 0:
        flags += ['-f', '--config']
        for param in params:
            flags.append(f"{REPO_DIR}/{param}")

    if len(flags) > 0:
        subprocess.run(["autopilot-configurator", "-v", *flags])
        if os.path.exists(BINARY_OUTPUT_PATH):
            os.remove(BINARY_OUTPUT_PATH)

def configure_cli():
    """
    ArgumentParser wrapper under configure() function.
    This function can be used in pyproject.toml
    """
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('path', help='Path to config .yaml file')
    args = parser.parse_args()
    configure(args.path)

if __name__ == "__main__":
    configure_cli()
