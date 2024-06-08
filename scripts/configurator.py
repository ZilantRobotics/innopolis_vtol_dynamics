#!/usr/bin/env python3
# This program is free software under the GNU General Public License v3.
# See <https://www.gnu.org/licenses/> for details.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
"""
Autopilot configurator:
1. Download a firmware and upload to a target if it is specified in yaml file
2. Configure autopilot parameters if it is specified in yaml file
"""
import os
import wget
import logging
import datetime
from argparse import ArgumentParser
import yaml
from autopilot_tools.px4.firmware_uploader import upload_firmware
from autopilot_tools.configurator.configurator import configure_params

REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
BINARY_OUTPUT_PATH = os.path.join(REPO_DIR, "firmware.bin")
LOG_FILENAME = f"log_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
LOG_PATH = os.path.join(REPO_DIR, LOG_FILENAME)

def configure(yaml_path: str, upload: bool = True, configure: bool = True) -> None:
    with open(yaml_path, 'r') as file:
        yaml_config = yaml.safe_load(file)

    firmware = yaml_config.get('firmware')
    if upload and firmware is not None:
        if os.path.exists(BINARY_OUTPUT_PATH):
            os.remove(BINARY_OUTPUT_PATH)

        wget.download(firmware, out=BINARY_OUTPUT_PATH)
        upload_firmware(firmware=BINARY_OUTPUT_PATH)

        if os.path.exists(BINARY_OUTPUT_PATH):
            os.remove(BINARY_OUTPUT_PATH)

    config_pathes = yaml_config.get('params')
    if configure and config_pathes is not None and len(config_pathes) > 0:
        for idx in range(len(config_pathes)):
            config_pathes[idx] = os.path.join(REPO_DIR, config_pathes[idx])
        configure_params(config_pathes=config_pathes, device="serial", force_calibrate=True)

def main():
    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                        filename=LOG_PATH,
                        filemode='w')
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
    console.setFormatter(formatter)
    logging.getLogger().addHandler(console)

    parser = ArgumentParser(description=__doc__)
    parser.add_argument('path', help='Path to config .yaml file')
    args = parser.parse_args()
    configure(args.path, False, True)

if __name__ == "__main__":
    main()
