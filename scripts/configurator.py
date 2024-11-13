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
import logging
import datetime
from argparse import ArgumentParser
from autopilot_tools.configurator import AutopilotConfigurator

REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
BINARY_OUTPUT_PATH = os.path.join(REPO_DIR, "firmware.bin")
LOG_FILENAME = f"log_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
LOG_PATH = os.path.join(REPO_DIR, LOG_FILENAME)

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
    AutopilotConfigurator.configure_with_yaml_file(args.path, False, True)

if __name__ == "__main__":
    main()
