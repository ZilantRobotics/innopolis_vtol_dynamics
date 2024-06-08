#!/usr/bin/env python3
# This program is free software under the GNU General Public License v3.
# See <https://www.gnu.org/licenses/> for details.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
"""
This script checks whether the project is ready for use:
- check submodules are initialized
- check python dependencies are installed
"""
import os
import sys
import logging
from pathlib import Path
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
import pkg_resources

REPO_DIR = Path(os.path.dirname(os.path.realpath(__file__))).parent.absolute()
BUILD_DIR = os.path.join(REPO_DIR, "build")
SUBMODULE_DIR = os.path.join(REPO_DIR, "uav_dynamics", "uav_hitl_dynamics")

def check_folder_exist_and_not_empty(dir) -> bool:
    return os.path.isdir(dir) and os.listdir(dir)

def check_python_requirements(requirements_file='requirements.txt') -> None:
    try:
        with open(requirements_file, 'r') as f:
            requirements = f.read().splitlines()
        pkg_resources.require(requirements)
    except pkg_resources.DistributionNotFound as e:
        logging.error(e)
        sys.exit(1)
    except pkg_resources.VersionConflict as e:
        logging.error(e)
        sys.exit(1)

def check_submodules() -> None:
    if not check_folder_exist_and_not_empty(SUBMODULE_DIR):
        logging.error("submodules not exist. Run: 'git submodule update --init --recursive'")
        sys.exit(1)

if __name__ == "__main__":
    check_python_requirements()
    check_submodules()
