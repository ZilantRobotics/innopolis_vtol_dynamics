#!/usr/bin/env python3
"""
  UAV Cyphal/DroneCAN HITL Simulator is Hardware In The Loop simulator for PX4
  quadcopters and VTOLs. It works in such a way that the hardware knows nothing
  about the simulation. It covers more modules than standard SITL and HITL.

  sim.py is an interactive CLI for working with the UAV HITL Simulator with
  Docker. It automatically does:
  1. Set all the necessary Docker flags,
  2. Find the autopilot, upload the firmware and parameters to it,
  3. Find CAN-sniffer and create SLCAN/Socketcan interface,
  4. Configure and run the core HITL simulator.

  More details: https://github.com/ZilantRobotics/innopolis_vtol_dynamics
"""
import os
import sys
import time
import logging
import datetime
import subprocess
from typing import Optional
from pathlib import Path
from argparse import ArgumentParser, RawTextHelpFormatter
from autopilot_tools.configurator import AutopilotConfigurator

from mode import SimMode
from command import SimCommand, CommandsManager
from docker_wrapper import DockerWrapper
from model import SimModel

REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
VEHICLES_DIR = os.path.join(REPO_DIR, "configs", "vehicles")
BINARY_OUTPUT_PATH = os.path.join(REPO_DIR, "firmware.bin")
LOG_FILENAME = f"log_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
LOGS_DIR = os.path.join(REPO_DIR, "logs")
LOG_PATH = os.path.join(LOGS_DIR, LOG_FILENAME)
ISSUES_URL = "https://github.com/ZilantRobotics/innopolis_vtol_dynamics/issues"

logger = logging.getLogger(__name__)

class SimCommander:
    def __init__(self, model: SimModel) -> None:
        assert isinstance(model, SimModel)
        self._model = model
        self.command = None

    def __del__(self):
        if self.command.name not in ['rviz']:
            self._kill()

    def execute(self, command: SimCommand) -> None:
        assert isinstance(command, SimCommand)
        self.command = command

        command_actions = {
            "kill": self.execute_kill_command,
            "build": self.execute_build_command,
            "rviz": self.execute_rviz_command,
            "px4-sitl": self.execute_px4_sitl,
            "monitor": lambda: None,  # No action for monitor
        }

        action = command_actions.get(command.name, self.execute_run_simulation_command)
        action()

    def execute_kill_command(self) -> None:
        self._kill()
        sys.exit(0)

    def execute_build_command(self) -> None:
        DockerWrapper.build(self._model.full_image_name)
        sys.exit(0)

    def execute_rviz_command(self) -> None:
        rviz_dir = "rviz"
        repo_url = "git@github.com:PonomarevDA/rviz_docker.git"
        if not os.path.exists(rviz_dir) or not os.listdir(rviz_dir):
            logger.info("Cloning repository from %s into %s...", repo_url, rviz_dir)
            try:
                subprocess.run(["git", "clone", repo_url, rviz_dir], check=True)
            except subprocess.CalledProcessError:
                logger.critical("Failed to clone repository from %s. Exiting.", repo_url)
                sys.exit(1)
        if not os.path.exists(rviz_dir):
            logger.critical("The directory 'rviz' does not exist after cloning. Exiting.")
            sys.exit(1)

        scripts = ['build.sh', 'run.sh']
        for script in scripts:
            script_path = os.path.join(rviz_dir, script)
            if not os.path.isfile(script_path):
                print(f"Error: {script_path} not found. Exiting.")
                sys.exit(1)
            try:
                subprocess.run(["bash", script_path, *self.command.args], check=True)
            except subprocess.CalledProcessError:
                logger.critical("Script %s failed. Exiting.", script)
                sys.exit(1)
        sys.exit(0)

    def execute_px4_sitl(self) -> None:
        logger.info("Execute %s", self.command)
        image_name = "ponomarevda/px4-sitl"
        DockerWrapper.build(image_name, f"{REPO_DIR}/scripts/px4-sitl")
        DockerWrapper.interactive(image=image_name)
        sys.exit(0)

    def execute_run_simulation_command(self) -> None:
        try:
            self._kill()

            if self.command.mode in [SimMode.DRONECAN_HITL, SimMode.CYPHAL_HITL]:
                self._execute_hitl_container(self.command)
            elif self.command.mode in [SimMode.MAVLINK_SITL]:
                self._execute_sitl_container(self.command)

        except (RuntimeError, ValueError) as err:
            self._model.log((
                "Failed to start the Docker container with Simulator.\n"
                f'Reason: "{err}".\n'
                "\n"
                "Please, fix the issue and run the simulator again.\n"
                f"If you don't know how to fix it, open an issue: {ISSUES_URL}.\n"
                f"Provide a log file {LOG_PATH} and text or screenshot of the error.\n"
                "\n"
                "Press CTRL+C to exit.."
            ))
            self._kill()

    def _execute_hitl_container(self, command) -> None:
        if command.need_upload_firmware or command.need_load_parameters:
            config_path = f"{VEHICLES_DIR}/{command.name}.yaml"
            AutopilotConfigurator.configure_with_yaml_file(config_path,
                                                           command.need_upload_firmware,
                                                           command.need_load_parameters)

        process = DockerWrapper.run_sim_container(mode=command.mode,
                                                  image_name=self._model.full_image_name,
                                                  argument=command.sim_config,
                                                  sniffer_path=self._model.transport)
        self._model.add_process(process)

    def _execute_sitl_container(self, command) -> None:
        process = DockerWrapper.run_sim_container(mode=command.mode,
                                                  image_name=self._model.full_image_name,
                                                  argument=command.sim_config)
        self._model.add_process(process)

    def _kill(self) -> None:
        DockerWrapper.kill_container_by_id(self._model.docker_info.id)
        self._model.docker_info.id = None

class SimView:
    def __init__(self, model: SimModel, command: Optional[str]) -> None:
        assert isinstance(model, SimModel)
        self.model = model
        self.command = command

    def process(self) -> None:
        assert isinstance(self.command, str) or self.command is None

        if not self.model.update():
            return

        rows, _ = os.popen('stty size', 'r').read().split()
        max_log_rows = int(rows) - 11
        os.system('clear')
        print("\033c" + str(self.model))

        log_buffer = self.model.log_buffer
        if len(log_buffer) <= 0:
            return

        for line in log_buffer[-max_log_rows:]:
            print(line)
        print("--------------------------------------------------------------------------------")

def setup_logging():
    Path(LOGS_DIR).mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                        filename=LOG_PATH,
                        filemode='w')
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
    console.setFormatter(formatter)
    logging.getLogger().addHandler(console)

    argv_string = " ".join(sys.argv)
    logger.debug("Running %s", argv_string)

def main():
    setup_logging()

    parser = ArgumentParser(description=__doc__, formatter_class=RawTextHelpFormatter)

    parser.add_argument('command', help=CommandsManager.get_help(), nargs='+')

    upload_help = "upload the required firmware"
    parser.add_argument("--upload", help=upload_help, default=False, action='store_true')

    configure_help = "reset all the parameters to default and configure"
    parser.add_argument("--configure", help=configure_help, default=False, action='store_true')

    force_help = "both upload and then configure"
    parser.add_argument("--force", help=force_help, default=False, action='store_true')

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()

    command = CommandsManager.parse_command(args.command)
    if command is None:
        print(f"Unknown command {args.command}.")
        parser.print_help()
        sys.exit(1)

    command.need_upload_firmware = args.force or args.upload
    command.need_load_parameters = args.force or args.configure

    model = SimModel()
    view = SimView(model, command.name)
    commander = SimCommander(model=model)
    commander.execute(command)

    try:
        while True:
            view.process()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

if __name__ == "__main__":
    main()
