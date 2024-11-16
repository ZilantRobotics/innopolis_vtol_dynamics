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
from typing import Optional
from pathlib import Path
from argparse import ArgumentParser, RawTextHelpFormatter
from autopilot_tools.configurator import AutopilotConfigurator

from command import SimCommand
from docker_wrapper import DockerWrapper
from model import SimModel

REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
VEHICLES_DIR = os.path.join(REPO_DIR, "configs", "vehicles")
BINARY_OUTPUT_PATH = os.path.join(REPO_DIR, "firmware.bin")
LOG_FILENAME = f"log_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
LOGS_DIR = os.path.join(REPO_DIR, "logs")
LOG_PATH = os.path.join(LOGS_DIR, LOG_FILENAME)

COMMANDS = [
    SimCommand(name="build", alias='b', mode=None, info="Build the Docker image"),
    SimCommand(name="kill", alias='', mode=None, info="Kill the running Docker container"),
    SimCommand(name="monitor", alias='', mode=None, info="Just monitor"),
    *SimCommand.create_list_from_directory(dir_with_yaml_files=VEHICLES_DIR)
]

class SimCommander:
    def __init__(self, model: SimModel) -> None:
        assert isinstance(model, SimModel)
        self._model = model

    def __del__(self):
        self._kill()

    def execute(self, command: SimCommand, need_upload_firmware: bool, need_load_parameters: bool) -> None:
        assert isinstance(command, SimCommand)

        if command.name == "kill":
            self._kill()
            sys.exit(0)

        if command.name == "build":
            self._build()
            sys.exit(0)

        if need_upload_firmware or need_load_parameters:
            config_path = f"{VEHICLES_DIR}/{command.name}.yaml"
            AutopilotConfigurator.configure_with_yaml_file(config_path=config_path,
                                                           need_upload_firmware=need_upload_firmware,
                                                           need_load_parameters=need_load_parameters)

        if command.mode is None:
            return

        try:
            self._kill()
            process = DockerWrapper.run_container(sniffer_path=self._model.transport,
                                                  image_name=self._model.full_image_name,
                                                  sim_config=command.sim_config,
                                                  mode=command.mode)
            self._model.add_process(process)
        except Exception as err:
            self._model.log((
                "Failed to start the Docker container with HITL simulator. "
                f'Reason: "{err}". '
                "Please, fix the issue and run the simulator again."
            ))
            self._model.log("\nPress CTRL+C to exit...")
            self._kill()

    def _kill(self) -> None:
        DockerWrapper.kill_container_by_id(self._model.docker_info.id)

    def _build(self) -> None:
        DockerWrapper.build(self._model.full_image_name)

class SimView:
    def __init__(self, model: SimModel) -> None:
        assert isinstance(model, SimModel)
        self.model = model

    def process(self, command: Optional[str]) -> None:
        assert isinstance(command, str) or command is None

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


def main():
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

    parser = ArgumentParser(description=__doc__, formatter_class=RawTextHelpFormatter)

    command_help = f'{"Command":<36} {"Alias":<10} {"Info"}\n'
    command_help += '\n'.join([f"{cmd.name:<36} {cmd.alias:<10} {cmd.info}" for cmd in COMMANDS])
    parser.add_argument('command', help=command_help)

    upload_help = ("upload the required firmware")
    parser.add_argument("--upload", help=upload_help, default=False, action='store_true')

    configure_help = ("reset all the parameters to default and configure")
    parser.add_argument("--configure", help=configure_help, default=False, action='store_true')

    force_help = ("both upload and then configure")
    parser.add_argument("--force", help=force_help, default=False, action='store_true')

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()

    if args.force:
        args.upload = True
        args.configure = True

    command = next((cmd for cmd in COMMANDS if cmd.check(args.command)), None)
    if command is None:
        print(f"Unknown command {args.command}.")
        parser.print_help()
        sys.exit(1)

    model = SimModel()
    view = SimView(model)
    commander = SimCommander(model=model)
    commander.execute(command, args.upload, args.configure)

    try:
        while True:
            view.process(command.name)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

if __name__ == "__main__":
    main()
