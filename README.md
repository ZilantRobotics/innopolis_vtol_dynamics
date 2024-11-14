# UAV HITL Simulator [![catkin_build](https://github.com/ZilantRobotics/innopolis_vtol_dynamics/actions/workflows/catkin_build.yml/badge.svg)](https://github.com/ZilantRobotics/innopolis_vtol_dynamics/actions/workflows/catkin_build.yml)

UAV HITL Simulator brings up a set of ROS packages, config files and instructions to establish a full simulation for UAV based on PX4/Ardupilot autopilot.

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/dynamics.png" alt="drawing"/>

The key feature of this simulation is to run it in such a way that the hardware knows nothing about the simulation. This can be done with Cyphal/DroneCAN. It covers more PX4 modules than standard SITL and HITL.

**Purpose**

- Simulation of UAV onboard systems at a low hardware level
- Testing and debugging of UAV control systems using a CAN-bus
- Development and testing of intelligent automatic control systems for UAVs
- Training in the development and use of drones, including creating datasets and automated testing

**Requirements**

- Operating System: We've tailored the simulator for modern versions of Windows, Linux, and Mac. Choose the build that matches your OS.

- CPU: Aim for an Intel i7 from the 11th or 12th generation. For those using AMD, any equivalent processor will suffice.

- RAM: 16GB is a recommended minimum, but more is always better for performance.

- Hardware: CUAV V5+, CAN-sniffer

## 1. USE CASES

### 1.1. CI/CD Unit

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/use_case_1.png" alt="drawing" width="640"/>

### 1.2. HITL Simulator with desktop computer and 3D-simulator

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/use_case_2.png" alt="drawing" width="640"/>

### 1.3. UAV in HITL Mode

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/use_case_3.png" alt="drawing" width="640"/>

## 2. Usage

The simulator is distributed as a Docker image. To simplify the interraction with Docker, a `./scripts/sim.py` script was written. The script configures all the necessary Docker flags, performs automatic firmware upload, configuration, creates a CAN interface, and generally provides a simple interface to interact with the simulator.

**Step 1. Clone repository with submodules**

```bash
git clone https://github.com/ZilantRobotics/innopolis_vtol_dynamics.git --recursive
```

Whenever you pull this repository, don't forget to update submodules:

```bash
git submodule update --init --recursive
```

**Step 2. Build/pull the docker image**

To build docker image, type:

```bash
./scripts/sim.py b # build
```

> An image on dockerhub usually is not up to date, so it's better to build manually

**Step 3. Connect everything together for HITL**

> You should skip this step if you want to run PX4 MAVLink SITL mode. Please follow [docs/px4/mavlink](docs/px4/mavlink.md) for details.

Typically we use [CUAV v5+](https://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html) and [RL-programmer-sniffer](https://docs.raccoonlab.co/guide/programmer_sniffer/), but it might be anything else.

An example of a connection is shown in the picture below.

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/sniffer_connection.png" alt="drawing"  width="640"/>

All default parameters expect that you use CAN1 on the autopilot side.

**Step 4. Run the container in force mode**

In `--force` mode the script automatically upload the required firmware and parameters corresponded
to the specified mode, create SLCAN and run the container with required docker flags.

To run force mode you need to install [autopilot-tools](https://pypi.org/project/autopilot-tools/) python package: `pip install autopilot-tools`.

To get the list of all supported modes, just type:

```bash
./scripts/sim.py --help
```

To run PX4 Cyphal quadcopter, type:

```bash
./scripts/sim.py cq  # cq = px4_v1_15_0_cyphal_quadcopter
```

To run PX4 Dronecan VTOL, type:

```bash
./scripts/sim.py dv  # cq = dronecan_vtol
```

Troubleshooting:
- If your sniffer connection is not found or something else is missing, it will exit in a few seconds.

If something doesn't work, please open an issue.

**Step 5. Run ground control station**

Here 2 options are suggested.
1. You can run QGroundControl or MissionPlanner to have manual flight
2. (soon) You can run a script to run one of the test scenario in automatic mode.

**Step 6. (optional) 3D Simulator**

> A new 3D simulator will appear here soon.

## 3. Supported modes

You can obrain the actual list of the suported modes by typing `./scripts/sim.py --help`.

Well, here is the output of the command:

```bash
Primary supported modes (with aliases):
  px4_v1_15_0_cyphal_quadcopter,cq       | Cyphal     PX4 v1.15-beta  Quadrotor x (4001)
  px4_v1_15_0_cyphal_quadplane_vtol,csv  | Cyphal     PX4 v1.15-beta  Standard VTOL (13000)
  px4_v1_15_0_dronecan_quadrotor,dq      | DroneCAN   PX4 v1.15-beta  Quadrotor (4001)
  px4_v1_15_0_dronecan_quadplane_vtol,dv | DroneCAN   PX4 v1.15-beta  Standard VTOL (13000)
  px4_v1_13_0_dronecan_vtol,dv1130       | DroneCAN   PX4 v1.13.0     vtol 13070

Other modes:
  px4_v1_12_0_mavlink_quadplane_vtol    | MAVLink     PX4 v1.12       vtol 13070
  px4_v1_12_0_mavlink_quadcopter        | MAVLink     PX4 v1.12       Quadrotor (4001)
  cyphal_and_dronecan                   | 2 CAN       AP  v4.4.0      Copter
  px4_v1_15_0_cyphal_octorotor,co       | Cyphal      PX4 v1.15-beta  Octorotor Coaxial (12001)

```

New modes will be extended step by step.

## 4. Design

VTOL HITL Dynamics Simulator is designed to be modular. It is divided into the following main components:

1. `UAV dynamics` is the main node that handles actuator commands from the communicator, performs dynamics simulation, and publishes vehicle and sensors states.
2. `Communicator` is the set of nodes that communicate with the `PX4 flight stack` in HITL (via Cyphal/DroneCAN) and SITL (via MAVLink) modes.
3. `inno_sim_interface` is a bridge for interaction with `3D-Simulator` through ROS.

The design of the simulator is shown below.

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/scheme.png" alt="drawing"/>

## 5. Example

Check the video below.

[![Cyphal/DroneCAN HITL VTOL dynamics simulator](https://img.youtube.com/vi/e9MREW6tCmE/0.jpg)](https://youtu.be/e9MREW6tCmE)

## 6. Auxilliary documentation

Docs:

- [Developer docs](docs/dev_docs.md)
- [PX4 MAVLink SITL manual configuration instructions](docs/px4/mavlink.md)

Outdated manual instructions:

- [PX4 Cyphal manual configuration instructions](docs/px4/cyphal.md)
- [PX4 DroneCAN manual configuration instructions](docs/px4/dronecan.md)
- [ArduPilot manual configuration instructions](docs/ardupilot/README.md)

## 7. Changelog notes

| Version | ReleaseDate | Major changes |
| ------- | ----------- | ------------- |
| v0.9.0 | in progress  | Add fmu-v6c and fmu-v6x support beside fmu-v5 |
| v0.8.0 | Jun 10, 2024 | Update PX4 from v1.14 to v1.15 |
| v0.7.0 | Oct 31, 2023 | Update PX4 from v1.13 to v1.14 |
| v0.6.0 | Jul 16, 2023 | Add Octorotor dynamics, fault scenarios and Cyphal ESC feedback |
| v0.5.0 | May 17, 2023 | Add Cyphal PX4 v1.13.0 quadcopter, update DroneCAN PX4 from v1.12.1 to v1.13.0 |
| v0.4.0 | May 16, 2022 | Add Cyphal/DroneCAN custom version of Ardupilot |
| v0.3.0 | Aug 25, 2021 | Add Docker |
| v0.2.0 | Aug 17, 2021 | Update to public DroneCAN PX4 v1.12.1 |
| v0.1.0 | Mar 18, 2021 | First public release for private custom version of DroneCAN PX4 v1.11.2, only CUAV V5+, SITL and HITL modes |
