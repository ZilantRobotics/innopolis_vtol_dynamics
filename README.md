# VTOL HITL Dynamics Simulator

VTOL HITL Dynamics Simulator is a set of ROS packages proposed full simulation for UAV based on PX4/Ardupilot autopilot.

![dynamics](docs/img/dynamics.png?raw=true "dynamics")

The key feature of this simulation is to run it in such a way that the hardware knows nothing about the simulation. This can be done with Cyphal/DroneCAN. It covers more PX4 modules than standard SITL and HITL.

## 1. Design

VTOL HITL Dynamics Simulator is designed to be flexible.

It supports multiple dynamics:
- [inno_vtol](https://github.com/RaccoonlabDev/inno_vtol_dynamics),
- [flightgoggles_multicopter](https://github.com/mit-aera/FlightGoggles)

and protocols:
- `MAVLink SITL`,
- `DroneCAN HITL`,
- `Cyphal HITL`.

It is expected to use either PX4 or ArduPilot autopilots.

The simulator is divided into the following main components:

1. `UAV dynamics` is the main node that handles actuator commands from the communicator, performs dynamics simulation, and publishes vehicle and sensors states.
2. `Communicator` is the set of nodes that communicate with the `PX4 flight stack` in HITL (via Cyphal/DroneCAN) and SITL (via MAVLink) modes.
3. `inno_sim_interface` is a bridge for interaction with `Inno Simulator` through ROS.

The design of the simulator is shown below.

![scheme](docs/img/scheme.png?raw=true "scheme")

## 2. Supported modes

The primaries modes are:

| № | Autopilot | Communicator | dynamics | Airframe |
| - | -------- | --------- | ------------ | -------- |
| 1 | PX4 v1.12/v1.13 | DroneCAN HITL | inno_vtol | inno_standard_vtol [(13070)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d/airframes/13070_innopolis_vtol)
| 2 | PX4 v1.13 | Cyphal HITL | inno_vtol | Generic Quadcopter [(4001)](https://dev.px4.io/master/en/airframes/airframe_reference.html#quadrotor-x)

The following modes are supported as well, but they are not well-tested:

| № | Autopilot | Communicator | dynamics | Airframe |
| - | -------- | --------- | ------------ | -------- |
| 1 | PX4 v1.12| DroneCAN HITL | quadcopter | iris [(10016)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d-posix/airframes/10016_iris) |
| 2 | PX4 v1.12 | MAVLink SITL | inno_vtol | innopolis_vtol [(1050)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d-posix/airframes/1050_innopolis_vtol) |
| 3 | PX4 v1.12 | MAVLink SITL | quadcopter | iris [(10016)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d-posix/airframes/10016_iris) |
| 4 | ArduPilot v4.3/v4.4 | DroneCAN HITL | inno_vtol | copter |
| 5 | ArduPilot v4.3/v4.4 | Cyphal HITL | inno_vtol | copter |

New modes will be extended step by step.

## 3. Usage

The instruction below discribes the most common use case: PX4 v1.13 in DroneCAN VTOL and Cyphal Quadcopter modes. If you want to use ArduPilot, another dynamic, airframe or SITL mode, you can still use this instruction, but it is recommended to read additional instructions.

The simulator is distributed as a Docker image. If you want to contribute or try it without Docker, you can find more details in [the developer docs](docs/dev_docs.md).

**Step 1. Clone repository with submodules**

```bash
git clone https://github.com/RaccoonlabDev/innopolis_vtol_dynamics.git --recursive
```

Whenever you pull this repository, don't forget to update submodules:

```bash
git submodule update --init --recursive
```

**Step 2. Build/pull the docker image**

It is recommended to use the `./scripts/docker.sh` script. It configures all the necessary Docker flags, SLCAN and provides  a simple interface to interact with the simulator.

To get help, just type:

```bash
./scripts/docker.sh --help
```

To build docker image, type:

```bash
./scripts/docker.sh build
```

To pull docker image, type:

```bash
./scripts/docker.sh pull
```

**Step 3. Connect everything together**

> This step is not necessary if you want to run PX4 MAVLink SITL mode. Please follow [docs/px4/mavlink](docs/px4/mavlink.md) instead.

Typically we use [CUAV v5+](https://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html) and [RL-programmer-sniffer](https://docs.raccoonlab.co/guide/programmer_sniffer/), but it might be anything else.

An example of a connection is shown in the picture below.

<img src="docs/img/sniffer_connection.png" alt="drawing" width="640"/>

If you have a choice, it is sometimes preferable to use CAN1 on the autopilot side.

**Step 4. Upload firmware**

HITL simulator may require some additional features that the official PX4 and ArduPilot doesn't have.

The appropriated and ready to use binaries are uploaded here: [PX4](https://github.com/ZilantRobotics/PX4-Autopilot/releases) and [ArduPilot](https://github.com/ZilantRobotics/ardupilot/releases). Alternativelly, you can build the firmware manually.

For PX4 DroneCAN you can upload the firmware by typing the following 2 lines:

```bash
wget https://github.com/ZilantRobotics/PX4-Autopilot/releases/download/v1.13.0_hitl/px4_fmu-v5_default.px4
./scripts/px4/upload.sh px4_fmu-v5_default.px4
```

For PX4 Cyphal you can upload the firmware by typing:

```bash
wget https://github.com/ZilantRobotics/PX4-Autopilot/releases/download/v1.13.0_hitl/px4_fmu-v5_cyphal.px4
./scripts/px4/upload.sh px4_fmu-v5_cyphal.px4
```

**Step 5. Configure autopilot**

You need to run the [scripts/autopilot_configure.py](scripts/autopilot_configure.py). It will automatically will reset your parameters to default, and then configure exactly the parameters your autopilot needs for the HITL simulation. It will automatically restart the autopilot a few times.

```bash
pip install mavlink_tools
./scripts/autopilot_configure.py <desired_mode>
```

If your desired mode is not supported, refer to the corresponded section for the details:
- [PX4 Autopilot](docs/px4/README.md)
- [Ardupilot](docs/ardupilot/README.md)

**Step 6. Run the container**

You can run the simulator with the same `./scripts/docker.sh` script.

To get help, type:

```bash
./scripts/docker.sh --help
```

To run Cyphal VTOL, type:

```bash
./scripts/docker.sh cv
```

To run Dronecan VTOL, type:

```bash
./scripts/docker.sh dv
```

Here the commands are similar for both PX4 and ArduPilot.

The scripts will automatically attach SLCAN. If your sniffer connection is not found, it will exit in a few seconds.

**Step 7. Run QGC**

Run QGroundControl or any other MAVLink based ground station you need to start the flight.

**Step 8. (optional) InnoSimulator**

InnoSimulator is a photorealistic simulator.

At that moment we use it for visualization purposes only.

To use it you need to download it from [inno-robolab/InnoSimulator](https://github.com/inno-robolab/InnoSimulator) repository.

An example of running might be:

```bash
~/software/InnoSimulator-Linux64-2021.1.3/InnoSimulator.x86_64
```

Then choose a drone and press the Launch button.


## 4. Example

Check the video below.

[![Cyphal/DroneCAN HITL VTOL dynamics simulator](https://img.youtube.com/vi/e9MREW6tCmE/0.jpg)](https://youtu.be/e9MREW6tCmE)

## 5. Repos used as references

1. [flightgoggles_uav_dynamics (multicopter)](https://github.com/mit-fast/FlightGoggles/blob/master/flightgoggles_uav_dynamics/) - read their [paper](https://arxiv.org/pdf/1905.11377.pdf)
2. [PX4 mavlink communicator](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge)
3. [sitl_gazebo](https://github.com/PX4/sitl_gazebo)
4. [innopolis_vtol_indi](https://github.com/RaccoonlabDev/innopolis_vtol_indi) - dynamics written in Octave
5. [InnoSimulator](https://github.com/inno-robolab/InnoSimulator) - photorealistic simulator
6. [inno_sim_interface](https://github.com/RaccoonlabDev/inno_sim_interface) - the bridge between dynamics and photorealistic simulator
