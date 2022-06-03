# Innopolis VTOL dynamics simulator

Innopolis VTOL dynamics simulator is a set of ROS packages suggested full simulation for UAV based on PX4/Ardupilot autopilot.

![dynamics](docs/img/dynamics.png?raw=true "dynamics")

Typical PX4 simulations ways are [SITL and HITL](https://docs.px4.io/master/en/simulation/):
- SITL (software in the loop) allows you to run simulation and flight stack fully on your computer, it doesn't cover hardware related modules,
- HITL (hardware in the loop) allows to run flight stack on real device, but it still doesn't cover actuator and sensor related modules because autopilot works in special mode and uses special MAVLink HITL messages instead of real actuators and sensor drivers.

The key feature of this simulation is to run it in such way that the hardware knows nothing about simulation. This could be possible using uavcan. For users, expecialy those using the uavcan network for uav, it can be very usefull, because it covers more PX4 modules than standard SITL and HITL.

So, Inno VTOL dynamics simulation allows to run simulation in both `SITL` and `uavcan HITL` mode. It also allows to run InnopolisSimulator (left part of the first figure) and visualize forces and moments in RVIZ (right part of the first figure).

## Content
  - [1. Design](#1-design)
  - [2. Supported modes](#2-supported-modes)
  - [3. Installation, building and configuration](#3-installation-building-and-configuration)
    - [3.1. Inno Dynamics](#31-inno-dynamics)
    - [3.2. Autopilot configuration](#32-autopilot-configuration)
    - [3.3. (optional) InnoSimulator ](#33-optional-innosimulator)
  - [4. Usage example](#4-usage-example)
    - [4.1. Autopilot setup](#41-autopilot-setup)
    - [4.2. Running the simulator](#42-running-the-simulator)
    - [4.3. Loading parameters into a vehicle](#43-loading-parameters-into-a-vehicle)
    - [4.4. InnoSimulator](#44-innosimulator)
    - [4.5. Example](#45-example)
  - [5. Repos used as references](#5-repos-used-as-references)
  - [6. Tests](#6-tests)

## 1. Design

Innopolis VTOL dynamics simulator is designed to support several dynamics and protocols `MAVLink SITL` and `DroneCAN HITL`, `Cyphal HITL` modes. As an example, beside `inno_vtol` dynamics [flightgoggles_multicopter](https://github.com/mit-aera/FlightGoggles) dynamics was integrated into simulator as well. It is expected to use either PX4 or Ardupilot autopilots.

The simulator is divided into following main components:

1. `UAV dynamics` is the main node that handles actuator commands from communicator, performs dynamics simulation and publishes vehicle and sensors states.
2. `Communicator` is the set of nodes which communicate with `PX4 flight stack` in HITL (via UAVCAN) and SITL (via MAVLink) modes.
3. `inno_sim_interface` is a bridge for interaction with `Inno Simulator` through ROS.

The design of the simulator is shown below.

![scheme](docs/img/scheme.png?raw=true "scheme")

To communicate with the flight stack via communicator `UAV dynamics` node subscribes on following topics:

| № | Subscribed topics         | msg                                   |
| - | -------------------------- | ------------------------------------- |
| 1 | /uav/actuators             | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)                             |
| 2 | /uav/actuators             | [std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)                          |

Minimal sensors publishers list of topics:

| № | Advertised topics         | msg                                   |
| - | -------------------------- | ------------------------------------- |
| 1 | /uav/static_temperature    | uavcan_msgs/StaticTemperature         |
| 2 | /uav/static_pressure       | uavcan_msgs/StaticPressure            |
| 3 | /uav/raw_air_data          | uavcan_msgs/RawAirData                |
| 4 | /uav/gps_position          | uavcan_msgs/Fix                       |
| 5 | /uav/imu                   | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)                      |
| 6 | /uav/mag                   | [sensor_msgs/MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html)  |

Extended sensors publishers list of topics:

| № | Advertised topics         | msg                                   |
| - | -------------------------- | ------------------------------------- |
| 7 | /uav/esc_status            | uavcan_msgs/EscStatus                 |
| 8 | /uav/ice_status            | uavcan_msgs/IceReciprocatingStatus                       |
| 9 | /uav/fuel_tank_status      | uavcan_msgs/IceFuelTankStatus             |
| 10| /uav/battery_status        | [sensor_msgs/BatteryState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html)    |

Here topics 1-6 are necessary for any simulation. Last 4 topics are auxilliary and you may enable/disable them in [sim_params.yaml](uav_dynamics/inno_vtol_dynamics/config/sim_params.yaml) config file. You may implement your own sensors in [sensors.cpp](uav_dynamics/inno_vtol_dynamics/src/sensors.cpp) file.

To work in pair with [InnoSimulator](https://github.com/inno-robolab/InnoSimulator) as physics engine via [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface) it publishes and subscribes on following topics.

| № | Advertised topics | msg                             |
| - | ----------------- | ------------------------------- |
| 1 | /uav/actuators    | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)                 |
| 2 | /uav/gps_position | uavcan_msgs/Fix                 |
| 3 | /uav/attitude     | geometry_msgs/QuaternionStamped |

## 2. Supported modes

At that moment the simulation might be run if following modes:

| № | Autopilot | Communicator | dynamics | Airframe |
| - | -------- | --------- | ------------ | -------- |
| 1 | PX4 | DroneCAN HITL | inno_vtol | inno_standard_vtol [(13070)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d/airframes/13070_innopolis_vtol)
| 2 | PX4 | DroneCAN HITL | [flightgoggles_multicopter](https://github.com/mit-aera/FlightGoggles) | iris [(10016)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d-posix/airframes/10016_iris) |
| 3 | PX4 | MAVLink SITL | inno_vtol | innopolis_vtol [(1050)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d-posix/airframes/1050_innopolis_vtol) |
| 4 | PX4 | MAVLink SITL | [flightgoggles_multicopter](https://github.com/mit-aera/FlightGoggles) | iris [(10016)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d-posix/airframes/10016_iris) |
| 5 | Ardupilot | DroneCAN HITL | inno_vtol | copter |
| 6 | Ardupilot | Cyphal HITL | inno_vtol | copter |

New modes will be extended step by step.

## 3. Installation, building and configuration

First thing you need to do is to decide in which mode you are going to use the simulator.

Typically, you can run the simulator in:
- either HITL mode (using real hardware and CAN sniffer) or SITL (by running px4 flight stack on the computer),
- and either with docker (the easiest way to start playing with the simulator), or by building and installing the simulator manually (might be preffered if you are going to contribute something).

Then follow the instruction below. The steps you need to take will depend on the selected mode.

### 3.1. Inno Dynamics

This repository is the only necessary component to start work with the simulator.

**Step 1.** Firstly, you need to clone the repository with submodules.

```bash
git clone https://github.com/InnopolisAero/innopolis_vtol_dynamics.git --recursive
```

Every time when you pull this repository, don't forget to update submodules:

```bash
git submodules update --init --recursive
```

**Step 2.** Then, you should either build/pull the docker image, or install and build everything manually.

1. With Docker. All work with docker is recommended to do via `./scripts/docker.sh` script. For example, you can build it by typing `./scripts/docker.sh build` or you can pull the image by typing `./scripts/docker.sh pull`. This script is simply configurate the correct image name and run all necessary auxilliary scripts.

2. Without Docker. If you are going to build this repository and install all dependencies manually, please follow the instruction from [Dockerfile](Dockerfile) or from [build workflow](.github/workflows/catkin_build.yml).


### 3.2. Autopilot configuration

Please, refer to the corresponded section for the references:
- [PX4 Autopilot](docs/px4/README.md)
- [Ardupilot](docs/ardupilot/README.md)

### 3.3. (optional) InnoSimulator

InnoSimulator is a photorealistic simulator.

At that moment we use it for visualization purpose only.

To use it you need to download it from [inno-robolab/InnoSimulator](https://github.com/inno-robolab/InnoSimulator) repository.


## 4. Usage example

### 4.1. Autopilot setup

This step is higtly different for HITL and SITL modes.

**1. PX4 MAVLink SITL mode** requires running PX4 flight stack on your PC.

Depending on which airframe you are going to use, you need to run px4.launch file with argument correponded to your airframe.

If you are going to use the simulator for `inno_vtol` dynamics, you may run:

```
roslaunch px4 px4.launch vehicle:=innopolis_vtol
```

If you are going to use the simulator for `flight_goggles` dynamics, you may run:

```
roslaunch px4 px4.launch vehicle:=iris
```

For extended documentation about PX4 SITL flight stack, please check the official PX4 instruction.

**2. DroneCAN/Cyphal HITL modes** requires physical connection of your autopilot and PC via CAN/UART sniffer.

The example of connection shown on the picture below.

![sniffer_connection](docs/img/sniffer_connection.png?raw=true "sniffer_connection")

Typically we use [cuav v5+](https://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html) and [inno-programmer-sniffer](https://github.com/InnopolisAero/inno_uavcan_node_binaries/blob/master/doc/programmer_sniffer/README.md), but it might be anything else.

### 4.2. Running the simulator

It is recommended to play with the simulator with one of 2 scripts depending on your mode:
- use [scripts/docker.sh](scripts/docker.sh) if you build the docker image
- use [scripts/run_sim.sh](scripts/run_sim.sh) if you build and install all necessary dependencies manually.

The usage of these scripts are the same. Moremore, `docker.sh` actually internally call the `run_sim.sh` script. To get extended info about these scripts try them with `help` command, for example: `./scripts/docker.sh help`.

Both HITL and SITL modes requires only to run a single command in the termianl. Below your can see an example how to run HITL mode for inno_vtol dynamics and SITL simulator for flight goggles dynamics.

```
./scripts/docker.sh hitl_inno_vtol
./scripts/run_sim.sh sitl_flight_goggles
```

If you are using SITL mode, don't forget to run PX4 SITL flight stack according to [3.1. Autopilot setup](#31-autopilot-setup).

### 4.4. InnoSimulator

If you set parameter `run_inno_sim_bridge:=true` or leave it by default, you will only need to type something like that:

```bash
~/software/InnoSimulator-Linux64-2021.1.3/InnoSimulator.x86_64
```

Then choose a drone and press Launch button.

### 4.5 Example

Check the video below.

[![uavcan vtol dynamics simulator](https://img.youtube.com/vi/e9MREW6tCmE/0.jpg)](https://youtu.be/e9MREW6tCmE)

## 5. Repos used as references

1. [flightgoggles_uav_dynamics (multicopter)](https://github.com/mit-fast/FlightGoggles/blob/master/flightgoggles_uav_dynamics/) - read their [paper](https://arxiv.org/pdf/1905.11377.pdf)
2. [PX4 mavlink communicator](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge)
3. [sitl_gazebo](https://github.com/PX4/sitl_gazebo)
4. [innopolis_vtol_indi](https://github.com/InnopolisAero/innopolis_vtol_indi) - dynamics written in Octave
5. [InnoSimulator](https://github.com/inno-robolab/InnoSimulator) - photorealistic simulator
6. [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface) - bridge between dynamics and photorealistic simulator

## 6. Tests
We use [GoogleTest](https://github.com/google/googletest/tree/master/googletest).
To install this you should follow official instruction or this sequence:

```
git clone https://github.com/google/googletest.git -b release-1.10.0
cd googletest               # Main directory of the cloned repository.
mkdir build                 # Create a directory to hold the build output.
cd build
cmake .. -DBUILD_GMOCK=OFF  # Generate build scripts for GoogleTest without GoogleMock.
make
sudo make install           # Install in /usr/local/ by default
```

To run test, type (and read [here](http://wiki.ros.org/gtest) and [here](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html#building-and-running-tests) if you don't configured and launched accordingly gtest from ros yet):

```
roscd innopolis_vtol_dynamics
./catkin_test
```
