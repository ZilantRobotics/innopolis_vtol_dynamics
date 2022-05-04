# Innopolis VTOL dynamics simulator

Innopolis VTOL dynamics simulator is a set of ROS packages suggested full simulation for UAV based on PX4.

![dynamics](img/dynamics.png?raw=true "dynamics")

Typical PX4 simulations ways are [SITL and HITL](https://docs.px4.io/master/en/simulation/):
- SITL (software in the loop) allows you to run simulation and flight stack fully on your computer, it doesn't cover hardware related modules,
- HITL (hardware in the loop) allows to run flight stack on real device, but it still doesn't cover actuator and sensor related modules because autopilot works in special mode and uses special MAVLink HITL messages instead of real actuators and sensor drivers.

The key feature of this simulation is to run it in such way that the hardware knows nothing about simulation. This could be possible using uavcan. For users, expecialy those using the uavcan network for uav, it can be very usefull, because it covers more PX4 modules than standard SITL and HITL.

So, Inno VTOL dynamics simulation allows to run simulation in both `SITL` and `uavcan HITL` mode. It also allows to run InnopolisSimulator (left part of the first figure) and visualize forces and moments in RVIZ (right part of the first figure).

## Content
  - [1. Design](#1-design)
  - [2. Installation and building](#2-installation-and-building)
    - [2.1. Inno Dynamics](#21-inno-dynamics)
    - [2.2. PX4 Autopilot](#22-px4-autopilot)
    - [2.3. (optional) InnoSimulator ](#23-optional-innosimulator)
  - [3. Usage example](#3-usage-example)
    - [3.1. Autopilot setup](#31-autopilot-setup)
    - [3.2. Running the simulator](#32-running-the-simulator)
    - [3.3. Loading parameters into a vehicle](#33-loading-parameters-into-a-vehicle)
    - [3.4. InnoSimulator](#34-innosimulator)
    - [3.5. Example](#35-example)
  - [4. Repos used as references](#4-repos-used-as-references)
  - [5. Tests](#5-tests)

## 1. Design

Innopolis VTOL dynamics simulator is designed to support several dynamics and both `MAVLink SITL` and `UAVCAN HITL` modes. As an example, beside `inno_vtol` dynamics [flightgoggles_multicopter](https://github.com/mit-aera/FlightGoggles) dynamics was integrated into simulator as well. You also may use different airframes based on your version of PX4-Autopilot.

At that moment following combination of dynamics and airframes are supported:

| № | dynamics | HITL airframe | SITL airframe (vehicle) |
| - | -------- | -------- | ------------------ |
| 1 | inno_vtol | inno_standard_vtol [(13070)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d/airframes/13070_innopolis_vtol) | innopolis_vtol [(1050)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d-posix/airframes/1050_innopolis_vtol)    |
| 2 | [flightgoggles_multicopter](https://github.com/mit-aera/FlightGoggles)    | iris [(10016)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d/airframes/10016_3dr_iris)               | iris [(10016)](https://github.com/PonomarevDA/Firmware/blob/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d-posix/airframes/10016_iris) |

The simulator is divided into following main components:

1. `UAV dynamics` is the main node that handles actuator commands from communicator, performs dynamics simulation and publishes vehicle and sensors states.
2. `Communicator` is the set of nodes which communicate with `PX4 flight stack` in HITL (via UAVCAN) and SITL (via MAVLink) modes.
3. `inno_sim_interface` is a bridge for interaction with `Inno Simulator` through ROS.

The design of the simulator is shown below.

![scheme](img/scheme.png?raw=true "scheme")

To communicate with the flight stack via communicator `UAV dynamics` node subscribes on following topics:

| № | UAVCAN->ROS topics         | msg                                   |
| - | -------------------------- | ------------------------------------- |
| 1 | /uav/actuators             | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)                             |
| 2 | /uav/actuators             | [std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)                          |

and publishes to following topics:

| № | ROS->UAVCAN topics         | msg                                   |
| - | -------------------------- | ------------------------------------- |
| 1 | /uav/static_temperature    | uavcan_msgs/StaticTemperature         |
| 2 | /uav/static_pressure       | uavcan_msgs/StaticPressure            |
| 3 | /uav/raw_air_data          | uavcan_msgs/RawAirData                |
| 4 | /uav/gps_position          | uavcan_msgs/Fix                       |
| 5 | /uav/imu                   | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)                      |
| 6 | /uav/mag                   | [sensor_msgs/MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html)  |
| 7 | /uav/esc_status            | uavcan_msgs/EscStatus                 |
| 8 | /uav/ice_status            | uavcan_msgs/IceReciprocatingStatus                       |
| 9 | /uav/fuel_tank_status      | uavcan_msgs/IceFuelTankStatus             |
| 10| /uav/battery_status        | [sensor_msgs/BatteryState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html)    |

Here topics 1-6 are necessary for any simulation. Last 4 topics are auxilliary and you may enable/disable them in [sim_params.yaml](uav_dynamics/inno_vtol_dynamics/config/sim_params.yaml) config file. You may implement your own sensors in [sensors.cpp](uav_dynamics/inno_vtol_dynamics/src/sensors.cpp) file.

To work in pair with [InnoSimulator](https://github.com/inno-robolab/InnoSimulator) as physics engine via [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface) it publishes and subscribes on following topics.

| № | topic             | msg                             |
| - | ----------------- | ------------------------------- |
| 1 | /uav/actuators    | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)                 |
| 2 | /uav/gps_position | uavcan_msgs/Fix                 |
| 3 | /uav/attitude     | geometry_msgs/QuaternionStamped |


## 2. Installation and building

First thing you need to do is to decide in which mode you are going to use the simulator.

Typically, you can run the simulator in:
- either HITL mode (using real hardware and UAVCAN sniffer) or SITL (by running px4 flight stack on the computer),
- and either with docker (the easiest way to start playing with the simulator), or by building and installing the simulator manually (might be preffered if you are going to contribute something).

Then follow the instruction below. The steps you need to take will depend on the selected mode.

### 2.1. Inno Dynamics

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


### 2.2. PX4 Autopilot

Anyway, to start the simulator at least initially, you need to install PX4 Autopilot Firmware.

Docker image of this repository doesn't include PX4 Autopilot to make the work more flexible and image ligher.

- In HITL mode the PX4 Autopilot is necessary for uploading the firmware to the hardware.
- In SITL mode the PX4 Autopilot is required any time when you run the simulator.

Most probably, you need exactly [the following version of PX4 Autopilot](https://github.com/InnopolisAero/PX4-Autopilot/tree/px4_v1.12.1_inno_vtol_dynamics). It has 2 differences compared to the master branch:
- Firstly, it has [the ability to disable board sensors](https://github.com/PX4/PX4-Autopilot/pull/18550). This feature is higly necessary for working in HITL mode.
- Secondly, it has `inno_vtol` custom airframe.

So, the only way when you can use the master or any other branch based on the original software, is SITL simulator with `flight_goggles` (MR) dynamics. It is based on default `iris` airframe.

**Step 1.** Installation

For installation use the official PX4 instruction, but the custom firmware version [InnopolisAero/PX4-Autopilot](https://github.com/InnopolisAero/PX4-Autopilot/tree/px4_v1.12.1_inno_vtol_dynamics) (or the original one, if you are going to use the simuator in SITL for `iris` airframe only as mentioned above).

**Step 2.** Build

To build either in SITL or in TRUE HITL mode run:

```
cd PX4-Autopilot
git checkout px4_v1.12.1_inno_vtol_dynamics
git submodule update --init --recursive && make clean && make distclean
```

Then for SITL mode:

```bash
DONT_RUN=1 make px4_sitl gazebo_standard_vtol
```
or

```bash
DONT_RUN=1 make px4_sitl gazebo_iris
```

And for HITL mode for Cuav V5+ type line below. If you use another hardware, read the [PX4 doc](https://dev.px4.io/master/en/setup/building_px4.html) for details.

```
make px4_fmu-v5_default upload
```

If you want to use SITL, don't forget to add these lines to your `.bashrc` file, don't forget to change `~/PX4-Autopilot` to your actual Firmware path

```
source ~/catkin_ws/devel/setup.bash    # (optional)
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo
```

### 2.3. (optional) InnoSimulator

InnoSimulator is a photorealistic simulator.

At that moment we use it for visualization purpose only.

To use it you need to download it from [inno-robolab/InnoSimulator](https://github.com/inno-robolab/InnoSimulator) repository.


## 3. Usage example

### 3.1. Autopilot setup

This step is higtly different for HITL and SITL modes.

**1. SITL mode** requires running PX4 flight stack on your PC.

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

**2. HITL mode** requires physical connection of your autopilot and PC via CAN/UART sniffer.

The example of connection shown on the picture below.

![sniffer_connection](img/sniffer_connection.png?raw=true "sniffer_connection")

Typically we use [cuav v5+](https://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html) and [inno-programmer-sniffer](https://github.com/InnopolisAero/inno_uavcan_node_binaries/blob/master/doc/programmer_sniffer/README.md), but it might be anything else.

### 3.2. Running the simulator

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


### 3.3. Loading parameters into a vehicle

This step is the same for all modes. Each vehicle especially need to update his default parameters. Especially for HITL it is necessary to disable board sensors and disable few pre-flight checks.

- Run QGC and load correposponded [params](uav_dynamics/inno_vtol_dynamics/config/) into your vehicle,
- Restart your vehicle and QGC,
- Note: if your QGC is turned off right after the loading of parameters, you need to run QGC again and perform device rebooting manually,
- Note: sometimes from the first attempt params are not loaded correctly, so it's better to load them twice. Usually it happens in HITL mode.
 
![usage_load_params](img/usage_load_params.png?raw=true "usage_load_params")

After restarting check correspondences of your airframe and that vehicle is `Ready To Fly`

![usage_check_airframe](img/usage_check_airframe.png?raw=true "usage_check_airframe")

### 3.4. InnoSimulator

If you set parameter `run_inno_sim_bridge:=true` or leave it by default, you will only need to type something like that:

```bash
~/software/InnoSimulator-Linux64-2021.1.3/InnoSimulator.x86_64
```

Then choose a drone and press Launch button.

### 3.5 Example

Check the video below.

[![uavcan vtol dynamics simulator](https://img.youtube.com/vi/e9MREW6tCmE/0.jpg)](https://youtu.be/e9MREW6tCmE)

## 4. Repos used as references

1. [flightgoggles_uav_dynamics (multicopter)](https://github.com/mit-fast/FlightGoggles/blob/master/flightgoggles_uav_dynamics/) - read their [paper](https://arxiv.org/pdf/1905.11377.pdf)
2. [PX4 mavlink communicator](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge)
3. [sitl_gazebo](https://github.com/PX4/sitl_gazebo)
4. [innopolis_vtol_indi](https://github.com/InnopolisAero/innopolis_vtol_indi) - dynamics written in Octave
5. [InnoSimulator](https://github.com/inno-robolab/InnoSimulator) - photorealistic simulator
6. [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface) - bridge between dynamics and photorealistic simulator

## 5. Tests
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
