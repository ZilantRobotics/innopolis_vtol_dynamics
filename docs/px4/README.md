# PX4 Dronecan HITL simulation configuration



## Content
  - [Requirements](#requirements)
  - [Introduction](#introduction)
  - [1. Installation](#1-installation)
  - [2. Build and load firmware](#2-build-and-load-firmware)
  - [3. Load parameters](#3-load-parameters)
  - [4. Run the simulation](#4-run-the-simulation)
  - [5. Ready to fly](#5-ready-to-fly)

## Requirements

You need following hardware:

- Any autopilot that supports PX4 flight stack,
- CAN-sniffer (for example [inno-programmer-sniffer](https://github.com/InnopolisAero/inno_uavcan_node_binaries/blob/master/doc/programmer_sniffer/README.md)).

## Introduction

Anyway, to start the simulator at least initially, you need to install PX4 Autopilot Firmware.

Docker image of this repository doesn't include PX4 Autopilot to make the work more flexible and image lighter.

- In DroneCAN HITL mode the PX4 Autopilot is necessary for uploading the firmware to the hardware.
- In MAVLink SITL mode the PX4 Autopilot software is required any time when you run the simulator.

Most probably, you need exactly [the following version of PX4 Autopilot](https://github.com/InnopolisAero/PX4-Autopilot/tree/px4_v1.12.1_inno_vtol_dynamics). It has 2 differences compared to the master branch:
- Firstly, it has [the ability to disable board sensors](https://github.com/PX4/PX4-Autopilot/pull/18550). This feature is highly necessary for working in HITL mode.
- Secondly, it has `inno_vtol` custom airframe.

So, the only way when you can use the master or any other branch based on the original software, is SITL simulator with `flight_goggles` (MR) dynamics. It is based on default `iris` airframe.

## 1. Installation

For installation use the official PX4 instruction, but the custom firmware version [InnopolisAero/PX4-Autopilot](https://github.com/InnopolisAero/PX4-Autopilot/tree/px4_v1.12.1_inno_vtol_dynamics) (or the original one, if you are going to use the simulator in SITL for `iris` airframe only as mentioned above).

## 2. Build and load

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

## 3. Load parameters

This step is the same for MAVLink SITL and DroneCAN HITL modes. Each vehicle especially need to update his default parameters. Especially for HITL it is necessary to disable board sensors and disable few pre-flight checks.

- Run QGC and load corresponded [params](uav_dynamics/inno_vtol_dynamics/config/) into your vehicle,
- Restart your vehicle and QGC,
- Note: if your QGC is turned off right after the loading of parameters, you need to run QGC again and perform device rebooting manually,
- Note: sometimes from the first attempt params are not loaded correctly, so it's better to load them twice. Usually it happens in HITL mode.
 
![usage_load_params](usage_load_params.png?raw=true "usage_load_params")

After restarting check correspondences of your airframe and that vehicle is `Ready To Fly`

![usage_check_airframe](usage_check_airframe.png?raw=true "usage_check_airframe")

## 4. Run the simulation

Run the simulation.

## 5. Ready to fly

