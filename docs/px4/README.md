# PX4 Dronecan HITL / MAVLink SITL simulation configuration

This page explains how to configure PX4 to work with DroneCAN HITL / MAVLink SITL simulator. It doesn't cover how to run the simulator itself.

## Requirements

You need the following hardware:

- Any autopilot that supports PX4 flight stack,
- CAN-sniffer (for example [inno-programmer-sniffer](https://github.com/InnopolisAero/inno_uavcan_node_binaries/blob/master/doc/programmer_sniffer/README.md)).

The autopilot and sniffer should be connected via any CAN bus.

## Introduction

The simulator may work in 2 modes. Your steps will depends on which mode are you going to use.

- In DroneCAN HITL mode the PX4 Autopilot is necessary for uploading the specific firmware to the hardware.
- In MAVLink SITL mode the PX4 Autopilot software is required any time when you run the simulator.

Most probably, you need exactly [the following version of PX4 Autopilot](https://github.com/InnopolisAero/PX4-Autopilot/tree/px4_v1.12.1_inno_vtol_dynamics). It has 2 differences compared to the master branch:
- Firstly, it has [the ability to disable board sensors](https://github.com/PX4/PX4-Autopilot/pull/18550). This feature is required for working in DroneCAN HITL mode.
- Secondly, it has `inno_vtol` custom airframe.

So, the only way when you can use the master or any other branch based on the original software, is MAVLink SITL simulator with `flight_goggles` (MR) dynamics. It is based on the default `iris` airframe.

## 1. Build and load

It is expected that you already familiar with PX4, have everything installed according to the official PX4 instruction. The only difference is that you rather need the custom software version [InnopolisAero/PX4-Autopilot](https://github.com/InnopolisAero/PX4-Autopilot/tree/px4_v1.12.1_inno_vtol_dynamics) (or the original one, if you are going to use the simulator in MAVLink SITL for `iris` airframe only as mentioned above).

To build either in MAVLink SITL or in DroneCAN HITL mode run:

```
cd PX4-Autopilot
git checkout px4_v1.12.1_inno_vtol_dynamics
git submodule update --init --recursive && make clean && make distclean
```

Then for MAVLink SITL mode:

```bash
DONT_RUN=1 make px4_sitl gazebo_standard_vtol
```
or

```bash
DONT_RUN=1 make px4_sitl gazebo_iris
```

And for DroneCAN HITL mode for Cuav V5+ type line below. If you use another hardware, read the [PX4 doc](https://dev.px4.io/master/en/setup/building_px4.html) for details.

```
make px4_fmu-v5_default upload
```

If you want to use MAVLink SITL, don't forget to add these lines to your `.bashrc` file, don't forget to change `~/PX4-Autopilot` to your actual Firmware path

```
source ~/catkin_ws/devel/setup.bash    # (optional)
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo
```

## 2. Load parameters

When the firmware is uploaded, you need to properly configure your parameters.

You need to:
- (for HITL) disable few preflight checks related to the USB connection and the battery
- (for HITL) disable all onboard sensors (it is possible only on custom firmware version)
- configure control,
- and other things.

The easiest way is to simply load the parameters from the file.

1. Run QGC, load the corresponded [params](../../params/) into your vehicle and accept all changes
2. If your QGC application terminated unexpectedly, just run it again
3. Restart your vehicle via QGC

You need to load the parameters and reboot the vehicle twice!
 
![usage_load_params](usage_load_params.png?raw=true "usage_load_params")

After second parameters load, the vehicle should be completely configured.

## 3. Ready to fly

After restarting check correspondences of your airframe.

If the simulation is connected, the vehicle should be `Ready To Fly`.

![usage_check_airframe](usage_check_airframe.png?raw=true "usage_check_airframe")

```
Note. It is better to initially connect the hardware with the simulator via sniffer, run the simulator and only then power/reboot the vehicle. If the vehicle starts with no input data at the beginning it may not allow to fly.
```
