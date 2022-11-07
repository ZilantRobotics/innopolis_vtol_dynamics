# PX4 Cyphal HITL simulation configuration

## Hardware Requirements

For HITL simulation you need the following hardware:

- Any autopilot that supports PX4 flight stack (tested on [cuav v5+](https://docs.px4.io/main/en/flight_controller/cuav_v5_plus.html))
- CAN-sniffer (for example [RaccoonLab sniffer](https://raccoonlabdev.github.io/docs/guide/programmer_sniffer/)).

An example of connection is shown below:

<img src="https://github.com/RaccoonlabDev/innopolis_vtol_dynamics/blob/master/docs/img/sniffer_connection.png?raw=true" alt="drawing" width="640"/>

The autopilot and sniffer should be connected via any CAN bus.

## Software requirements

PX4 doesn't fully support the Cyphal UAV HITL simulation right now. The following features are required:
- [the ability to disable board sensors](https://github.com/PX4/PX4-Autopilot/pull/18550)
- `inno_vtol` custom airframe (for VTOL simulation only),
- [few Cyphal related fixes](https://github.com/PX4/PX4-Autopilot/pull/20671),
- [Cyphal gps, mag, baro and other sensors implementation](https://github.com/PonomarevDA/PX4-Autopilot/tree/pr-cyphal).

It is recommended to use the custom branch: [pr-cyphal](https://github.com/PonomarevDA/PX4-Autopilot/tree/pr-cyphal).

## Step 1. Build and load the firmware

It is expected that you already familiar with PX4, have everything installed according to the official PX4 instruction.

To build and upload the firmware in HITL mode run:

```
cd PX4-Autopilot
git checkout pr-cyphal
git submodule update --init --recursive
make clean
make distclean
make px4_fmu-v5_cyphal upload
```

## Step 2. Load the parameters

An autopilot for HITL mode configuration requires several steps such as:
- disable few preflight checks,
- disable onboard sensors,
- configure Cyphal related part including registers,
- calibrate sensors.

The simplest way is to just call a python script based on [pymavlink](https://github.com/ArduPilot/pymavlink) that handles everything.

> Warning! The script below will reset all your autopilot parameters. If you need them, don't forget to save.

```bash
cd scripts/setup
./px4_cyphal.py
```

## Step 3. Verify with Yukon

<img src="../ardupilot/yukon.png" alt="drawing" width="640"/>

## Step 4. Ready to fly

Open QGroundControl. If the simulation is connected, the vehicle should be `Ready To Fly`. You should see:

![usage_check_airframe](usage_check_airframe.png?raw=true "usage_check_airframe")

