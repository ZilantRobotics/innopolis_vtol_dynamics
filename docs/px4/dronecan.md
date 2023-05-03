# PX4 DroneCAN HITL simulation configuration

## Hardware Requirements

For HITL simulation you need the following hardware:

- Any autopilot that supports PX4 flight stack (tested on [cuav v5+](https://docs.px4.io/main/en/flight_controller/cuav_v5_plus.html))
- CAN-sniffer (for example [RaccoonLab sniffer](https://raccoonlabdev.github.io/docs/guide/programmer_sniffer/)).

An example of connection is shown below:

<img src="../img/sniffer_connection.png" alt="drawing" width="640"/>

The autopilot and sniffer should be connected via any CAN bus.

## Software requirements

PX4 doesn't fully support the DroneCAN UAV HITL simulation right now. The following features are required:
- [the ability to disable board sensors](https://github.com/PX4/PX4-Autopilot/pull/18550)
- `inno_vtol` custom airframe (for VTOL simulation only).

It is recommended to use one of the following custom branches:
- [v1.12.1 + DroneCAN HITL for QuadCopter and VTOL](https://github.com/RaccoonlabDev/PX4-Autopilot/tree/px4_v1.12.1_inno_vtol_dynamics),
- [main branch + DroneCAN HITL for QuadCopter](https://github.com/RaccoonlabDev/PX4-Autopilot/tree/pr-disable-board-sensors)


## Step 1. Build and load

It is expected that you already familiar with PX4, have everything installed according to the official PX4 instruction.

To build and upload the firmware in HITL mode run:

```
cd PX4-Autopilot
git checkout <branch_with_dronecan_hitl>
git submodule update --init --recursive
make clean
make distclean
make px4_fmu-v5_default upload
```

## Step 2. Load parameters

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

## Step 3. Verify with gui_tool

Run `gui_tool` and check messages.

## Step 4. Ready to fly

After restarting check correspondences of your airframe.

If the simulation is connected, the vehicle should be `Ready To Fly`.

![usage_check_airframe](usage_check_airframe.png?raw=true "usage_check_airframe")
