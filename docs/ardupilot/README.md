# Ardupilot Cyphal HITL simulation configuration

This page explains how to configurate Ardupilot to work with Cyphal HITL simulator. It doesn't cover how to run the simulator.

## Requirements

You need the following hardware:

- CUAV V5+ autopilot or similar fmuv5,
- CAN-sniffer (for example [inno-programmer-sniffer](https://github.com/InnopolisAero/inno_uavcan_node_binaries/blob/master/doc/programmer_sniffer/README.md)).

The autopilot and sniffer should be connected via CAN 1 bus (second bus is not supported at this moment).

## 1. Load firmware

First of all you need to load the firmware with few additional modification relative to the master branch.

It is expected you already have downloaded repository and everything is installed according to the official manuals.

Checkout on [Cyphal HITL branch](https://github.com/PonomarevDA/ardupilot/tree/pr-uavcan-v1-hitl). Then press the default command from ardupilot tutorial to load firmware:

```
./waf --targets bin/arducopter --upload
```

## 2. Load parameters

Let's say you start with the default configuration of the autopilot (after pressing 'Reset all to firmware default').

You will see something like on the picture below:

<img src="step_2_1_default.png" alt="drawing" width="640"/>
<img src="step_2_2_default.png" alt="drawing" width="640"/>

Fig. The default state of the autopilot illustrated in QGC

The autopilot will send notifications that vehicle is not configurated yet.

Press `load from file` button and choose corresponded [file with parameters]().

The example how to do it is illustrated on the pictures below:

<img src="step_2_3_load_from_file.png" alt="drawing" width="640"/>
<img src="step_2_4_choose_file.png" alt="drawing" width="640"/>

Fig. Loading parameters to the autopilot in QGC

Accept all suggested parameters changes and reboot the autopilot.

```
Note: if the QGC application terminated unexpectedly, just run it again and press the reboot vehicle button.
```

After reboot your internal board sensors should be already disabled.

## 3. Load registers

After reboot the cyphal driver will be activated on CAN 1 bus, but registers will not be loaded yet. Typically, the autopilot will show you following errors.

<img src="step_3_errors_and_warnings.png" alt="drawing" width="800"/>

Fig. Errors and warnings after first load

So, load parameters one more time. Since registers configuration is performed only during initialization, you need to reboot the autopilot as well.

## 4. Ready to fly

If the connection between the autopilot and the simulator is ok, the vehicle will be ready to fly. In the window with notification it will always send a single warning about Cyphal IMU to remember you that you are in HITL mode and on a custom firmware.

<img src="step_5_ready_to_fly.png" alt="drawing" width="640"/>

Fig. The vehicle is ready to fly

If the autopilot receives nothing from CAN-bus, it will be `Not Ready` and it will use dummy IMU measurements and notify about EKF and sensors such as Baro and Compass.

At this stage, the autopilot internal sensors should be disabled as well. If they are enabled, the simulation can't work properly. 

```
Note 1. I can foresee some problems on autopilot other than CUAV V5+, but you can try to handle them by yourself.
```

```
Note 2. It is necessary to initially connnect the hardware with the simulator via sniffer, run the simulator and only then power/reboot the vehicle. If the vehicle starts with no input data at the beginning it will not allow to fly.
```
