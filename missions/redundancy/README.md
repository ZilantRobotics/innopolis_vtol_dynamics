# Redundancy

Cyphal/CAN and DroneCAN protocols allow to use redundant CAN-bus and redundant sensor.

## 1. Redundant differential pressure sensors scenario

Prerequisites:
- VTOL with 2 diffirential pressure / airspeed sensors

Steps:

| № | Step | Expected behaviour |
| - | ---- | ------------------ |
| 1 | Takeoff, go into FW mode |
| 2 | Disable first airspeed | The airspeed selector switches to the second airspeed and continue the flight continues
| 3 | Re-enable first airspeed
| 4 | Disable second airspeed | The airspeed selector switches to the first second airspeed and the flight continues
| 5 | Re-enable second airspeed
| 6 | Land

### 1.1. Flight log example

During the flight, the first airspeed was disabled, the vehicle triggered a failsafe behavior (adivese RTL) and the airspeed selector switched to the second airspeed instance:

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/scenarios/redundant_dprs.gif" alt="drawing"/>

The disabled airspeed was veryfied in MAVLink console with `listener airspeed`:

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/scenarios/redundant_dprs.png" alt="drawing"/>

The full flight log: [Cyphal VTOL PX4 v1.14](https://review.px4.io/plot_app?log=a6a3fed5-4917-4887-b32c-812defe7cb1d).

## 2. Octorotor

Flight log: [Cyphal Octorotor PX4 v1.14](https://review.px4.io/plot_app?log=6531bc4c-c593-44f6-9d0e-5c58d7d17a91).

## Enable and disable port with Yakut

You can disable and re-enable any sensor by changing the corresponded port ID in Yakut.

Type `y mon` to display the information about online nodes and network traffic in real time:

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/cyphal/yakut_monitor.png" width="512" alt="drawing"/>


For example, the port identifier of the first differential pressure is 2600.

You can disable it by setting the port ID to 65535:

```bash
y r 2 uavcan.pub.aspd.dpres.0.id 65535
```

And re-enable by setting the port ID to 2600:

```bash
y r 2 uavcan.pub.aspd.dpres.0.id 2600
```

You can get all existed registers by typing:

```bash
y rl 2
```
