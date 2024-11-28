[![catkin_build](https://github.com/ZilantRobotics/innopolis_vtol_dynamics/actions/workflows/catkin_build.yml/badge.svg)](https://github.com/ZilantRobotics/innopolis_vtol_dynamics/actions/workflows/catkin_build.yml) [![docker](https://github.com/ZilantRobotics/innopolis_vtol_dynamics/actions/workflows/docker.yml/badge.svg)](https://github.com/ZilantRobotics/innopolis_vtol_dynamics/actions/workflows/docker.yml) [![pylint](https://github.com/ZilantRobotics/innopolis_vtol_dynamics/actions/workflows/pylint.yml/badge.svg)](https://github.com/ZilantRobotics/innopolis_vtol_dynamics/actions/workflows/pylint.yml)

# UAV HITL Simulator

UAV HITL Simulator brings up a set of ROS packages, config files and instructions to establish a full simulation for UAV based on PX4/Ardupilot autopilot.

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/dynamics.png" alt="drawing"/>

The key feature of this simulation is to run it in such a way that the hardware knows nothing about the simulation. This can be done with Cyphal/DroneCAN. It covers more PX4 modules than standard SITL and HITL.

**Purpose**

- Simulation of UAV onboard systems at a low hardware level
- Testing and debugging of UAV control systems using a CAN-bus
- Development and testing of intelligent automatic control systems for UAVs
- Training in the development and use of drones, including creating datasets and automated testing

**Minimal requirements (for HITL only):**

- Operating System: Linux based OS such as Ubuntu 22.04
- Raspberry PI 4 is enough

**Recommended requirements (for HITL + 3D simulator):**

- Operating System: We've tailored the simulator for modern versions of Windows, Linux, and Mac. Choose the build that matches your OS.
- CPU: Aim for an Intel i7 from the 11th or 12th generation. For those using AMD, any equivalent processor will suffice.
- RAM: 16GB is a recommended minimum, but more is always better for performance.

**Required hardware:**

- Flight controller: fmu-v5, fmu-v6c or fmu-v6x
- CAN-sniffer

## 1. USE CASES

By centralizing communication through CAN and leveraging the HITL simulator, you can perform a wide range of tests that were previously complex or impractical. Let's consider them.

### 1.1. CI/CD Unit

The HITL simulator itself is not computationally expensive. You can run it even on single-board computers like the Raspberry Pi. This feature allows the HITL simulator to be used as part of CI/CD process. Each time a developer makes a commit to the autopilot software, a the compiled binary can be deployed to a real flight controller and tested with HITL simulator. It is especially useful for developers actively working with DroneCAN/Cyphal drivers or related parts of the autopilot software.

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/use_case_1.png" alt="drawing" width="800"/>

A few examples of test scenarios for CI/CD are shown in the table below.

| Test | Description |
|-|-|
| 1. Takeoff And Land | This is the simplest possible test scenario: take off, wait a few seconds, and land. It is the fastest scenario and it is intended to be triggered on every commit as part of CI. </br> Approximate duration: 30 sec </br> Plan: tests/ci/takeoff_and_land.plan |
| 2. Square flight | Simple quadcopter flight test. </br> Approximate duration: 1 minute </br> Plan: tests/ci/square.plan |
| 3. VTOL Long flight | This is the longest test scenario. It is dedicated for testing the stability. </br> Approximate duration: 10 minutes </br> Plan: tests/ci/sviyazhsk_vtol.plan |

Any of these scenarios can be run with 3 steps.

```bash
# 1. Upload the new firmware to the autopilot
autopilot-configurator --firmware <path_to_the_binary.px4>

# 2. Run the simulator itself with the desired protocol and airframe.
./scripts/sim.py cq

# 3. Run the test scenario. The default test scenario execution timeout is 5 minutes. For long flights you need to explicitly increase the timeout:
test-scenario --output flight.ulg --timeout 1000 tests/ci/sviyazhsk_vtol.plan
```

### 1.2. HITL Simulator with desktop computer and 3D-simulator

Running the HITL simulator on a desktop unlocks advanced visualization and simulation capabilities. With tools like `gui_tool`, `yakut`, and `rviz`, you can monitor and interact with the simulation in real-time.
If your desktop is equipped with a modern GPU, you can further enhance the experience by integrating a 3D simulator. This setup is ideal for testing complex scenarios such as delivery missions or inspections and can even be used for pilot training, offering a realistic and immersive simulation environment.

The setup is as simple as possible. Just connect a CAN-sniffer and a flight controller to you Desktop via USB and connect the devices with each outher with CAN cable.

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/use_case_2.png" alt="drawing" width="800"/>

**Tool 1. CAN bus analysis tools: gui_tool, yakut**

If you run a HITL simulator, you can then run both yakut, gui_tool and any other similar tools at any time. It allows you analyse the CAN bus in real time.

| DroneCAN: [gui_tool](https://github.com/DroneCAN/gui_tool) | Cyphal: [yakut](https://github.com/OpenCyphal/yakut) |
|-|-|
| <img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/use_case_2_gui_tool.gif" alt="drawing" width="355"/> | <img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/use_case_2_yakut.gif" alt="drawing" width="395"/>

**Tool 2. RVIZ for a flight visualization**

With RVIZ you can visualise the vehicle orientation, vectors of the applied forces, torques, speed and more.

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/use_case_2_rviz.gif" alt="drawing" width="800"/>

How to run:

```bash
./scripts/sim.py rviz vtol_T_300
```

**Tool 3. 3D-simulator**

Simulator3d is a digital environment that replicates the world as closely as a robot or UAV would perceive it, with all interactions being facilitated through the same API formats these robots would use in reality. It is avaliable on GitHub: [ZilantRobotics/Simulator3d](https://github.com/ZilantRobotics/Simulator3d).

**Test scenarios**

| Test scenario | Description |
|-|-|
| <img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/delivery.png" alt="drawing"/> | **Delivery** or Last Mile Aerologistics Scenarios. Testing and optimization of cargo delivery processes using unmanned aerial vehicles. </br> 1. Delivery from KazanExpress to the Yard. 4 minutes. Plan: kazanexpress_to_yard.plan </br> 2. Delivery from KazanExpress to the Technopark. 6 minutes. Plan: kazanexpress_to_technopark.plan |
| <img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/inspection.png" alt="drawing"/> | **Construction Inspection**. </br> 1. Quadcopter. 3:40. Plan: technopark.plan </br> 2. VTOL. Structure scan of the northest techonopark. 9:25. Plan: technopark_structure_scan.plan </br> 3. VTOL. Survey of all technoparks in the town. 28:00. technopark_survey_half_town.plan </br> 3. VTOL. Survey of all the town. 44:40. technopark_survey_full_town.plan |
| <img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/scenarios/redundant_dprs.gif" alt="drawing"/> | **Fault scenarios**. </br> Evaluation of the behavior and response of UAV to various types of failures during flight. </br> 1. Using `/uav/scenario` ROS topic run or stop a specific event, for example disable differential pressure sensor (as shown on the example), gnss, ESC feedback or turn off ICE. Please, check [scenarios.hpp](https://github.com/ZilantRobotics/uav_hitl_dynamics/blob/main/src/scenarios.hpp) for details </br> 2. In Cyphal you can disable/enable any port in real time |

<!--
Cartography	Cartography Scenarios are indended to be used with 3D-simulator.
Ground Infrastructure	Checking the interaction of unmanned aerial vehicles with ground infrastructure within the framework of air logistics tasks. These scenarios might be a part of the previous scenarios.
-->

**How to run:**

Download the simulator before the first usage:

```bash
# 1. Download Simulator3d in Downloads directory.
cd Downloads
wget https://github.com/ZilantRobotics/Simulator3d/releases/download/v0.1.19/ZilantSimulator-Linux64-0.1.19.zip

# 2. Unzip it and make it executable
unzip ZilantSimulator-Linux64-0.1.19.zip
chmod +x ZilantSimulator.x86_64
```

Run it:

```bash
# 1. Run the Simulator3d itself
./ZilantSimulator.x86_64

# 2. Run rosbridge and sim interface nodes
./scripts/docker.sh i
roslaunch innopolis_vtol_dynamics 3d_sim.launch
```

**Key points in 3D simulation**

| Yard | Delivery point | Technopark office parking | Buildings |
| ---- | -------------- | ------------------------- | --------- |
| <img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/landing_station_yard.png" alt="drawing"/> | <img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/landing_station_kazanexpress.png" alt="drawing"/> | <img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/landing_station_technopark.png" alt="drawing"/> | <img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/landing_station_building.png" alt="drawing"/> |

<!-- | lat: 55.7487847875 </br> lon: 48.7430507069 | lat: 55.7503992494 </br> lon: 48.7481202714 | lat: 55.7517506178 </br> lon: 48.7506804476 | lat: 55.7531869667 </br> lon: 48.7510098844 | -->

### 1.3. Testing with Real UAV Components

Connect the HITL simulator to a particular component of a vehicle or almost to the whole system.
- Full-System Simulation: Use actual UAV hardware to simulate real-world operations without physical flight.
- Actuator and Payload Testing: Validate that motors, control surfaces, and payloads respond correctly to simulated inputs.

An example of connection to the whole system is show below.

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/use_case_3.png" alt="drawing" width="800"/>

### 1.4. SITL if you don't have the hardware

SITL mode is out of the scope of the interests of this simulator. But anyway, it happens that you need to test something and you don't have the required hardware in your hand. So, you can run the flight stack and the dynamics on your PC:

Run the HITL dynamics as usual, but choose MAVLink mode, for example:

```bash
./scripts/sim.py mq  # mq stands for PX4 Mavlink Quadcopter
```

Run the PX4 flight stack. You can either build and run it according to PX4 official instructions, or use our Dockerfile to build and ru neverything in a single command:

```bash
./scripts/sim.py px4-sitl iris
```

## 2. USAGE

The simulator is distributed as a Docker image. To simplify the interraction with Docker, a `./scripts/sim.py` script was written. The script configures all the necessary Docker flags, performs automatic firmware upload, configuration, creates a CAN interface, and generally provides a simple interface to interact with the simulator.

**Step 1. Clone repository with submodules**

```bash
git clone https://github.com/ZilantRobotics/innopolis_vtol_dynamics.git --recursive
```

Whenever you pull this repository, don't forget to update submodules:

```bash
git submodule update --init --recursive
```

**Step 2. Install requirements and Build/pull the docker image**

```bash
pip install -r requirements.txt
```

To build docker image, type:

```bash
./scripts/sim.py b # b stands for build
```

> An image on dockerhub usually is not up to date, so it's better to build manually

**Step 3. Connect everything together for HITL**

> You should skip this step if you want to run PX4 MAVLink SITL mode. Please follow [docs/px4/mavlink](docs/px4/mavlink.md) for details.

Typically we use [CUAV v5+](https://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html) and [RL-programmer-sniffer](https://docs.raccoonlab.co/guide/programmer_sniffer/), but it might be anything else.

An example of a connection is shown in the picture below.

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/sniffer_connection.png" alt="drawing"  width="800"/>

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

## 3. SUPPORTED MODES

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

## 4. DESIGN

VTOL HITL Dynamics Simulator is designed to be modular. It is divided into the following main components:

1. `UAV dynamics` is the main node that handles actuator commands from the communicator, performs dynamics simulation, and publishes vehicle and sensors states.
2. `Communicator` is the set of nodes that communicate with the `PX4 flight stack` in HITL (via Cyphal/DroneCAN) and SITL (via MAVLink) modes.
3. `inno_sim_interface` is a bridge for interaction with `3D-Simulator` through ROS.

The design of the simulator is shown below.

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/welcome/scheme.png" alt="drawing"/>

## 5. EXAMPLE

Check the video below.

[![Cyphal/DroneCAN HITL VTOL dynamics simulator](https://img.youtube.com/vi/e9MREW6tCmE/0.jpg)](https://youtu.be/e9MREW6tCmE)

## 6. REFERENCE

Docs:

- [Developer docs](docs/dev_docs.md)
- [PX4 MAVLink SITL manual configuration instructions](docs/px4/mavlink.md)

Outdated manual instructions:

- [PX4 Cyphal manual configuration instructions](docs/px4/cyphal.md)
- [PX4 DroneCAN manual configuration instructions](docs/px4/dronecan.md)
- [ArduPilot manual configuration instructions](docs/ardupilot/README.md)

## 7. CHANGELOG NOTES

| Version | ReleaseDate | Major changes |
| ------- | ----------- | ------------- |
| v0.9.0 | Nov .., 2024 | Add fmu-v6c and fmu-v6x support beside fmu-v5 |
| v0.8.0 | Jun 10, 2024 | Update PX4 from v1.14 to v1.15 |
| v0.7.0 | Oct 31, 2023 | Update PX4 from v1.13 to v1.14 |
| v0.6.0 | Jul 16, 2023 | Add Octorotor dynamics, fault scenarios and Cyphal ESC feedback |
| v0.5.0 | May 17, 2023 | Add Cyphal PX4 v1.13.0 quadcopter, update DroneCAN PX4 from v1.12.1 to v1.13.0 |
| v0.4.0 | May 16, 2022 | Add Cyphal/DroneCAN custom version of Ardupilot |
| v0.3.0 | Aug 25, 2021 | Add Docker |
| v0.2.0 | Aug 17, 2021 | Update to public DroneCAN PX4 v1.12.1 |
| v0.1.0 | Mar 18, 2021 | First public release for private custom version of DroneCAN PX4 v1.11.2, only CUAV V5+, SITL and HITL modes |
