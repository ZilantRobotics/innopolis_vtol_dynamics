# Innopolis VTOL dynamics simulator

Innopolis VTOL dynamics simulator is a set of ROS packages suggested full simulation for UAV based on PX4.

![dynamics](img/dynamics.png?raw=true "dynamics")

Typical PX4 simulations ways are [SITL and HITL](https://docs.px4.io/master/en/simulation/):
- SITL (software in the loop) allows you to run simulation and flight stack fully on your computer, it doesn't cover hardware related modules,
- HITL (hardware in the loop) allows to run flight stack on real device, but it still doesn't cover actuator and sensor related modules because autopilot works in special mode and uses special MAVLink HITL messages instead of real actuators and sensor drivers.

The key feature of this simulation is to run it in such way that the hardware knows nothing about simulation. This could be possible using uavcan. For users, expecialy those using the uavcan network for uav, it can be very usefull, because it covers more PX4 modules than standard SITL and HITL.

So, Inno VTOL dynamics simulation allows to run simulation in both SITL and `uavcan HITL` mode. It also allows to run InnopolisSimulator (left part of the first figure) and visualize forces and moments in RVIZ (right part of the first figure).

# Design

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

To communicate with flight stack via communicator `UAV dynamics` node subscribes on following topics:

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


# Installation and building:

It's assumed that you are using Ubuntu 18.04 with installed ROS and created catkin_ws. Ubuntu 20.04 may works as well, but we didn't test it.

The whole system required several packages.

**1. Inno Dynamics**

Clone this repository with submodules and follow the instruction from [Dockerfile](Dockerfile).

**2. (optional) PX4 Autopilot**

You need [following version of PX4 Autopilot](https://github.com/PonomarevDA/Firmware/tree/px4_v1.12.1_inno_vtol_dynamics/ROMFS/px4fmu_common/init.d/airframes).

For installation use official instruction and [InnopolisAero/PX4-Autopilot](https://github.com/InnopolisAero/PX4-Autopilot/tree/px4_v1.12.1_inno_vtol_dynamics).

To build either in SITL or in TRUE HITL mode run:

```
cd Firmware
git checkout px4_v1.12.1_inno_vtol_dynamics
git submodule update --init --recursive && make clean && make distclean
```

Then for SITL mode:

```
DONT_RUN=1 make px4_sitl gazebo_innopolis_vtol
DONT_RUN=1 make px4_sitl gazebo_iris
```

And for HITL mode for Cuav V5+ type line below. If you use another hardware, read the [PX4 doc](https://dev.px4.io/master/en/setup/building_px4.html) for details.

```
make px4_fmu-v5_default upload
```

Don't forget to add these lines to your `.bashrc` file, don't forget to change `~/Firmware` to your actual Firmware path

```
source ~/catkin_ws/devel/setup.bash    # (optional)
source ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware/Tools/sitl_gazebo
```

**3. InnoSimulator**

InnoSimulator is a photorealistic simulator.

To use it you should install [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface) and [InnoSimulator](https://github.com/inno-robolab/InnoSimulator).

Use branch `uavcan` in `inno_sim_interface` repository.

**4. Building**
Build all by typing `./catkin_build.sh` from `InnoDynamics` package.

# Usage example

1. Run any of `scripts/start*` scripts.
If you run `hitl` script, it will automatically attached slcan based on your serial port (if sniffer is connected).
Example:
```bash
./scripts/start_hitl_inno_vtol.sh
```
2. Run QGC
3. Setup your vehicle
- Run QGC and load correposponded [params](uav_dynamics/inno_vtol_dynamics/config/) into vehicle
- Restart your vehicle and QGC
- Note: sometimes from the first attempt params is not loaded correctly, so you may try it twice
 
![usage_load_params](img/usage_load_params.png?raw=true "usage_load_params")

4. Check correspondences of your airframe and that vehicle is `Ready To Fly`

![usage_check_airframe](img/usage_check_airframe.png?raw=true "usage_check_airframe")

5. (optional) Visualization using InnoSimulator

If you set parameter `run_inno_sim_bridge:=true` or leave it by default, you will only need to type something like that:

```bash
roscd inno_sim_interface/cfg
~/software/InnoSimulator-Linux64-2020.1.2/InnoSimulator.x86_64 --config config.yaml
```

# Repos used as references:

1. [flightgoggles_uav_dynamics (multicopter)](https://github.com/mit-fast/FlightGoggles/blob/master/flightgoggles_uav_dynamics/) - read their [paper](https://arxiv.org/pdf/1905.11377.pdf)
2. [PX4 mavlink communicator](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge)
3. [sitl_gazebo](https://github.com/PX4/sitl_gazebo)
4. [innopolis_vtol_indi](https://github.com/InnopolisAero/innopolis_vtol_indi) - dynamics written in Octave
5. [InnoSimulator](https://github.com/inno-robolab/InnoSimulator) - photorealistic simulator
6. [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface) - bridge between dynamics and photorealistic simulator

# Tests
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
