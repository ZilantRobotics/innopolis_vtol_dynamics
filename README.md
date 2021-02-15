# Innopolis VTOL dynamics

Innopolis VTOL dynamics is a ROS node that simulates full Innopolis VTOL dynamics. It includes quadcopter and plane dynamics with aerodynamics coefficients, minimum sensors set.

![dynamics](img/dynamics.png?raw=true "dynamics")

Typical PX4 simulations ways are [SITL and HITL](https://docs.px4.io/master/en/simulation/). While SITL allows you to run simulation and flight stack fully on your computer, HITL allows to run flight stack in real device in special `HITL mode`.

The key feature of this simulation is to run it in such way that the hardware knows nothing about simulation. This could be possible using uavcan. For users, expecialy those using the uavcan network for uav, it can be very usefull, because it covers more PX4 modules than standard SITL and HITL.

So, Inno VTOL dynamics simulation allows to run simulation in both SITL and `uavcan HITL` mode. It also allows to run InnopolisSimulator (left part of the first figure) and visualize forces and moments in RVIZ (right part of the first figure).

# Design

The design of simulator shown below.

![scheme](img/scheme.png?raw=true "scheme")

As you can see, InnoDynamics is a separate ROS node that perform communication via topics and it doesn't matter which type of communication with flight stack you are using.

To communicate with flight stack via [px4 drone communicator]() it publishes and subscribes on following topics:

| № | Type         | topic                      | msg                                   |
| - | ------------ | -------------------------- | ------------------------------------- |
| 1 | subscribtion | /uav/actuators             | sensor_msgs/Joy                       |
| 2 | publication  | /uav/static_temperature    | drone_communicators/StaticTemperature |
| 3 | publication  | /uav/static_pressure       | drone_communicators/StaticPressure    |
| 4 | publication  | /uav/raw_air_data          | drone_communicators/RawAirData        |
| 5 | publication  | /uav/gps_position          | drone_communicators/Fix               |
| 6 | publication  | /uav/imu                   | sensor_msgs/Imu                       |
| 7 | publication  | /uav/mag                   | sensor_msgs/MagneticField             |

To work in pair with [InnoSimulator](https://github.com/inno-robolab/InnoSimulator) as physics engine via [InnoSimInterface ros bridge package](https://github.com/InnopolisAero/inno_sim_interface) it publishes and subscribes on following topics.

| № | topic             | msg                             |
| - | ----------------- | ------------------------------- |
| 1 | /uav/actuators    | sensor_msgs/Joy                 |
| 5 | /uav/gps_position | drone_communicators/Fix         |
| 2 | /uav/attitude     | geometry_msgs/QuaternionStamped |


# Installation and building:

At first, it's assumed that you are using Ubuntu 18.04 with installed ROS and created catkin_ws. 

The whole system required several packages.

**1. Inno Dynamics**

Clone `this package`, [timed_roslaunch](https://github.com/MoriKen254/timed_roslaunch.git) that allows to run nodes with delay from launch file and `geographiclib_conversions` package:

```
git clone https://github.com/InnopolisAero/innopolis_vtol_dynamics.git
git clone https://github.com/MoriKen254/timed_roslaunch.git
# there is no geographiclib_conversions package yet
```

Then install python packages:

```
pip install -r requirements
```

And download `wmm2020` from [here](https://geographiclib.sourceforge.io/html/magnetic.html).

**2. PX4 Autopilot**

Use official instruction and [this version of PX4 Autopilot](https://github.com/InnopolisAero/Inno_PX4_Firmware/tree/inno_dynamics).

To build either in SITL or in TRUE HITL mode run:

```
cd Firmware
git checkout inno_dynamics
make clean
make distclean
```

Then for SITL mode:

```
DONT_RUN=1 make px4_sitl gazebo_standard_vtol
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

**3. Drone communicator**

This package establishes communication between flight stack and dynamics, it also required `mavlink` and `mavros` packages which could be installed while PX4 Autopilot installation.

Use instruction from `drone_communicators repo`. It's preaty simple. 


**4. InnoSimulator**

InnoSimulator is a photorealistic simulator.

To use it you should install [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface) and [InnoSimulator](https://github.com/inno-robolab/InnoSimulator).

**Building**
Build all by typing `./catkin_build.sh` from `InnoDynamics` package.


# Usage

1. Running dymamics, PX4 flight stack and communicator between them:
```roslaunch innopolis_vtol_dynamics dynamics.launch```

There are 3 optional usefull parameters:
- vehicle:=standard_vtol - it allows to choose one of 2 vehicles: standard_vtol (by default, it means Innopolis VTOL) and iris
- run_rviz:=false - it allows to run rviz to visualize orientation, forces and moments (it is turned off by default)
- sitl_instead_of_hitl:=false - choose SITL or True HITL mode

2. Controlling and monitoring flight using QGroundControl

```~/software/qgroundcontrol/QGroundControl.AppImage```

3. Visualization using InnoSimulator

```
roscd inno_sim_interface/cfg
~/software/InnoSimulator-Linux64-2020.1.2/InnoSimulator.x86_64 --config config.yaml

roslaunch innopolis_vtol_dynamics visualization_using_inno_sim.launch
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
