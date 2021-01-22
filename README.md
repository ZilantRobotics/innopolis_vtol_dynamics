# Innopolis VTOL dynamics simulator

The goal is to create a ROS node to simualate full Innopolis VTOL dynamics. It includes quadcopter and plane dynamics with aerodynamics coefficients, minimum sensors set. Simulation node can contact with PX4 in 2 modes:
- SITL mode simulatuion via [MAVLink HIL_* messages](https://mavlink.io/en/messages/common.html#HIL_CONTROLS)
- true HITL mode simulation via uavcan (not ready yet)

It could be used in pair with InnoSimulator as physics engine.

Repos used as references:

1. [flightgoggles_uav_dynamics (multicopter)](https://github.com/mit-fast/FlightGoggles/blob/master/flightgoggles_uav_dynamics/) - read their [paper](https://arxiv.org/pdf/1905.11377.pdf)
2. [PX4 mavlink communicator](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge)
3. [sitl_gazebo](https://github.com/PX4/sitl_gazebo)
4. [innopolis_vtol_indi](https://github.com/InnopolisAero/innopolis_vtol_indi) - dynamics written in Octave
5. [InnoSimulator](https://github.com/inno-robolab/InnoSimulator) - photorealistic simulator
6. [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface) - bridge between dynamics and photorealistic simulator

# Preparation:

1. Build [this version of PX4](https://github.com/InnopolisAero/Inno_PX4_Firmware/tree/inno_dynamics) either in SITL or in TRUE HITL mode

```
cd Firmware
git checkout inno_dynamics
make clean
make distclean
```

SITL mode:

```
DONT_RUN=1 make px4_sitl gazebo_standard_vtol
```

True HITL mode. You should build PX4 Autopilot in a way corresponded to your hardware and uload it, an example for cuav V5+:

```
make px4_fmu-v5_default upload
```

Read the [PX4 doc](https://dev.px4.io/master/en/setup/building_px4.html) for details.


2. Add these lines to your `.bashrc` file, don't forget to change `~/Firmware` to your actual Firmware path

```
source ~/catkin_ws/devel/setup.bash    # (optional)
source ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware/Tools/sitl_gazebo
```

3. Install `geographiclib_conversions` package
4. Download `wmm2020` from [here](https://geographiclib.sourceforge.io/html/magnetic.html)
5. (opional) To use `scripts/run.sh` you should set up correct Firmare path and install tmux
6. Install [timed_roslaunch](https://github.com/MoriKen254/timed_roslaunch.git). This package allows to run nodes with delay from launch file.
7. (optional) Install [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface) with [InnoSimulator](https://github.com/inno-robolab/InnoSimulator)
8. Install python packages
`pip install -r requirements`
9.  Build InnoDynamics by typing `./catkin_build`

# How to use InnopolisDynamic simulator in both SITL and True HITL modes:

1. Run dymamics and px4:
```roslaunch innopolis_vtol_dynamics dynamics.launch```

There are 3 optional usefull parameters:
- vehicle:=standard_vtol - it allows to choose one of 2 vehicles: standard_vtol (by default, it means Innopolis VTOL) and iris
- run_rviz:=false - it allows to run rviz to visualize orientation, forces and moments (it is turned off by default)
- sitl_instead_of_hitl:=false - choose SITL or True HITL mode

2. To control and monitor flight using QGroundControl
```~/software/qgroundcontrol/QGroundControl.AppImage```

3. To visualize using InnoSimulator
- at first, you need to run simulator:
```
roscd inno_sim_interface/cfg
~/software/InnoSimulator-Linux64-2020.1.2/InnoSimulator.x86_64 --config config.yaml
```
- and then bridge data between dynamics and InnoSimulator:
```roslaunch innopolis_vtol_dynamics visualization_using_inno_sim.launch```

# (optional) How to use Gazebo simulator instead of InnoDynamics:

- Instead of `innopolis_vtol_dynamics dynamics.launch` use `roslaunch innopolis_vtol_dynamics gazebo.launch`

# Tests
We use [GoogleTest](https://github.com/google/googletest/tree/master/googletest)
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

# How it works?

not rdy yet