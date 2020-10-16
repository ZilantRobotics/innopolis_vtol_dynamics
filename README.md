# Innopolis VTOL dynamics simulator

The goal is to create a ROS node to simualate full Innopolis VTOL dynamics. It includes quadcopter and plane dynamics with aerodynamics coefficients, minimum sensors set. Simulation node should contact with PX4 SITL mode simulatuion via MAVLink HIL_* messages https://mavlink.io/en/messages/common.html#HIL_CONTROLS. Could be used in pair with InnoSimulator as physics engine.

Repos used as references:

1. [flightgoggles_uav_dynamics (multicopter)](https://github.com/mit-fast/FlightGoggles/blob/master/flightgoggles_uav_dynamics/). Also you can read [the paper about this simulator](https://arxiv.org/pdf/1905.11377.pdf)
2. [PX4 communicator](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge)
3. [sitl_gazebo](https://github.com/PX4/sitl_gazebo)
4. [innopolis_vtol_indi](https://github.com/InnopolisAero/innopolis_vtol_indi)
5. [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface)
6. [InnoSimulator](https://github.com/inno-robolab/InnoSimulator)

# Preparation:

1. Build PX4 on SITL

```
DONT_RUN=1 make px4_sitl_default gazebo
```

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
6. Install [timed_roslaunch](https://github.com/MoriKen254/timed_roslaunch.git)
7. Install [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface) with [InnoSimulator](https://github.com/inno-robolab/InnoSimulator)
8. Install python packages
`pip install -r requirements`
9.  Build using `./catkin_build`

# How to use InnopolisDynamic simulator only for iris model yet:

1. Run dymamics and px4:
```roslaunch innopolis_vtol_dynamics dynamics.launch```

2. To control and monitor flight using QGroundControl
```~/software/qgroundcontrol/QGroundControl.AppImage```

3. To visualize using InnoSimulator
```
roscd inno_sim_interface/cfg
~/software/InnoSimulator-Linux64-2020.1.2/InnoSimulator.x86_64 --config config.yaml
```

4. To bridge data between dynamics and InnoSimulator:
```roslaunch innopolis_vtol_dynamics visualization_using_inno_sim.launch```

# How to use Gazebo simulator:

- Instead of `innopolis_vtol_dynamics dynamics.launch` use `roslaunch innopolis_vtol_dynamics gazebo.launch vehicle:=iris`