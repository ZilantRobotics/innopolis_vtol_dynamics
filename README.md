# Innopolis VTOL dynamics simulator

The goal is to create a ROS node to simualate full Innopolis VTOL dynamics. It includes quadcopter and plane dynamics with aerodynamics coefficients, minimum sensors set. Simulation node should contact with PX4 SITL mode simulatuion via MAVLink HIL_* messages https://mavlink.io/en/messages/common.html#HIL_CONTROLS. Could be used in pair with InnoSimulator as physics engine.

Repos used as references:

1. [flightgoggles_uav_dynamics (multicopter)](https://github.com/mit-fast/FlightGoggles/blob/master/flightgoggles_uav_dynamics/). Also you can read [the paper about this simulator](https://arxiv.org/pdf/1905.11377.pdf)
2. [PX4 communicator](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge)
3. [sitl_gazebo](https://github.com/PX4/sitl_gazebo)
4. [innopolis_vtol_indi](https://github.com/InnopolisAero/innopolis_vtol_indi)

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

3. Manually install `geographiclib_conversions` package
4. Download `wmm2020` from [here](https://geographiclib.sourceforge.io/html/magnetic.html)
5. (opional) To use `scripts/run.sh` you should set up correct Firmare path and install tmux
6. Manually install `timed_roslaunch` package from [here](https://github.com/MoriKen254/timed_roslaunch.git) to use launch files.
7. Build using `./catkin_build`

# How to use InnopolisDynamic simulator only for iris model yet:

- Run in first terminal:
`roslaunch innopolis_vtol_dynamics node.launch`
- Run in second terminal:
`roslaunch innopolis_vtol_dynamics px4.launch`
- To control and monitor flight run QGroundControl


# How to use Gazebo simulator:

- Run in first terminal:
`roslaunch innopolis_vtol_dynamics run_gazebo.launch vehicle:=iris`
where instead of `iris` you can use any other model, for example `standard_vtol`
- To control and monitor flight run QGroundControl