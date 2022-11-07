# PX4 MAVLink SITL simulation configuration

## 1. Software requirements

For MAVLink SITL simulation you can use any version of the software you want. It was tested on v1.13 and v1.12.

## 2. Autopilot setup

This step is highly different for HITL and SITL modes.

**1. PX4 MAVLink SITL mode** requires running PX4 flight stack on your PC.

Depending on which airframe you are going to use, you need to run px4.launch file with an argument corresponded to your airframe.

If you are going to use the simulator for `inno_vtol` dynamics, you may run:

```
roslaunch px4 px4.launch vehicle:=innopolis_vtol
```

If you are going to use the simulator for `flight_goggles` dynamics, you may run:

```
roslaunch px4 px4.launch vehicle:=iris
```

For extended documentation about the PX4 SITL flight stack, please check the official PX4 instruction.

## 3. Build and load

It is expected that you already familiar with PX4, have everything installed according to the official PX4 instruction.

To build in MAVLink SITL mode run:

```
cd PX4-Autopilot
git checkout <desired_branch>
git submodule update --init --recursive
make clean
make distclean
```

Then for MAVLink SITL mode:

```bash
DONT_RUN=1 make px4_sitl gazebo_standard_vtol
```
or

```bash
DONT_RUN=1 make px4_sitl gazebo_iris
```

Don't forget to add these lines to your `.bashrc` file, don't forget to change `~/PX4-Autopilot` to your actual Firmware path

```
source ~/catkin_ws/devel/setup.bash    # (optional)
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo
```

## 4. Load parameters

The easiest way is to simply load the parameters from the file.

1. Run QGC, load the corresponded [params](../../params/) into your vehicle and accept all changes
2. If your QGC application terminated unexpectedly, just run it again
3. Restart your vehicle via QGC

You need to load the parameters and reboot the vehicle twice!
 
![usage_load_params](usage_load_params.png?raw=true "usage_load_params")

After second parameters load, the vehicle should be completely configured.

## 5. Ready to fly

After restarting check correspondences of your airframe.

If the simulation is connected, the vehicle should be `Ready To Fly`.

![usage_check_airframe](usage_check_airframe.png?raw=true "usage_check_airframe")
