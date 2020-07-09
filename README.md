# Innopolis VTOL dynamics simulator

The goal is to create a ROS node to simualate full Innopolis VTOL dynamics. It includes quadcopter and plane dynamics with aerodynamics coefficients, minimum sensors set. Simulation node should contact with PX4 SITL mode simulatuion via MAVLink HIL_* messages https://mavlink.io/en/messages/common.html#HIL_CONTROLS. Could be used in pair with InnoSimulator as physics engine.

Repos used as references:
https://github.com/mit-fast/FlightGoggles/blob/master/flightgoggles_uav_dynamics/
https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge
https://github.com/PX4/sitl_gazebo

