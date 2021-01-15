#!/bin/bash
rostopic pub -r 20 /sim/attitude geometry_msgs/QuaternionStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
quaternion:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0" &
rostopic pub -r 20 /sim/gps_position sensor_msgs/NavSatFix "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
status: {status: 0, service: 0}
latitude: 4874268390
longitude: 5575444250
altitude: -6500
position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
position_covariance_type: 0" &
rostopic echo /sim/actuators