rostopic pub /mavros/hil/state mavros_msgs/HilStateQuaternion "header: auto
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
angular_velocity: {x: 0.0, y: 0.0, z: 0.0}
linear_acceleration: {x: 0.01, y: 0.0, z: 0.0}
linear_velocity: {x: 0.0, y: 0.0, z: 0.0}
geo: {latitude: 55.10, longitude: 40.0, altitude: 10.0}
ind_airspeed: 0.0
true_airspeed: 0.0" -r 500 -s &

rostopic pub /mavros/hil/rc_inputs mavros_msgs/RCIn "header: auto
rssi: 0
channels:
- 1
- 2
- 3
- 4
- 5
- 6
- 7
- 8" -r 5 -s &

rostopic pub /mavros/hil/gps mavros_msgs/HilGPS "header: auto
fix_type: 0
geo: {latitude: 55.10, longitude: 40.0, altitude: 10.0}
eph: 10
epv: 10
vel: 0
vn: 0
ve: 0
vd: 0
cog: 10
satellites_visible: 10" -r 5 -s &

rostopic pub /mavros/hil/imu_ned mavros_msgs/HilSensor "header: auto
acc: {x: 0.0, y: 0.0, z: 0.0}
gyro: {x: 0.0, y: 0.0, z: 0.0}
mag: {x: 0.0, y: 0.0, z: 0.0}
abs_pressure: 956.024
diff_pressure: 0
pressure_alt: 487.376
temperature: 20.0
fields_updated: 8191" -r 550 -s