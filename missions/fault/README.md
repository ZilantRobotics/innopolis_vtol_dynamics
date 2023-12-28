# Fault scenarios

Evaluation of the behavior and response of UAV to various types of failures during flight.

The simulator supports varios of fault test scenarios. Please, refer to the [scenarios.hpp](https://github.com/ZilantRobotics/uav_hitl_dynamics/blob/main/src/scenarios.hpp) for details.

To run a desired scenario, you should publish the corresponded scenario number to the `/uav/scenario` ROS topic as follows:

```bash
rostopic pub /uav/scenario std_msgs/UInt8 "data: 0"
```

The following scenarios are currently supported:

| Fault scenario                    | Make a fault | Restore |
| --------------------------------- | ------------ | ------- |
| Barometer: disabled               | 0  | 1
| Differential pressure: disabled   | 2  | 3
| GNSS: disabled                    | 4  | 5
| Internal combustion engine: stale | 6  | 7
| Magnetomter: disabled             | 8  | 9
| Feedback from all ESC: disabled   | 10 | 11

An example of flight log with the scenario `Feedback from all ESC: disabled`: [flight_log](https://review.px4.io/plot_app?log=14f6605e-c5c9-47ae-b792-852bb194b71d).

These scenarios might be a part of the previous scenarios.
