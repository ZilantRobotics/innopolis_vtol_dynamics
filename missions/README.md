# Test scenarios

UAV HITL simulator supports multiple types of test scenarios.

### 1. Simple CI Scnearios

There are a few simple scenarios intended to be used as part of CI process. They shoud be trigger every commit pushed to the UAV HITL simulator repository.

More details in [ci](ci/README.md) folder.

### 2. Last Mile Aerologistics (Delivery) Scenarios

This scenrios are indended to be used with 3D-simulator.

More details in the [delivery](delivery/README.md) folder.

### 3. Construction Inspection Detection Scenarios

This scenrios are indended to be used with 3D-simulator.

More details in the [inspection](inspection/README.md) folder.

### 4. Ground Infrastructure Scenarios

Checking the interaction of unmanned aerial vehicles with ground infrastructure within the framework of air logistics tasks.

These scenarios might be a part of the previous scenarios.

### 5. Fault Scenarios

Evaluation of the behavior and response of UAV to various types of failures during flight.

The simulator supports varios of fault test scenarios. Please, refer to the [scenarios.hpp](https://github.com/RaccoonlabDev/inno_vtol_dynamics/blob/main/src/scenarios.hpp) for details.

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


### Key points

There are 4 landing stations:

| Position, Lat/Lon               | Picture |
| ------------------------- | ------- |
| In the yard (55.7487847875, 48.7430507069) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_yard.png" width="512" alt="drawing"/> |  |
| Kazan Express point (55.7503992494, 48.7481202714) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_kazanexpress.png" width="512" alt="drawing"/> |
| Technopark office parking (55.7517506178, 48.7506804476) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_technopark.png" width="512" alt="drawing"/> |
| Building (55.7531869667, 48.7510098844) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_building.png" width="512" alt="drawing"/> |

The first 3 landing stations are used for deliveries. The last one is used for a regular inspection of the buildings.
