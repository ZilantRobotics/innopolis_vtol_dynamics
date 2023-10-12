# Test scenarios

Here we have a few test scenarios:

## HITL test scenarios

### 1. The simplest Takeoff And Land scenario

This is the simplest possible test scenario: take off, wait a few seconds, and land. It is the fastest scenario and it is intended to be triggered on every commit as part of CI.

| Plan | Flight log example | Approximate execution time |
|-|-|-|
| [takeoff_and_land.plan](takeoff_and_land.plan) | [log10](https://review.px4.io/plot_app?log=9aeb4932-991a-4856-b292-187d6a9f37af) | 30 seconds |

### 2. The longest flight scenario

This is the longest test scenario. It takes ~ 60 minutes. It is dedicated for testing the stability of the Cyphal HITL communication.

| Plan | Flight log example | Approximate execution time |
|-|-|-|
| [sviyazhsk_60_minutes.plan](sviyazhsk_60_minutes.plan) | [log5](https://review.px4.io/plot_app?log=651ad2b4-149c-4b6c-8dd5-70a0c92c3617) | 60 minutes |

## 3D part test scenarios

### 1. Last Mile Aerologistics Test Scenario

> more details here: [delivery](delivery/README.md)

### 2. Construction Inspection Detection Scenario

> more details here: [inspection](inspection/README.md)

## Additional scenarios

The following scenarios might be a part of the previous scenarios.

### 1. Ground infrastructure testing scenario:

Checking the interaction of unmanned aerial vehicles with ground infrastructure within the framework of air logistics tasks.

### 2. Fault Test Scenario

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

## Key points

There are 4 landing stations:

| Position, Lat/Lon               | Picture |
| ------------------------- | ------- |
| In the yard (55.7487847875, 48.7430507069) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_yard.png" width="512" alt="drawing"/> |  |
| Kazan Express point (55.7503992494, 48.7481202714) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_kazanexpress.png" width="512" alt="drawing"/> |
| Technopark office parking (55.7517506178, 48.7506804476) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_technopark.png" width="512" alt="drawing"/> |
| Building (55.7531869667, 48.7510098844) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_building.png" width="512" alt="drawing"/> |

The first 3 landing stations are used for deliveries. The last one is used for a regular inspection of the buildings.
