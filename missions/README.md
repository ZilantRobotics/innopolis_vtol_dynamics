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

Testing and optimization of cargo delivery processes using unmanned aerial vehicles.

| Plan | Flight log example | Approximate execution time |
|-|-|-|
| [Deliver from KazanExpress to the Yard, come back and land](delivery_from_kazanexpress_to_yard.plan) | [log16](https://review.px4.io/plot_app?log=c63a3a10-5f1f-4d51-97a9-cfa0970b9f3c) | 7 minutes |
| [Deliver from KazanExpress to the Technopark, come back and land](delivery_from_kazanexpress_to_technopark.plan) | [log22](https://review.px4.io/plot_app?log=3bea8e60-9f15-47ef-8954-be7cf0ae5e65) | 6 minutes |

An illustration of the flight:

<img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/delivery.png" alt="drawing"/>


### 2. Construction Inspection Detection Scenario

Development of the process of automatic regular inspection of construction sites using unmanned aerial vehicles.

| Plan | Flight log example | Approximate execution time |
|-|-|-|
| [inspection_technopark.plan](inspection_technopark.plan) (takeoff from technopark landing station, inspect the building process, come back and land) | [log18](https://review.px4.io/plot_app?log=2087803f-a0ea-41c6-b322-eb8ef06ad82d), [log19](https://review.px4.io/plot_app?log=03ceb5fd-3eb6-4f11-a743-4a65a1246c71) | 4 minutes |

An illustration of the flight:

<img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/inspection.png" alt="drawing"/>


## Additional scenarios

The following scenarios might be a part of the previous scenarios.

### 1. Ground infrastructure testing scenario:

Checking the interaction of unmanned aerial vehicles with ground infrastructure within the framework of air logistics tasks.

### 2. Fault Test Scenario

Evaluation of the behavior and response of unmanned aerial vehicles to various types of failures during flight.

The simulator supports varios of fault test scenarios. Please, refer to the [scenarios.hpp](https://github.com/RaccoonlabDev/inno_vtol_dynamics/blob/main/src/scenarios.hpp).

To run a desired scenario, you should publish the corresponded scenario number to the `/uav/scenario` ROS topic as follows:

```bash
rostopic pub /uav/scenario std_msgs/UInt8 "data: 0"
```


## Key points

There are 4 landing stations:

| Position, Lat/Lon               | Picture |
| ------------------------- | ------- |
| In the yard (55.7487847875, 48.7430507069) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_yard.png" width="512" alt="drawing"/> |  |
| Kazan Express point (55.7503992494, 48.7481202714) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_kazanexpress.png" width="512" alt="drawing"/> |
| Technopark office parking (55.7517506178, 48.7506804476) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_technopark.png" width="512" alt="drawing"/> |
| Building (55.7531869667, 48.7510098844) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_building.png" width="512" alt="drawing"/> |

The first 3 landing stations are used for deliveries. The last one is used for a regular inspection of the buildings.
