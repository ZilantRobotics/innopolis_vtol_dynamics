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

Testing and optimization of cargo delivery processes using unmanned aerial vehicles

<img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/delivery.png" alt="drawing"/>

| Plan | Flight log example | Approximate execution time |
|-|-|-|
| [delivery_from_kazanexpress_to_yard.plan](delivery_from_kazanexpress_to_yard.plan) (takeoff from KazanExpress, deliver package to the yard, come back and land) | [log16](https://review.px4.io/plot_app?log=c63a3a10-5f1f-4d51-97a9-cfa0970b9f3c) | 7 minutes |

### 2. Construction Inspection Detection Scenario

Development of the process of automatic regular inspection of construction sites using unmanned aerial vehicles

<img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/inspection.png" alt="drawing"/>

| Plan | Flight log example | Approximate execution time |
|-|-|-|
| [inspection_technopark.plan](inspection_technopark.plan) (takeoff from technopark landing station, inspect the building process, come back and land) | [log18](https://review.px4.io/plot_app?log=2087803f-a0ea-41c6-b322-eb8ef06ad82d) | 4 minutes |


## Additional scenarios

### 1. Ground infrastructure testing scenario:

Checking the interaction of unmanned aerial vehicles with ground infrastructure within the framework of air logistics tasks.

### 2. Fault Test Scenario

Evaluation of the behavior and response of unmanned aerial vehicles to various types of failures during flight.

## Key points

There are 4 landing stations:

| Position, Lat/Lon               | Picture |
| ------------------------- | ------- |
| In the yard (55.7487847875, 48.7430507069) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_yard.png" width="512" alt="drawing"/> |  |
| Kazan Express point (55.7503992494, 48.7481202714) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_kazanexpress.png" width="512" alt="drawing"/> |
| Technopark office parking (55.7517506178, 48.7506804476) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_technopark.png" width="512" alt="drawing"/> |
| Building (55.7531869667, 48.7510098844) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_building.png" width="512" alt="drawing"/> |

The first 3 landing stations are used for deliveries. The last one is used for a regular inspection of the buildings.
