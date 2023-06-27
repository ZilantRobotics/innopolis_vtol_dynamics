# Test scenarios

Here we have a few test scenarios:

## HITL test scenarios

### 1. The simplest Takeoff And Land scenario ([takeoff_and_land.plan](takeoff_and_land.plan))

This is the simplest possible test scenario: take off, wait a few seconds, and land. It is the fastest scenario and it is intended to be triggered on every commit as part of CI.

Flight log example: [log10](https://review.px4.io/plot_app?log=9aeb4932-991a-4856-b292-187d6a9f37af).

### 2. The longest flight scenario ([sviyazhsk_60_minutes.plan](sviyazhsk_60_minutes.plan))

This is the longest test scenario. It takes ~ 60 minutes. It is dedicated for testing the stability of the Cyphal HITL communication.

Flight log example: [log5](https://review.px4.io/plot_app?log=8db32ee4-cae0-4594-acc1-902cab07cd94).

## 3D part test scenarios

### 1. Last Mile Aerologistics Test Scenario

Testing and optimization of cargo delivery processes using unmanned aerial vehicles

<img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/delivery.png" alt="drawing"/>

Flight log example (takeoff from KazanExpress, deliver package to the yard, come back and land): [log16](https://review.px4.io/plot_app?log=c63a3a10-5f1f-4d51-97a9-cfa0970b9f3c).

### 2. Construction Inspection Detection Scenario

Development of the process of automatic regular inspection of construction sites using unmanned aerial vehicles

<img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/inspection.png" alt="drawing"/>

## Additional scenarios

### 1. Ground infrastructure testing scenario:

Checking the interaction of unmanned aerial vehicles with ground infrastructure within the framework of air logistics tasks.

There are 3 landing station:

| Position, Lat/Lon               | Picture |
| ------------------------- | ------- |
| In the yard (55.7487847875036, 48.7430507069631) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_yard.png" width="512" alt="drawing"/> |  |
| Kazan Express point (55.7503992494391, 48.7481202714611) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_kazanexpress.png" width="512" alt="drawing"/> |
| Technopark office parking (55.7517506178689, 48.7506804476454) | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_technopark.png" width="512" alt="drawing"/> |

### 2. Fault Test Scenario

Evaluation of the behavior and response of unmanned aerial vehicles to various types of failures during flight.
