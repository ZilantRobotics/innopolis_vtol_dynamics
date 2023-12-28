# Test scenarios

UAV HITL simulator supports multiple types of test scenarios.

| Scenario | Description |
| -------- | ----------- |
| 1. Simple HITL CI </br> [ci/README.md](ci/README.md) | The simplest scenarios intended to be used as part of CI process. |
| 2. Delivery </br> [delivery/README.md](delivery/README.md) | Last Mile Aerologistics Scenarios are indended to be used with 3D-simulator |
| 3. Inspection </br> [- Quadcopter](inspection/README.md) </br> [- VTOL Quadplane](inspection_vtol/README.md) | Construction Inspection Detection Scenarios are indended to be used with 3D-simulator. |
| 4. Cartography | Cartography Scenarios are indended to be used with 3D-simulator. |
| 5. Ground Infrastructure | Checking the interaction of unmanned aerial vehicles with ground infrastructure within the framework of air logistics tasks. These scenarios might be a part of the previous scenarios. |
| 6. Fault </br> [fault/README.md](fault/README.md) | Evaluation of the behavior and response of UAV to various types of failures during flight. These scenarios might be a part of the previous scenarios. |
| 7. Reduncancy </br> [redundancy/README.md](redundancy/README.md) | These scenarios might be a part of the previous scenarios. |

### Key points

There are 4 landing stations:

| Position, Lat/Lon               | Picture |
| ------------------------- | ------- |
| In the yard: </br> lat_ref: 55.7487847875 </br> lon_ref: 48.7430507069 | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_yard.png" width="512" alt="drawing"/> |  |
| Kazan Express point: </br> lat_ref: 55.7503992494 </br> lon_ref: 48.7481202714 | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_kazanexpress.png" width="512" alt="drawing"/> |
| Technopark office parking: </br> lat_ref: 55.7517506178 </br> lon_ref: 48.7506804476 | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_technopark.png" width="512" alt="drawing"/> |
| Building: </br> lat_ref: 55.7531869667 </br> lon_ref: 48.7510098844 | <img src="https://raw.githubusercontent.com/RaccoonlabDev/innopolis_vtol_dynamics/docs/assets/landing_station_building.png" width="512" alt="drawing"/> |

The first 3 landing stations are used for deliveries. The last one is used for a regular inspection of the buildings.
