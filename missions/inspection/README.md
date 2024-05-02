# Inspection Quadcopter

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/inspection.png" alt="drawing"/>

## Usage

1. Set initial coordinates in [sim_params.yaml](uav_dynamics/uav_hitl_dynamics/config/sim_params.yaml):

```bash
code uav_dynamics/uav_hitl_dynamics/config/sim_params.yaml
```

```yaml
lat_ref : 55.7531869667
lon_ref : 48.7510098844
alt_ref : -6.5
```

2. In first terminal build docker image and run container with force config :

```
./scripts/docker.sh b
./scripts/docker.sh cq --force
```

4. In second terminal run test scenario:  

```
./scripts/test_scenario.sh --mission missions/inspection/technopark.plan
```

## Flight logs

- [Jun 28, 2023: log18](https://review.px4.io/plot_app?log=2087803f-a0ea-41c6-b322-eb8ef06ad82d), 3:37
- [Jun 28, 2023: log19](https://review.px4.io/plot_app?log=03ceb5fd-3eb6-4f11-a743-4a65a1246c71), 3:37
- [Oct 12, 2023: log25](https://review.px4.io/plot_app?log=0f0ff42d-05d4-4fe3-bd40-d18ee2228781), 3:38
