# Delivery

Testing and optimization of cargo delivery processes using unmanned aerial vehicles.

<img src="https://github.com/ZilantRobotics/innopolis_vtol_dynamics/wiki/assets/delivery.png" alt="drawing"/>

## Usage

1. Set initial coordinates in [sim_params.yaml](uav_dynamics/uav_hitl_dynamics/config/sim_params.yaml):

```yaml
lat_ref : 55.7503992494
lon_ref : 48.7481202714
alt_ref : 0.0
```

2. In first terminal build docker image and run container with force config :

```
./scripts/docker.sh b
./scripts/docker.sh cq --force
```

4. In second terminal run test scenario:  

```
./scripts/test_scenario.sh --mission missions/delivery/<plan_name>.plan
```

## Flight logs

- Delivery from KazanExpress to the Technopark v0.1: [log22](https://review.px4.io/plot_app?log=3bea8e60-9f15-47ef-8954-be7cf0ae5e65), 6 minutes
- Delivery from KazanExpress to the Yard v0.1: [log16](https://review.px4.io/plot_app?log=c63a3a10-5f1f-4d51-97a9-cfa0970b9f3c), 7 minutes
- Delivery from KazanExpress to the Technopark v0.2: [log23](https://review.px4.io/plot_app?log=1c707f3e-ae76-49d5-9914-5abc0a5c5589), 4 minutes
