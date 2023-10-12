# CI tests (without 3D graphics)

All tests expected that the initial position is configured in [sim_params.yaml](uav_dynamics/inno_vtol_dynamics/config/sim_params.yaml) as shown below:

```yaml
lat_ref : 55.7531869667
lon_ref : 48.7510098844
alt_ref : -6.5
```

## 1. Takeoff And Land

This is the simplest possible test scenario: take off, wait a few seconds, and land. It is the fastest scenario and it is intended to be triggered on every commit as part of CI.

```
./missions/ci/takeoff_and_land.sh
```

Flight logs:
- [log10](https://review.px4.io/plot_app?log=9aeb4932-991a-4856-b292-187d6a9f37af), 30 seconds

## 2. Square flight

Simple quadcopter flight test.

```
./missions/ci/square.sh
```

## 3. VTOL flight to Sviyazhsk

This is the longest test scenario. It takes ~ 60 minutes. It is dedicated for testing the stability of the Cyphal HITL communication.

```
./scripts/sviyazhsk_vtol.sh
```

Flight logs:
- sviyazhsk_vtol_v0.1.plan: [log5](https://review.px4.io/plot_app?log=651ad2b4-149c-4b6c-8dd5-70a0c92c3617), 60 minutes
- [sviyazhsk_vtol_v0.2.plan](sviyazhsk_vtol.plan): [log28](https://review.px4.io/plot_app?log=cb1ae54d-cc3c-4b60-9788-f1e9f262ecfe), 13:02 minutes
- [sviyazhsk_vtol_v0.2.plan](sviyazhsk_vtol.plan): [log1](https://logs.px4.io/plot_app?log=fac98022-1076-45ee-bdc1-46cdd8d2b594), 8:20 minutes
