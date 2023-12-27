# Inspection VTOL

> Add an image example from VTOL flight here

## Missions plans

| № | Plan | Estimated distance, m | Estimated time | Comment |
| - | ---- | --------------------- | -------------- | ------- |
| 1 | technopark_structure_scan.plan   | 7528  | 00:09:25 | Structure scan of the northest techonopark |
| 2 | technopark_survey_half_town.plan | 23956 | 00:27:46 | Survey of all technoparks in the town |
| 3 | technopark_survey_full_town.plan | 39128 | 00:44:38 | Survey of all the town |

## Usage

1. Set initial coordinates in [sim_params.yaml](uav_dynamics/inno_vtol_dynamics/config/sim_params.yaml):

```bash
code uav_dynamics/inno_vtol_dynamics/config/sim_params.yaml
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
./scripts/test_scenario.sh --mission missions/inspection_vtol/<.plan>
```

## Flight logs

| Date and log number | Scenario | Duration | Comment |
| ------------------- | -------- | -------- | ------- |
| [Dec 27, 2023: log26](https://review.px4.io/plot_app?log=78846d3d-0ab9-49b8-b4e3-0cffb8b3c890) | [Technopark survey full town](technopark_structure_scan.plan) | 29:55 | Stable long flight without warnings. |
| [Dec 27, 2023: log25](https://review.px4.io/plot_app?log=e07e0d5d-a6cc-4ca9-90ec-b36ad844c625) | [Technopark survey half town](technopark_structure_scan.plan) | 20:38 | Stable long flight without warnings, but survey quality is not good satisfying. |
| [Dec 27, 2023: log23](https://review.px4.io/plot_app?log=ebfe8537-99a3-4aeb-9c53-a93f10b1410a) | [Technopark structure scan](technopark_structure_scan.plan) | 7:29 | Flight with geofences. It has a few warnins, but it was expected in this test flight. |
| [Dec 27, 2023: log200](https://review.px4.io/plot_app?log=bdbc79d1-9cbf-4007-b2e2-60cfafbb9ee7) | [Technopark structure scan](technopark_structure_scan.plan) | 8:09 | Flight without geofences. No warnings.
