# layer_3_navigation_tests

```bash
python3 src/layer_3_navigation/tests/alf_driveway_back_and_forth_test.py \
  --round-trips 1 \
  --map-origin-easting <utm_easting_of_map_origin> \
  --map-origin-northing <utm_northing_of_map_origin>
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_localization`
- `amr_sweeper_waypoint_follower`
- `amr_sweeper_tool_controller`
- `amr_sweeper_layer_1_hardware_bringup`
- `amr_sweeper_layer_2_controllers_bringup`

## Purpose
This folder contains ad hoc navigation tests for the real AMR Sweeper stack.

## Main Test File
`tests/alf_driveway_back_and_forth_test.py`

## Available Test Files
- `alf_driveway_back_and_forth_test.py`

## Test Arguments
- `--geojson`: default `src/layer_3_navigation/tests/Alf_Driveway.geojson`
- `--namespace`: default `amr_sweeper`
- `--frame-id`: default `map`
- `--map-origin-easting`: required when `--frame-id map` is used
- `--map-origin-northing`: required when `--frame-id map` is used
- `--round-trips`: default `1`
- `--brush-linear`: default `0.5`
- `--brush-angular`: default `0.0`
- `--brush-publish-hz`: default `10.0`

## Overview
`alf_driveway_back_and_forth_test.py` reads the `Alf_Driveway.geojson` line, converts the WGS84 coordinates into UTM, and then converts those UTM coordinates into the local Nav2 `map` frame using the provided map-origin easting and northing. It sends the route to Nav2 as forward and reverse waypoint sequences. While the route is running, it continuously publishes brush commands on `cmd_vel_joy_brushes` so the layer 2 tool controller keeps both brushes turned on.

## Notes
- Run this after layer 1, layer 2, and layer 3 bringup are active on the robot.
- The test uses the Nav2 `follow_waypoints` action under the configured namespace.
- When `--frame-id map` is used, the map origin must match the local `map` frame used by Nav2. The script now logs both the raw UTM route endpoints and the converted `map` endpoints to help verify the chosen origin.
- Set `--round-trips 0` to keep sweeping back and forth until interrupted.
