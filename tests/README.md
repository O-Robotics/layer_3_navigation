# layer_3_navigation_tests

```bash
python3 src/layer_3_navigation/tests/waypoint_follower_test.py \
  --geojson src/layer_3_navigation/tests/Forskerparken_zigzag_path.geojson \
  --round-trips 1
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
`tests/waypoint_follower_test.py`

## Available Test Files
- `waypoint_follower_test.py`

## Test Arguments
- `--geojson`: required path to the GeoJSON route file
- `--namespace`: default `amr_sweeper`
- `--frame-id`: default `map`
- `--map-origin-easting`: optional UTM easting override for the local `map` origin
- `--map-origin-northing`: optional UTM northing override for the local `map` origin
- `--navsat-topic`: default `navsat`, used to derive the local `map` origin when no explicit UTM origin is provided
- `--navsat-timeout`: default `10.0`
- `--round-trips`: default `1`
- `--brush-linear`: default `1.0`
- `--brush-angular`: default `0.0`
- `--brush-publish-hz`: default `10.0`

## Overview
`waypoint_follower_test.py` reads the GeoJSON line passed with `--geojson`, converts the WGS84 coordinates into UTM, and then converts those UTM coordinates into the local Nav2 `map` frame. By default it derives the local `map` origin from a live `NavSatFix` sample on the configured `navsat` topic. If needed, you can still override the UTM origin explicitly with `--map-origin-easting` and `--map-origin-northing`. It sends the route to Nav2 as forward and reverse waypoint sequences. While the route is running, it continuously publishes brush commands on `cmd_vel_joy_brushes` so the layer 2 tool controller keeps both brushes turned on.

## Notes
- Run this after layer 1, layer 2, and layer 3 bringup are active on the robot.
- The test uses the Nav2 `follow_waypoints` action under the configured namespace.
- The test also publishes a `visualization_msgs/Marker` line on `/amr_sweeper/waypoint_test/route_marker` for the GeoJSON route and a `visualization_msgs/Marker` sphere on `/amr_sweeper/waypoint_test/next_waypoint` for the next commanded waypoint.
- When `--frame-id map` is used, the derived or supplied origin must match the local `map` frame used by Nav2. The script logs both the raw UTM route endpoints and the converted `map` endpoints to help verify the chosen origin.
- Set `--round-trips 0` to keep sweeping back and forth until interrupted.
