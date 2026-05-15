# layer_3_navigation_tests

```bash
python3 src/layer_3_navigation/tests/waypoint_follower_test.py \
  --geojson src/layer_3_navigation/tests/Ecopark_line.geojson \
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
- `--frame-id`: default `odom`
- `--fromll-service`: default `/fromLL`
- `--fromll-timeout`: default `10.0`
- `--round-trips`: default `1`
- `--tool-linear`: default `0.1`
- `--tool-angular`: default `0.0`
- `--tool-publish-hz`: default `10.0`
- `--max-segments-per-goal`: default `4`

## Overview
`waypoint_follower_test.py` reads the GeoJSON line passed with `--geojson` and converts each WGS84 coordinate through FusionCore's `/fromLL` service into the same local navigation frame used by Nav2. This keeps the waypoint conversion aligned with FusionCore's active GNSS reference when `reference.use_first_fix: true` is enabled. It sends the route to Nav2 as forward and reverse waypoint sequences in smaller follow-waypoints chunks so the rolling-window planner only has to solve a short stretch of the line at a time. While the route is running, it continuously publishes tool commands on `cmd_vel_joy_tools` so the layer 2 tool controller keeps both tool motors turned on.

## Notes
- Run this after layer 1, layer 2, and layer 3 bringup are active on the robot.
- Wait until FusionCore is active and has accepted a GNSS fix before starting the test, otherwise `/fromLL` cannot convert the route yet.
- The test uses the Nav2 `follow_waypoints` action under the configured namespace.
- `--max-segments-per-goal` controls how many connected GeoJSON line segments are sent in each Nav2 goal. The default of `4` keeps each local plan short enough to fit comfortably inside a rolling local world model.
- The test also publishes a `visualization_msgs/Marker` line on `/amr_sweeper/waypoint_test/route_marker` for the GeoJSON route and a `visualization_msgs/Marker` sphere on `/amr_sweeper/waypoint_test/next_waypoint` for the next commanded waypoint when using the default namespace.
- Use `--frame-id odom` unless your Nav2 stack is configured to consume a different FusionCore-aligned local frame.
- Set `--round-trips 0` to keep sweeping back and forth until interrupted.
