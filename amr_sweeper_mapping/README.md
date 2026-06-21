# amr_sweeper_mapping

```bash
ros2 launch amr_sweeper_mapping amr_sweeper_mapping.launch.py
```

## Purpose
This package provides the runtime mapping layer for AMR Sweeper, including global occupancy-map building, gaussian-map orchestration, and the map-to-odom alignment node.

## Nodes
- `gaussian_node`
- `mapping_node`

## Current Capabilities
- Converts generated mission routes through FusionCore `/fromLL` and executes them with Nav2 `follow_waypoints`.
- Publishes the Nav2-facing runtime global costmap on `mapping/global_costmap`.
- Supports mission route GeoJSON features tagged with `properties.coordinate_frame: "odom"` or `"local"` so small built-in sweep patterns can run directly in the local navigation frame.
- Rewrites builtin local-pattern run-folder route artifacts into `odom` once they are anchored at mission start, so the saved planned path can be compared directly against `<mission_id>_<run_timestamp>_path_actual.geojson`.
- Writes synchronized runtime trace artifacts where each `<mission_id>_<run_timestamp>_path_actual.geojson` odom sample has a matching raw-`gnss/navsat` point in `<mission_id>_<run_timestamp>_path_navsat.geojson`, along with per-sample timestamps and odom yaw values.
- Supports mission JSON files with `execution_mode: "manual_mapping"` so a mission such as `RecordMap` can keep SLAM and gaussian mapping active while an operator manually drives the robot.
- Supervises the gaussian-world builder during runtime while the in-package map builder maintains the 2D occupancy map.
- Reads the exact scheduler-selected mission execution directory passed through the FSM RUNNING launch path and loads that folder's stamped `*_context.json` file so the selected mission route, mission window, and mission output directory follow the active mission.
- Writes mission runtime outputs into a per-run subfolder under the selected mission folder.
- Records the traveled path into `<mission_id>_<run_timestamp>_path_actual.geojson` inside the active execution folder while layer 3 is running.
- Calls `amr_sweeper_mission_executor/end_mission` when an autonomous routed mission finishes or aborts so the run is finalized and the FSM returns to `IDLING`.

## Runtime Structure
- `mapping_node.cpp/.hpp` contains the `mapping_node` coordinator and the in-package occupancy-grid map builder.
- `map_pose_node.cpp/.hpp` publishes `map -> odom` by combining scan-to-map matching with GNSS and IMU consistency terms so Nav2 can consume the global map in the `map` frame without shifting that map at runtime. In mission runs it should correlate against the static mission-folder costmap artifact, not the run-folder live-updated copy.
- `gaussian_node.cpp/.hpp` builds the lightweight onboard 3D gaussian-world representation.

## Notes
- REP-105 ownership in this workspace is now: FusionCore publishes `odom -> base_footprint`, and `map_pose_node` inside `amr_sweeper_mapping` publishes the `map -> odom` alignment.
- The mapping coordinator publishes the Nav2-facing runtime global map on `mapping/global_costmap`.
- Nav2 now owns the rolling local costmap directly from live sensor observations; `amr_sweeper_mapping` no longer publishes a local costmap topic.
- The planned mission waypoints loaded from the active run folder's copied `<mission_id>_<run_timestamp>_path_planned.geojson` are visualized on `mapping/waypoint_path`.
- When `saved_costmap_yaml` is configured, the coordinator waits for a short GNSS/IMU stabilization window and then locks that georeferenced YAML/PGM artifact once into the metric `map` frame before live scans extend or overwrite it.
- During mission runs the mapping launch now keeps these roles separate: `map_pose_node` reads the mission-folder reference costmap, while `mapping_node` persists live scan updates into the run-folder costmap copy.
- Runtime costmap artifacts now embed georeference metadata in the YAML file so the exported YAML/PGM pair can be related back to the real world outside the robot.
- The required runtime input is the exact scheduler-selected mission execution directory passed as the `mission_execution_directory` launch argument.
- The mission route and mission costmap must come from that execution folder's stamped `*_context.json` file, not from shared top-level alias files.
- `auto_start_mission` defaults to `false`; FSM `RUNNING` profiles are expected to enable it explicitly when mission execution is intended.
- Standalone launch is still supported. When no mission-specific costmap yaml is provided, the package skips artifact seeding and continues building the runtime global map from live inputs instead of failing startup.
- Runtime artifacts such as the `mapping` section inside `<mission_id>_<run_timestamp>_context.json`, `<mission_id>_<run_timestamp>_path_actual.geojson`, and gaussian outputs are only written when a mission execution folder is resolved from `mission_execution_directory`.
- The scheduler-prepared execution context is expected at `missions/logs/<mission_id>/<execution_timestamp>/<mission_id>_<run_timestamp>_context.json`.
- Gaussian outputs are expected under the execution folder's `gaussian/` subdirectory.
- VDA5050 mission parsing and artifact generation now live in layer 0 inside `amr_sweeper_vda5050_parser`.
- `mapping_node` now owns startup costmap lock-in and runtime artifact persistence directly, so there is no separate `slam_node` process in the launch graph.
