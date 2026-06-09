# amr_sweeper_mapping

```bash
ros2 launch amr_sweeper_mapping amr_sweeper_mapping.launch.py
```

## Purpose
This package provides the runtime mapping layer for AMR Sweeper, including artifact-backed Nav2 costmap integration, SLAM orchestration, gaussian-map orchestration, and a mapping coordinator node.

## Nodes
- `amr_sweeper_slam_node`
- `amr_sweeper_gaussian_node`
- `amr_sweeper_mapping_node`

## Current Capabilities
- Loads generated costmap artifacts from `/missions` into a Nav2 global costmap plugin.
- Converts generated mission routes through FusionCore `/fromLL` and executes them with Nav2 `follow_waypoints`.
- Publishes REP-105 `earth -> map` from the FusionCore GNSS datum and the live SLAM `map -> odom` correction, so the `map` frame stays georeferenced while Nav2 continues to plan in `map` and control in `odom`.
- Supports mission route GeoJSON features tagged with `properties.coordinate_frame: "odom"` or `"local"` so small built-in sweep patterns can run directly in the local navigation frame.
- Supports mission JSON files with `execution_mode: "manual_mapping"` so a mission such as `RecordMap` can keep SLAM and gaussian mapping active while an operator manually drives the robot.
- Supervises the selected SLAM backend and the gaussian-world builder during runtime.
- Reads the exact scheduler-selected mission execution directory passed through the FSM RUNNING launch path and loads that folder's `execution_context.json` so the selected mission route, mission window, and mission output directory follow the active mission.
- Writes mission runtime outputs into a per-run subfolder under the selected mission folder.
- Records the traveled path into `actual_path.geojson` inside the active execution folder while layer 3 is running.
- Calls `amr_sweeper_mission_executor/end_mission` when an autonomous routed mission finishes or aborts so the run is finalized and the FSM returns to `IDLING`.

## Runtime Structure
- `amr_sweeper_mapping_node.cpp/.hpp` contains both the Nav2 costmap plugin and the mapping coordinator node.
- `slam_node.cpp/.hpp` provides SLAM supervision and startup seeding for `slam_toolbox`.
- `gaussian_node.cpp/.hpp` builds the lightweight onboard 3D gaussian-world representation.

## Notes
- REP-105 ownership in this workspace is now: FusionCore publishes `odom -> base_footprint`, slam_toolbox publishes `map -> odom`, and `amr_sweeper_mapping` publishes `earth -> map`.
- The REP-105 georeference path assumes FusionCore is configured for a local ENU frame at the selected GNSS datum, which is the current workspace default (`output.convert_to_enu_at_reference: true`).
- The required runtime input is the exact scheduler-selected mission execution directory passed as the `mission_execution_directory` launch argument.
- The mission route and mission costmap must come from that execution folder's `execution_context.json`, not from shared top-level alias files.
- `auto_start_mission` defaults to `false`; FSM `RUNNING` profiles are expected to enable it explicitly when mission execution is intended.
- Standalone launch is still supported. When no mission-specific costmap yaml is provided, the package logs a warning and keeps the GeoJSON costmap layer inactive instead of failing startup.
- Runtime artifacts such as `mapping_session.json`, `actual_path.geojson`, and gaussian outputs are only written when a mission execution folder is resolved from `mission_execution_directory`.
- The scheduler-prepared execution context is expected at `missions/logs/<mission_id>/<execution_timestamp>/execution_context.json`.
- Gaussian outputs are expected under the execution folder's `gaussian/` subdirectory.
- VDA5050 mission parsing and artifact generation now live in layer 0 inside `amr_sweeper_vda5050_parser`.
- The SLAM node and gaussian node are orchestration shells in this pass so backend selection stays flexible while we wire the package structure.
