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
- Supports mission route GeoJSON features tagged with `properties.coordinate_frame: "odom"` or `"local"` so small built-in sweep patterns can run directly in the local navigation frame.
- Supervises the selected SLAM backend and the gaussian-world builder during runtime.
- Reads the exact scheduler-selected mission execution directory passed through the FSM RUNNING launch path and loads that folder's `execution_context.json` so the selected mission route, mission window, and mission output directory follow the active mission.
- Writes mission runtime outputs into a per-run subfolder under the selected mission folder.

## Runtime Structure
- `amr_sweeper_mapping_node.cpp/.hpp` contains both the Nav2 costmap plugin and the mapping coordinator node.
- `slam_node.cpp/.hpp` provides SLAM supervision and startup seeding for `slam_toolbox`.
- `gaussian_node.cpp/.hpp` builds the lightweight onboard 3D gaussian-world representation.

## Notes
- The runtime costmap alias is expected at `src/missions/global_costmap.yaml`.
- The runtime route alias is expected at `src/missions/active_mission_path.geojson`.
- The preferred runtime input is the exact scheduler-selected mission execution directory passed as the `mission_execution_directory` launch argument.
- A scheduler-written execution pointer at `src/missions/active_execution.json` remains available as a fallback for manual launches.
- The scheduler-prepared execution context is expected at `src/missions/<order_id>_<timestamp>/<execution_timestamp>/execution_context.json`.
- VDA5050 mission parsing and artifact generation now live in layer 0 inside `amr_sweeper_mission_builder`.
- The SLAM node and gaussian node are orchestration shells in this pass so backend selection stays flexible while we wire the package structure.
