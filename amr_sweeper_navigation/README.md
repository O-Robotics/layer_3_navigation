# amr_sweeper_navigation

```bash
ros2 launch amr_sweeper_navigation programmed_missions_navigation.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_localization`
- `amr_sweeper_sweeping_controller`
- `amr_sweeper_drive_controller`

## Purpose
This package hosts the real-robot Nav2 bringup for the AMR Sweeper and keeps the runtime navigation stack cleanly separated by mission class.

## Main Launch Files
- `launch/default_missions_navigation.launch.py`
- `launch/manual_missions_navigation.launch.py`
- `launch/programmed_missions_navigation.launch.py`

## Internal Launch File
- `launch/navigation_launch.py`

## Configuration Files
- `config/default_missions_navigation.yaml`
- `config/manual_missions_navigation.yaml`
- `config/programmed_missions_navigation.yaml`

## Built-In Missions
- `missions/default_missions/3x3Sweep`
- `missions/default_missions/SpotSweep`
- `missions/manual_missions/RecordMap`
- `missions/manual_missions/Teleop`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `params_file`: default depends on the selected mission-class launch
- `mission_static_costmap_yaml`: default empty; layer 3 bringup injects the exact mission run costmap from `execution_context.json` when needed
- `autostart`: default `true`
- `use_respawn`: default `false`
- `log_level`: default `info`

## Overview
`amr_sweeper_navigation` contains the shared Nav2 node-launch logic plus three mission-class-specific entrypoints:

- `default_missions_navigation.launch.py`: built-in local missions such as `3x3Sweep` and `SpotSweep`
- `manual_missions_navigation.launch.py`: manual mission workflows such as `RecordMap` and `Teleop`
- `programmed_missions_navigation.launch.py`: scheduled/programmed routed missions

Each public launch file selects a different Nav2 parameter file while reusing the same internal `navigation_launch.py` node graph. This keeps mission policy out of the generic Nav2 launcher and lets layer 0 / layer 3 choose the right navigation stack from the mission classification.

The package also owns the built-in mission files directly:
- `missions/default_missions/`: local sweep templates such as `3x3Sweep` and `SpotSweep`
- `missions/manual_missions/`: operator-driven workflows such as `RecordMap` and `Teleop`

`default_missions_navigation.yaml` is intentionally odom-only and does not depend on map-frame planning. It is tuned for the short built-in sweep patterns that start from the robot's current pose, while still consuming mapping-published costmaps when mapping is enabled.

## Notes
- Depends on `amr_sweeper_localization` for robot pose estimation.
- Publishes navigation wheel commands that are expected by the layer 2 command chain.
- Nav2 now consumes `mapping/static_costmap` for global planning and builds its rolling local costmap directly from live sensor observations.
- `mapping/waypoint_path` is visualization-only; routed mission execution reaches Nav2 through the `navigate_through_poses` action server.
