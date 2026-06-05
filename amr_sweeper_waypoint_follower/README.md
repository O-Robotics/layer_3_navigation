# amr_sweeper_waypoint_follower

```bash
ros2 launch amr_sweeper_waypoint_follower bringup_launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_localization`
- `amr_sweeper_sweeping_controller`
- `amr_sweeper_wheel_controller`

## Purpose
This package hosts the real-robot Nav2 and waypoint-following setup for the AMR Sweeper.

## Main Launch File
`launch/bringup_launch.py`

## Available Launch Files
- `bringup_launch.py`
- `navigation_launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `map`: default `<package_share>/maps/map.yaml`
- `use_sim_time`: default `false`
- `params_file`: default `<package_share>/config/nav2_params.yaml`
- `mission_costmap_yaml`: default empty; layer 3 bringup must inject the exact mission run costmap from `execution_context.json`
- `autostart`: default `true`
- `use_respawn`: default `false`
- `log_level`: default `info`

## Overview
`amr_sweeper_waypoint_follower` contains the Nav2 bringup files, navigation parameters, and map assets used by the AMR Sweeper. It is the package that turns localization and planning into wheel-command output for the lower layers. The launch path rewrites `config/nav2_params.yaml` at runtime so the selected namespace, map path, and clock mode stay aligned whether the package is launched on its own or through layer 3 bringup.

## Notes
- Depends on `amr_sweeper_localization` for robot pose estimation.
- Publishes navigation wheel commands that are expected by the layer 2 command chain.
- The Nav2 obstacle layers subscribe to `depth_camera/scan`, which resolves to `/amr_sweeper/depth_camera/scan` under the default namespace and follows any custom robot root automatically.
- Intended for the trimmed real-robot stack only.
