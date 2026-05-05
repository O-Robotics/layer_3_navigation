# amr_sweeper_waypoint_follower

`ros2 launch amr_sweeper_waypoint_follower bringup_launch.py`

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_localization`
- `amr_sweeper_twist_mux`
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
- `autostart`: default `true`
- `use_respawn`: default `False`
- `log_level`: default `info`

## Overview
`amr_sweeper_waypoint_follower` contains the Nav2 bringup files, navigation parameters, and map assets used by the AMR Sweeper. It is the package that turns localization and planning into wheel-command output for the lower layers.

## Notes
- Depends on `amr_sweeper_localization` for robot pose estimation.
- Publishes navigation wheel commands that are expected by the layer 2 command chain.
- Intended for the trimmed real-robot stack only.
