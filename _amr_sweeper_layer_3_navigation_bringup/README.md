# amr_sweeper_layer_3_navigation_bringup

```bash
ros2 launch amr_sweeper_layer_3_navigation_bringup amr_sweeper_layer_3_navigation_bringup.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_localization`
- `amr_sweeper_mapping`
- `amr_sweeper_waypoint_follower`
- `amr_sweeper_layer_1_hardware_bringup`
- `amr_sweeper_layer_2_controllers_bringup`

## Purpose
This package is the main entrypoint for the AMR Sweeper navigation layer.

## Main Launch File
`launch/amr_sweeper_layer_3_navigation_bringup.launch.py`

## Available Launch Files
- `amr_sweeper_layer_3_navigation_bringup.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `use_amr_sweeper_localization`: default `true`
- `use_amr_sweeper_mapping`: default `true`
- `use_amr_sweeper_waypoint_follower`: default `true`
- `mission_execution_directory`: default `""`
- `auto_start_mission`: default `false`

## Overview
`amr_sweeper_layer_3_navigation_bringup` starts localization, mapping, and the Nav2/waypoint package together so the AMR Sweeper can estimate its pose, build the runtime mapping stack, and generate motion commands. It is the top-layer bringup for the real robot navigation stack.

## Notes
- Use this package after the required layer 1 and layer 2 packages are already available.
- The navigation commands produced here flow down through layer 2 and into the layer 1 drive interfaces.
- With the default namespace, localization and Nav2 consume `/amr_sweeper/gnss/navsat`, `/amr_sweeper/drive_controller/odom`, and `/amr_sweeper/depth_camera/scan`; the localization IMU source is configured via `imu.topic` in `amr_sweeper_localization/config/amr_sweeper_localization.yaml` and defaults to `imu/data_raw`, which resolves to `/amr_sweeper/imu/data_raw`.
- The localization stack consumes `/amr_sweeper/gnss/navsat` directly from the local GNSS node.
- The waypoint follower bringup rewrites `config/nav2_params.yaml` at launch time so relative sensor topics and the selected map file remain correct under both the default namespace and custom robot roots.
- Mission auto-start is off by default here and should only be enabled by FSM `RUNNING` profiles that hand in a mission-specific execution directory.
