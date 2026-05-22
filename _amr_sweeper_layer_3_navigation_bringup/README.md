# amr_sweeper_layer_3_navigation_bringup

```bash
ros2 launch amr_sweeper_layer_3_navigation_bringup amr_sweeper_layer_3_navigation_bringup.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_localization`
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
- `use_amr_sweeper_waypoint_follower`: default `true`

## Overview
`amr_sweeper_layer_3_navigation_bringup` starts the localization package and the Nav2/waypoint package together so the AMR Sweeper can estimate its pose and generate motion commands. It is the top-layer bringup for the real robot navigation stack.

## Notes
- Use this package after the required layer 1 and layer 2 packages are already available.
- The navigation commands produced here flow down through layer 2 and into the layer 1 drive interfaces.
- With the default namespace, localization and Nav2 consume `/amr_sweeper/imu/data_raw`, `/amr_sweeper/gnss/navsat`, `/amr_sweeper/diff_cont/odom`, and `/amr_sweeper/depth_camera/scan`.
- The localization stack republishes GNSS fixes onto `/amr_sweeper/gnss/navsat_reliable` before FusionCore consumes them, which avoids reliable-vs-best-effort DDS warnings on the raw GNSS topic.
- The waypoint follower bringup rewrites `config/nav2_params.yaml` at launch time so relative sensor topics and the selected map file remain correct under both the default namespace and custom robot roots.
