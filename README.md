# layer_3_navigation

```
ros2 launch amr_sweeper_layer_3_navigation_bringup amr_sweeper_layer_3_navigation_bringup.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_layer_3_navigation_bringup`
- `amr_sweeper_localization`
- `amr_sweeper_mapping`
- `amr_sweeper_default_missions`
- `amr_sweeper_waypoint_follower`
- `amr_sweeper_layer_1_hardware_bringup`
- `amr_sweeper_layer_2_controllers_bringup`

## Purpose
This repository is the navigation layer for the AMR Sweeper. It contains localization, Nav2 bringup, and waypoint-execution tools that use the hardware and controller layers to move the physical robot.

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `use_amr_sweeper_localization`: default `true`
- `use_amr_sweeper_mapping`: default `true`
- `use_amr_sweeper_waypoint_follower`: default `true`
- `execution_pointer_file`: default `active_execution.json`
- `mission_execution_directory`: default `""`

## Overview
Layer 3 consumes wheel odometry, GNSS, IMU, transforms, and controller topics from the lower layers. It is responsible for estimating robot pose, hosting the Nav2 stack, and publishing navigation commands that flow down into the layer 2 controller chain. Under the default namespace, it consumes `/amr_sweeper/imu/data_raw`, `/amr_sweeper/gnss/navsat`, `/amr_sweeper/diff_cont/odom`, and `/amr_sweeper/depth_camera/scan`.

The repository also ships `amr_sweeper_default_missions`, which owns the built-in manual mission templates so the runtime `/missions` directory can be reserved for mission history and schedule state.

## Notes
- The default command launches the full layer 3 navigation bringup package.
- Layer 3 depends on a running layer 1 and layer 2 stack.
- This layer is configured for the trimmed real-robot stack only.
- The localization launch now consumes `/amr_sweeper/gnss/navsat` directly because the local GNSS node publishes a compatible `NavSatFix` stream without the old bridge workaround.
- Nav2 runtime parameters live in `amr_sweeper_waypoint_follower/config/nav2_params.yaml`, and package-owned sensor topics are kept relative so the same files work under custom robot namespaces.
