# amr_sweeper_layer_3_navigation_bringup

```bash
ros2 launch amr_sweeper_layer_3_navigation_bringup amr_sweeper_layer_3_navigation_bringup.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_localization`
- `amr_sweeper_mapping`
- `amr_sweeper_navigation`
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
- `use_amr_sweeper_navigation`: default `true`
- `mission_execution_directory`: default `""`
- `auto_start_mission`: default `false`

## Overview
`amr_sweeper_layer_3_navigation_bringup` starts localization, mapping, and the mission-class-specific Nav2 package together so the AMR Sweeper can estimate its pose, build the runtime mapping stack, and generate motion commands. It is the top-layer bringup for the real robot navigation stack.

## Notes
- Use this package after the required layer 1 and layer 2 packages are already available.
- The navigation commands produced here flow down through layer 2 and into the layer 1 drive interfaces.
- With the default namespace, localization consumes `/amr_sweeper/gnss/navsat` and `/amr_sweeper/drive_controller/odom`, runs FusionCore natively on `base_link`, publishes the internal body-frame topic `/amr_sweeper/localization/odometry_body`, and projects it back to the existing planar `/amr_sweeper/localization/odometry_fused` topic plus `odom -> base_footprint` TF. `amr_sweeper_mapping` consumes `/amr_sweeper/depth_camera/scan`, and Nav2 consumes the mapping-published `/amr_sweeper/mapping/global_costmap` while building its rolling local costmap directly from live sensor observations; the localization IMU source is configured via `imu.topic` in `amr_sweeper_localization/config/amr_sweeper_localization.yaml` and defaults to `imu/data_raw`, which resolves to `/amr_sweeper/imu/data_raw`.
- The localization stack consumes `/amr_sweeper/gnss/navsat` directly from the local GNSS node.
- `amr_sweeper_navigation` selects `default_missions_navigation.launch.py`, `manual_missions_navigation.launch.py`, or `programmed_missions_navigation.launch.py` from the active mission context. Built-in local pattern missions still plan in `odom`, but when mapping is requested they now keep `amr_sweeper_mapping` active so Nav2 can consume `mapping/global_costmap` while owning the local rolling costmap directly.
- Mission auto-start is off by default here and should only be enabled by FSM `RUNNING` profiles that hand in a mission-specific execution directory.
- Mission-triggered rosbag recording is owned by `amr_sweeper_bringup` and triggered by `amr_sweeper_mission_executor`, so Teleop and other profiles that skip layer 3 can still capture bags into the mission run's `artifacts/rosbag` folder.
