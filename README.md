# layer_3_navigation

```
ros2 launch amr_sweeper_layer_3_navigation_bringup amr_sweeper_layer_3_navigation_bringup.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_layer_3_navigation_bringup`
- `amr_sweeper_localization`
- `amr_sweeper_mapping`
- `amr_sweeper_navigation`
- `amr_sweeper_layer_1_hardware_bringup`
- `amr_sweeper_layer_2_controllers_bringup`

## Purpose
This repository is the navigation layer for the AMR Sweeper. It contains localization, Nav2 bringup, and waypoint-execution tools that use the hardware and controller layers to move the physical robot.

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `use_amr_sweeper_localization`: default `true`
- `use_amr_sweeper_visual_odometry`: default `false`
- `use_amr_sweeper_mapping`: default `true`
- `use_amr_sweeper_navigation`: default `true`
- `mission_execution_directory`: default `""`
- `auto_start_mission`: default `false`

## Overview
Layer 3 consumes wheel odometry, GNSS, IMU, transforms, and controller topics from the lower layers. `amr_sweeper_localization` now runs FusionCore natively on `base_link`, publishing the internal body-frame topic `/localization/odometry_body`, then projects that estimate back to the public planar `/localization/odometry_fused` topic while owning `odom -> base_footprint`. `amr_sweeper_mapping` consumes `/localization/odometry_fused`, `/gnss/navsat`, `/imu/data_heading`, and `/depth_camera/scan`, publishes `map -> odom`, and publishes the georeferenced `mapping/static_costmap`. `amr_sweeper_navigation` then consumes `/localization/odometry_fused`, builds the rolling local costmap directly from live sensors, consumes `mapping/static_costmap` for global planning, and outputs `/navigation/cmd_vel`.

The repository also ships built-in mission assets inside `amr_sweeper_navigation/missions`, so `/missions/database` can stay dedicated to synced mission inputs and `/missions/logs` can stay dedicated to runtime outputs.

## Notes
- The default command launches the full layer 3 navigation bringup package.
- Layer 3 depends on a running layer 1 and layer 2 stack.
- This layer is configured for the trimmed real-robot stack only.
- The vendored FusionCore subtree used by localization is now reduced to the two runtime packages we actually use: `fusioncore_core` and `fusioncore_ros`.
- The localization launch now consumes `/amr_sweeper/gnss/navsat` directly because the local GNSS node publishes a compatible `NavSatFix` stream without the old bridge workaround.
- The old `compass_msgs` / `gnss.azimuth_topic` path has been removed; optional heading aid now comes only from `gnss.heading_topic` when configured.
- `amr_sweeper_localization` owns the optional visual odometry node, which publishes `/amr_sweeper/visual_odometry/odom`; FusionCore consumes that VO topic as `encoder2.topic` when visual odometry is enabled.
- Nav2 runtime parameters now live in mission-class-specific files under `amr_sweeper_navigation/config/`; Nav2 consumes mapping-published `mapping/static_costmap` for global planning and builds its rolling local costmap directly from live sensor topics.
- Mission execution is disabled by default at the layer 3 entrypoints and should only be enabled by FSM `RUNNING` profiles.
