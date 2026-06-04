# amr_sweeper_localization

```bash
ros2 launch amr_sweeper_localization amr_sweeper_localization.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_imu`
- `amr_sweeper_gnss`
- `amr_sweeper_layer_1_hardware_bringup`

## Purpose
This package provides the real-robot localization stack for the AMR Sweeper.

## Main Launch File
`launch/amr_sweeper_localization.launch.py`

## Available Launch Files
- `amr_sweeper_localization.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`

## Overview
`amr_sweeper_localization` contains the FusionCore launch path plus the parameter file `config/amr_sweeper_localization.yaml` used to fuse wheel odometry, IMU data, and GNSS data into a single robot odometry estimate. It is the localization foundation for the navigation layer and is normally launched as part of layer 3 bringup. The launch also publishes an identity `map -> odom` static transform so tools such as Foxglove can use a `map` fixed frame even when navigation is running directly in FusionCore's `odom` frame.

## Notes
- The primary IMU topic is configured by `imu.topic` in `config/amr_sweeper_localization.yaml`; by default it is `imu/data_raw`, which resolves to `/amr_sweeper/imu/data_raw` under the default namespace. An optional second IMU can be enabled with the `imu2.*` group.
- The primary wheel-odometry topic is configured by `encoder.topic` and defaults to `diff_cont/odom`; an optional visual-odometry source can be enabled and tuned with the `encoder2.*` group.
- Uses IMU data from `amr_sweeper_imu` and GNSS data from `amr_sweeper_gnss`.
- FusionCore consumes GNSS directly from `/amr_sweeper/gnss/navsat`.
- FusionCore is configured to publish `odom -> base_footprint` as yaw-only. Chassis roll/pitch is owned by the layer 2 attitude controller through the `base_footprint -> base_link` chain.
- Publishes a static `map -> odom` identity transform for visualization/debugging compatibility.
