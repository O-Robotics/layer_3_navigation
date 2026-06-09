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
- `use_imu`: default comes from `config/amr_sweeper_localization.yaml`
- `use_imu2`: default comes from `config/amr_sweeper_localization.yaml`
- `use_encoder`: default comes from `config/amr_sweeper_localization.yaml`
- `use_visual_odometry`: default comes from `config/amr_sweeper_localization.yaml`
- `use_gnss`: default comes from `config/amr_sweeper_localization.yaml`

## Overview
`amr_sweeper_localization` contains the FusionCore launch path plus the parameter file `config/amr_sweeper_localization.yaml` used to fuse wheel odometry, IMU data, and GNSS data into a single robot odometry estimate. It is the localization foundation for the navigation layer and is normally launched as part of layer 3 bringup. The current launch publishes `odom -> base_footprint` through FusionCore and does not publish any fallback or identity `map -> odom` transform.

## Notes
- The primary IMU topic is configured by `imu.topic` in `config/amr_sweeper_localization.yaml`; by default it is `imu/data_raw`, which resolves to `/amr_sweeper/imu/data_raw` under the default namespace. An optional second IMU can be enabled with the `imu2.*` group.
- The primary wheel-odometry topic is configured by `encoder.topic` and defaults to `drive_controller/odom`; an optional visual-odometry source can be enabled and tuned with the `encoder2.*` group.
- Uses IMU data from `amr_sweeper_imu` and GNSS data from `amr_sweeper_gnss`.
- FusionCore consumes GNSS directly from `/amr_sweeper/gnss/navsat`.
- FusionCore is configured to publish `odom -> base_footprint` as yaw-only. Chassis roll/pitch is owned by the layer 2 attitude controller through the `base_footprint -> base_link` chain.
- Publishes `odom -> base_footprint` from FusionCore when `publish.tf` is enabled in `config/amr_sweeper_localization.yaml`.
- Does not publish `map -> odom`; that transform is expected to come from the mapping stack when SLAM is running.

## Sensor Isolation
The default sensor enables now live in `config/amr_sweeper_localization.yaml`:

```yaml
use_imu: true
use_imu2: false
use_encoder: true
use_visual_odometry: false
use_gnss: true
```

You can still override any of them at launch time without editing the YAML:

```bash
ros2 launch amr_sweeper_localization amr_sweeper_localization.launch.py \
  use_imu:=true \
  use_imu2:=false \
  use_encoder:=true \
  use_visual_odometry:=false \
  use_gnss:=true
```

Notes:
- `use_visual_odometry` controls the `encoder2.topic` input and is separate from launching the standalone `amr_sweeper_visual_odometry` package.
- `use_gnss:=false` disables the primary `/gnss/fix` remap and clears optional GNSS heading/fix2 inputs inside FusionCore.
- `use_imu2:=true` preserves whatever `imu2.topic` is configured in `config/amr_sweeper_localization.yaml`; `use_imu2:=false` forces it off.
