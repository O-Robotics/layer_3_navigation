# amr_sweeper_localization

```bash
ros2 launch amr_sweeper_localization amr_sweeper_localization.launch.py
```

## Purpose
This package launches the official upstream FusionCore ROS 2 localization stack for the AMR Sweeper and provides the robot-specific parameter file used by that node.

## Dependencies
- `amr_sweeper_imu`
- `amr_sweeper_gnss`
- `amr_sweeper_layer_1_hardware_bringup`

## Launch File
- `launch/amr_sweeper_localization.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`
- `use_imu`: default comes from `launch_defaults` in `config/amr_sweeper_localization.yaml`
- `use_imu2`: default comes from `launch_defaults` in `config/amr_sweeper_localization.yaml`
- `use_encoder`: default comes from `launch_defaults` in `config/amr_sweeper_localization.yaml`
- `use_visual_odometry`: default comes from `launch_defaults` in `config/amr_sweeper_localization.yaml`
- `use_gnss`: default comes from `launch_defaults` in `config/amr_sweeper_localization.yaml`

## Overview
`amr_sweeper_localization` launches the upstream `fusioncore_ros` lifecycle node and remaps:
- `/gnss/fix` to `gnss/navsat`
- `/fusion/odom` to `localization/odometry_body`

It also launches a local `odometry_projection_node` that projects the body-frame
FusionCore output onto the leveled `base_footprint` plane:
- `localization/odometry_body`: `odom -> base_link` body estimate from FusionCore
- `localization/odometry_fused`: projected planar `odom -> base_footprint` odometry for downstream consumers
- `localization/pose`: FusionCore body-frame pose aligned with `localization/odometry_body`

The parameter file [config/amr_sweeper_localization.yaml](/mnt/c/home/dev/rob_ws/src/layer_3_navigation/amr_sweeper_localization/config/amr_sweeper_localization.yaml) now follows the official FusionCore schema and uses the official `fusioncore:` YAML root, plus a small `launch_defaults:` section that is only consumed by this package's launch file.
The odometry projector tunables live in [config/odometry_projection.yaml](/mnt/c/home/dev/rob_ws/src/layer_3_navigation/amr_sweeper_localization/config/odometry_projection.yaml).

## Sensor Inputs
- Primary IMU: `imu.topic`, default `imu/data_raw`
- Optional second IMU: `imu2.topic`
- Primary wheel odometry: `encoder.topic`, default `drive_controller/odom`
- Optional second velocity source: `encoder2.topic`, default `visual_odometry/odom`
- Primary GNSS: `/gnss/fix` remapped to `/amr_sweeper/gnss/navsat`
- Optional GNSS heading: `gnss.heading_topic`
- Optional GNSS azimuth heading: `gnss.azimuth_topic`

## Notes
- This package is intended to track the official upstream `fusioncore_core` and `fusioncore_ros` packages without local feature additions.
- The config uses the current upstream parameter names such as `imu.gyro_noise`, `encoder.vel_noise`, `gnss.base_noise_xy`, `vslam.position_noise`, and `ukf.q_encoder_wz_bias`.
- `base_frame` is configured as `base_link`, which matches the upstream FusionCore expectation for IMU/body-frame fusion.
- FusionCore TF publishing is disabled locally; the package projects `odom -> base_footprint` itself while the attitude controller continues to publish `base_footprint -> base_link`.
- The projector keeps the public odometry topic stable for the rest of the workspace by publishing the existing `localization/odometry_fused` name from the internal `localization/odometry_body` topic.
- `publish.yaw_only` is not part of the official upstream FusionCore package and is no longer used here.
- `map -> odom` is not published by this package; that transform is expected to come from the mapping stack.

## Sensor Isolation
The launch defaults live in `config/amr_sweeper_localization.yaml`:

```yaml
launch_defaults:
  use_imu: true
  use_imu2: false
  use_encoder: true
  use_visual_odometry: false
  use_gnss: true
```

You can still override them at launch time:

```bash
ros2 launch amr_sweeper_localization amr_sweeper_localization.launch.py \
  use_imu:=true \
  use_imu2:=false \
  use_encoder:=true \
  use_visual_odometry:=false \
  use_gnss:=true
```

Behavior:
- `use_visual_odometry:=false` clears `encoder2.topic`
- `use_imu:=false` clears `imu.topic`
- `use_imu2:=false` clears `imu2.topic`
- `use_gnss:=false` clears `gnss.fix2_topic`, `gnss.heading_topic`, and `gnss.azimuth_topic`, and remaps `/gnss/fix` to a disabled placeholder
