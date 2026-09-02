# amr_sweeper_localization

```bash
ros2 launch amr_sweeper_localization amr_sweeper_localization.launch.py
```

## Purpose
This package launches the official upstream FusionCore ROS 2 localization stack for the AMR Sweeper, provides the robot-specific parameter file used by that node, and owns the optional visual odometry node used as FusionCore's secondary encoder source.

## Dependencies
- `amr_sweeper_imu`
- `amr_sweeper_gnss`
- `amr_sweeper_layer_1_hardware_bringup`

## Launch File
- `launch/amr_sweeper_localization.launch.py`
- `launch/amr_sweeper_visual_odometry.launch.py`

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

When `use_visual_odometry:=true`, the same launch file also starts `visual_odometry_node`, which publishes `visual_odometry/odom` and `visual_odometry/status`.

The parameter file [config/amr_sweeper_localization.yaml](/home/alfolsen/dev/rob_ws/src/layer_3_navigation/amr_sweeper_localization/config/amr_sweeper_localization.yaml) follows the official FusionCore schema and uses the official `fusioncore:` YAML root, plus a small `launch_defaults:` section that is only consumed by this package's launch file.
The odometry projector tunables live in [config/odometry_projection.yaml](/home/alfolsen/dev/rob_ws/src/layer_3_navigation/amr_sweeper_localization/config/odometry_projection.yaml).
The visual odometry tunables live in [config/amr_sweeper_visual_odometry.yaml](/home/alfolsen/dev/rob_ws/src/layer_3_navigation/amr_sweeper_localization/config/amr_sweeper_visual_odometry.yaml).

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

## Local patches to vendored FusionCore
`dependencies/fusioncore` is a vendored snapshot of the upstream [manankharwar/fusioncore](https://github.com/manankharwar/fusioncore) project (trimmed to `fusioncore_core` + `fusioncore_ros`), not a live submodule. It should otherwise track upstream unmodified (see Notes above), but the following bug fix is patched locally pending upstreaming:

- **File**: `dependencies/fusioncore/fusioncore_ros/src/fusion_node.cpp`, IMU callback's startup bias-window accumulation block.
- **Bug**: the bias window that seeds the filter's initial heading gated orientation samples on `orientation_covariance[0] > 0.0`. Per `sensor_msgs/Imu` convention (and per `fuse_imu_orientation_if_valid()` elsewhere in the same file, which already gets this right), an all-zero `orientation_covariance` means "orientation is valid, covariance just isn't known" — only `-1` means "no orientation data." Gazebo's IMU plugin (and this stack's simulated `imu/data_raw`, which intentionally publishes all-zero orientation covariance — see `amr_sweeper_imu/config/amr_sweeper_imu.yaml`) was therefore always rejected by the startup window, so the filter always seeded its initial heading as identity (yaw = 0) instead of the true sensor-derived heading, regardless of the robot's actual starting orientation.
- **Symptom**: intermittent large (~90-100°) heading offset in `localization/odometry_fused` for most of a mission after a fresh FusionCore start, self-correcting only if/when GNSS track-heading later overrides it. Root-caused and confirmed against Gazebo ground truth in the `empty1` simulation debrief (2026-07-14).
- **Fix applied**: changed the gate to `orientation_covariance[0] >= 0.0`, matching the convention already used by `fuse_imu_orientation_if_valid()`.
- **Status**: patched locally only. Not yet reported or upstreamed to `manankharwar/fusioncore`. Re-check this patch still applies (or re-apply after re-deriving as a minimal diff against a clean upstream checkout) whenever `dependencies/fusioncore` is re-vendored from a newer upstream snapshot.

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
- `use_visual_odometry:=true` starts `visual_odometry_node` and enables FusionCore `encoder2.topic`
- `use_imu:=false` clears `imu.topic`
- `use_imu2:=false` clears `imu2.topic`
- `use_gnss:=false` clears `gnss.fix2_topic`, `gnss.heading_topic`, and `gnss.azimuth_topic`, and remaps `/gnss/fix` to a disabled placeholder
