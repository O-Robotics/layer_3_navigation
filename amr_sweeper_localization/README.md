# amr_sweeper_localization

```bash
ros2 launch amr_sweeper_localization fusioncore.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_imu`
- `amr_sweeper_gnss`
- `amr_sweeper_layer_1_hardware_bringup`

## Purpose
This package provides the real-robot localization stack for the AMR Sweeper.

## Main Launch File
`launch/fusioncore.launch.py`

## Available Launch Files
- `fusioncore.launch.py`

## Launch Arguments
- `namespace`: default `amr_sweeper`
- `use_sim_time`: default `false`

## Overview
`amr_sweeper_localization` contains the FusionCore launch path plus the parameter file used to fuse wheel odometry, IMU data, and GNSS data into a single robot odometry estimate. It is the localization foundation for the navigation layer and is normally launched as part of layer 3 bringup.

## Notes
- Uses wheel odometry from the layer 1 wheel-control path.
- Uses IMU data from `amr_sweeper_imu` and GNSS data from `amr_sweeper_gnss`.
