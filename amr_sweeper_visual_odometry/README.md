# amr_sweeper_visual_odometry

```bash
ros2 launch amr_sweeper_visual_odometry amr_sweeper_visual_odometry.launch.py
```

Dependencies to other AMR Sweeper packages:
- `amr_sweeper_layer_1_hardware_bringup`
- `amr_sweeper_usb_cameras`
- `amr_sweeper_description`

## Purpose
This package provides a monocular visual odometry path for the AMR Sweeper cameras.

## Main Launch File
`launch/amr_sweeper_visual_odometry.launch.py`

## Overview
`amr_sweeper_visual_odometry` tracks visual features from one or more monocular camera feeds, estimates per-camera relative motion from consecutive frames, and uses wheel odometry only to recover metric translation scale. The node then fuses the per-camera motion estimates into a single `nav_msgs/msg/Odometry` estimate in the robot's `odom` frame that can be fed into FusionCore as a secondary odometry source.

## Notes
- Main node: `visual_odometry_node`.
- Default configured camera list: `["tool_camera"]`.
- Default camera info input: `/amr_sweeper/usb_cameras/tools_camera/tools_camera_info`.
- Default wheel odometry input: `/amr_sweeper/drive_controller/odom`.
- Default odometry output: `/amr_sweeper/visual_odometry/odom`.
- `amr_sweeper_visual_odometry` does not launch FusionCore. The intended stack is visual odometry here, then `amr_sweeper_localization` consumes `/amr_sweeper/visual_odometry/odom` as FusionCore `encoder2.topic`.
- Additional cameras can be added through `camera_names` plus `cameras.<name>.image_topic`, `cameras.<name>.camera_info_topic`, and `cameras.<name>.camera_frame`.
- When multiple cameras are configured, their short-horizon motion estimates are grouped within `camera_fusion_tolerance_seconds` and fused into one odometry update.
- Monocular RGB alone is scale-ambiguous. This package uses wheel odometry magnitude to scale the visually estimated translation direction into metric motion.
- The tool camera currently runs at a low frame rate in layer 1. Visual odometry quality will improve noticeably if that camera is calibrated accurately and published at a higher rate.
