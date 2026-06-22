---
date: 2026-05-19
title: "FusionCore + icp_odometry feedback loop merged into rtabmap_ros"
description: "PR #1419 was merged into introlab/rtabmap_ros. Here is what the architecture actually does and why the feedback direction matters."
slug: rtabmap-fusioncore-icp-loop
tags:
  - ROS
---

PR [#1419](https://github.com/introlab/rtabmap_ros/pull/1419) was merged into `introlab/rtabmap_ros` last week, co-authored with Mathieu Labbe. It adds a TurtleBot3 Gazebo Harmonic demo showing a feedback loop between FusionCore (IMU + wheel UKF) and `icp_odometry`.

<!-- more -->

The architecture is worth describing because the feedback direction is non-obvious:

```
/imu  ──────────────────────┐
/odom (wheel) ──────────────┤──> FusionCore (UKF)
/rtabmap/icp_odometry ──────┘    │
    ^                            │ odom -> base_footprint TF
    │                            │ /fusion/odom
    │        guess_frame_id: odom│
    └──── icp_odometry <─────────┘
             │
             └──> rtabmap SLAM ──> map -> odom TF
```

FusionCore runs at 100 Hz and owns the `odom` frame. `icp_odometry` uses that frame as the initial guess for scan matching via `guess_frame_id`. A stable initial guess means scan matching succeeds more consistently and with lower residual error. The ICP result feeds back into FusionCore as a second velocity source (`encoder2`), tightening the UKF state estimate. rtabmap handles loop closure and map correction on top.

Each node tightens the other. Neither is strictly downstream.

## Running it

```bash
sudo apt install ros-jazzy-fusioncore-ros ros-jazzy-rtabmap-ros \
                 ros-jazzy-turtlebot3-gazebo ros-jazzy-nav2-bringup
export TURTLEBOT3_MODEL=waffle
ros2 launch rtabmap_demos turtlebot3_sim_fusioncore_icp_demo.launch.py
```

Full architecture notes and topic/TF table are in the [demo README](https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_demos/launch/turtlebot3/fusioncore).

If you are using rtabmap outdoors with GPS, FusionCore handles that separately. See the [FusionCore repository](https://github.com/manankharwar/fusioncore) for GPS + IMU + wheel encoder fusion configs.
