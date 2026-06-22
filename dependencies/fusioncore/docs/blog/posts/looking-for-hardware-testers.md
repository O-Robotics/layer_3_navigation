---
date: 2026-03-28
title: "Looking for hardware testers: FusionCore on real robots"
description: "Simulation works. Now I need real robots. If you have run into robot_localization's limits and haven't found a clean path forward, let's talk."
slug: looking-for-hardware-testers
tags:
  - ROS
---

The filter is working correctly in simulation. The benchmarks on the NCLT dataset look good. What I don't have yet is enough data from real hardware deployments across different robot platforms and sensor combinations. That's what I'm looking for.

<!-- more -->

## What I'm asking for

If you have a wheeled robot with IMU + GPS (or IMU + encoders only) running on ROS 2 Jazzy, and you have hit limits with `robot_localization` that you haven't resolved, I want to help you get FusionCore running on it.

Your specific robot, your specific sensors, your specific failure mode becomes the next thing I fix. That is not a marketing line. It is how the hardware config library grows into something useful for the whole community.

You don't need to commit anything. Open a GitHub issue describing your setup and what isn't working. Or reply on [Discourse](https://discourse.ros.org). I respond to everything within 24 hours.

## Platforms I particularly want to test

**Outdoor GPS robots:** Long blackout sequences, urban canyons, bad multipath. The filter's coast mode and outlier rejection are designed for exactly this but I want more sequences from the real world to validate against.

**Mecanum and omnidirectional robots:** Differential drive motion models assert lateral velocity = 0. That's wrong for mecanum. FusionCore now handles this automatically: `encoder.nhc_auto_detect: true` (default on) watches the encoder VY field at runtime. After 5 consecutive messages with lateral velocity above 0.02 m/s, it identifies the robot as holonomic and disables the lateral constraint without any config change. Marc from [this thread](https://discourse.ros.org/t/looking-for-real-hardware-testers-fusioncore-ros-2-jazzy-sensor-fusion/41074) raised this use case originally: mecanum robot on uneven floors, wheel slip. The auto-detection was built directly from that conversation.

**Robots with non-standard IMU mounting:** If your IMU is not directly above base_link or is mounted at an angle, FusionCore does a TF lookup from the IMU frame to base_link and rotates measurements before fusing. Set `imu.frame_id` to your URDF frame name, or leave it empty to read the frame from the message header.

**Dual-IMU setups:** Some platforms (VESC + RealSense, for example) have two IMU sources. `imu2.topic` subscribes to the second one and fuses each message as an independent measurement of the same state.

## What I can offer

- Personal help getting your config running
- If something doesn't work on your hardware, it goes to the top of the fix list
- Your platform gets a named hardware config in the repo ([fusioncore_ros/config/](https://github.com/manankharwar/fusioncore/tree/main/fusioncore_ros/config))
- Your deployment becomes a reference case for the project

## How to start

Open an issue on [GitHub](https://github.com/manankharwar/fusioncore/issues) with your robot platform, IMU model, GPS receiver model, and what's not working. Or post here. Either way I will respond.
