# navsat_transform startup errors and transform unavailable

"Could not obtain transform from map to base_link" is the most common startup error in robot_localization GPS setups. This page explains what causes it, how to fix it, and how FusionCore avoids the problem entirely.

---

## What the error means

navsat_transform_node needs two things before it can publish GPS odometry:

1. The current filter pose from ekf_node (subscribed as `/odometry/filtered`)
2. A valid transform from `map` to `base_link` (or `odom` to `base_link`)

If ekf_node hasn't initialized yet, or if the TF tree doesn't have that chain, navsat_transform logs the error and stalls. It will retry, but until both conditions are met, no GPS odometry flows to the EKF and the filter runs without GPS.

The startup ordering problem: ekf_node needs GPS odometry from navsat_transform to fuse GPS. navsat_transform needs filtered odometry from ekf_node to compute GPS odometry. Circular dependency. Whichever node starts first has nothing to subscribe to.

---

## Diagnosis

Check whether the transform is actually missing or just delayed:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

If this prints transforms, the TF chain exists and the error was transient (startup race). Wait a few seconds and check if `/odometry/gps` starts publishing.

If it never prints, something is wrong with the TF tree:

```bash
ros2 run tf2_tools view_frames
```

Open the generated PDF. Look for a broken chain between `map` and `base_link`. Common causes:

- No node is publishing `odom -> base_link` yet (ekf_node hasn't initialized)
- ekf_node is set to `world_frame: map` but no separate map publisher exists
- A `static_transform_publisher` is missing for a link in the chain

---

## Fixes

### Fix 1: Set world_frame to odom, not map

Most outdoor robots don't need a `map` frame until SLAM is running. Using `world_frame: odom` in ekf_node means the `odom -> base_link` transform is enough. navsat_transform doesn't need a `map` frame then.

```yaml
# ekf_node config
world_frame: odom  # not map
```

### Fix 2: Add a static map->odom transform while SLAM is inactive

If you need `world_frame: map` but don't have SLAM running at startup, publish a static identity transform until SLAM takes over:

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
),
```

Remove this when SLAM publishes the real `map -> odom` transform.

### Fix 3: Control startup order with launch events

Delay navsat_transform until ekf_node is up:

```python
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

# Start navsat_transform 3 seconds after ekf_node
TimerAction(
    period=3.0,
    actions=[navsat_transform_node]
)
```

This is a workaround, not a fix. The real issue is the circular dependency.

### Fix 4: Use wait_for_datum

If your robot starts stationary, set `wait_for_datum: false` and let navsat_transform anchor to the first valid GPS fix. The filter will run without GPS for a few seconds, then GPS comes online once navsat_transform has enough information. Position jumps slightly at first GPS integration, then stabilizes.

```yaml
wait_for_datum: false
```

---

## Why FusionCore doesn't have this problem

FusionCore has no navsat_transform node. GPS is fused directly inside the filter in ECEF coordinates. There is no circular dependency because there is no second node that needs the first node's output to produce its own output.

The startup sequence is: IMU arrives, filter initializes, GPS arrives, first fix anchors the ECEF datum, fusion begins. One node, linear startup, no TF prerequisite.

```bash
ros2 launch fusioncore_ros fusioncore.launch.py fusioncore_config:=your_robot.yaml
```

That's the full launch for GPS fusion. No navsat_transform, no circular ordering problem.

---

## The UTM zone boundary variant

A related failure: the robot crosses a UTM zone boundary and navsat_transform's `broadcast_utm_transform: true` produces a discontinuous jump. The UTM projection has a seam at every 6-degree longitude boundary. If your operating area is near one (or you're driving a long enough route to cross one), the transform jumps at the boundary and the filter sees a 300-700m position step.

Fix for robot_localization: use `use_local_cartesian: true` in navsat_transform. This uses a local flat-earth approximation anchored at the datum point instead of UTM projection, avoiding the zone boundary entirely.

Fix for FusionCore: not applicable. FusionCore fuses in ECEF, which has no zone boundaries anywhere on Earth.

---

## Related

- [Handling delayed GPS measurements](delayed-gps-measurements.md)
- [FusionCore vs robot_localization](../vs-robot-localization.md)
- [Migrating from robot_localization](../migration_from_robot_localization.md)
