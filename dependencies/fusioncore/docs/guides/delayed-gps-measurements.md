# Handling delayed GPS measurements in ROS 2 sensor fusion

GPS fixes typically arrive 50-200 ms after the moment they were computed. By the time the NavSatFix message reaches your filter, the robot has moved. If your filter doesn't account for this delay, it fuses a position that corresponds to a different point in the robot's trajectory and the estimate accumulates error proportional to speed times delay.

This page explains how delay causes errors, how to diagnose it in your ROS 2 setup, and what both robot_localization and FusionCore provide to handle it.

---

## Why GPS arrives late

The delay from physical observation to ROS timestamp has several sources:

- **Receiver processing:** the receiver takes 10-50 ms to compute the fix from satellite signals
- **Serial/USB latency:** UART at 9600 baud introduces up to 10 ms per byte; a full NMEA sentence at that rate takes ~50 ms
- **Driver buffering:** the ROS driver parses the sentence and publishes, adding 5-20 ms
- **Network latency:** if the receiver publishes over TCP or a serial bridge, add network jitter

Total: 50-200 ms is typical. At 1 m/s, 100 ms delay = 10 cm position error per fix. At 3 m/s (typical outdoor robot), it's 30 cm per fix, accumulating into meters over a long run.

---

## How to measure your GPS delay

Record a rosbag with both IMU and GPS, then compare timestamps vs receive times:

```bash
ros2 bag play your_bag.bag --pause
ros2 bag info your_bag.bag
```

For a rough estimate, check the lag between when the GPS timestamp says the fix happened and when it was received:

```python
import rclpy
from sensor_msgs.msg import NavSatFix
import time

def cb(msg):
    ros_time = rclpy.clock.Clock().now().nanoseconds / 1e9
    msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    print(f"GPS delay: {(ros_time - msg_time)*1000:.1f} ms")
```

If your driver publishes with `stamp.sec = 0`, it is not timestamping the fix at observation time. The message arrives with zero timestamp and the filter applies it "now", which is wrong by the full processing delay.

---

## Diagnosing delay-induced error

Delay-induced error looks different from GPS noise:

- Position error is correlated with speed: faster runs have worse trajectory error
- The error is consistent in direction: straight segments show the estimate behind the true path
- Plotting GPS fix positions vs filter output shows the filter output consistently ahead of the GPS fixes

---

## robot_localization: smooth_lagged_data

robot_localization provides `smooth_lagged_data: true` with `history_length` to handle late-arriving sensor messages. When enabled, rl buffers recent filter state and rewinds to re-process a late measurement at its correct timestamp.

```yaml
smooth_lagged_data: true
history_length: 0.5    # seconds of history to keep
```

The limitation: rewinding is computationally expensive and the re-linearization at the old state is approximate in EKF mode. For delays up to 100-200 ms at typical robot speeds, it works reasonably well. For longer delays or higher speeds the approximation error grows.

Also: if your GPS driver publishes with zero-stamped headers (`stamp.sec = 0`), rl cannot tell what time the fix corresponds to and smooth_lagged_data doesn't help. Fix the driver or use a timestamp correction node first.

---

## FusionCore: IMU ring buffer retrodiction

FusionCore keeps a 1-second ring buffer of IMU measurements. When a GPS fix arrives with a timestamp earlier than the current filter time, FusionCore:

1. Identifies the filter state at the GPS timestamp from the ring buffer
2. Applies the GPS update to that historical state
3. Replays all buffered IMU steps forward to reconstruct the current state exactly

This is exact replay, not approximation. The same sigma points are re-propagated forward from the GPS-updated historical state. No linearization artifact.

```yaml
# No configuration needed: IMU ring buffer is always on
# Handles any delay up to 1 second automatically
```

The buffer is always active. No parameters to set unless you want to adjust the buffer size:

```yaml
imu.buffer_duration: 1.0   # seconds; increase if GPS delays exceed 1s
```

For zero-stamped GPS messages, FusionCore uses wall clock as fallback:

```yaml
gnss.use_wall_clock_stamp: true   # use /clock if stamp=0
```

---

## Comparing the two approaches

| Aspect | robot_localization | FusionCore |
|---|---|---|
| Mechanism | State rewind + re-processing | IMU ring buffer exact replay |
| Max delay handled | Configurable via `history_length` | 1 second (default), configurable |
| Accuracy | Approximate (re-linearization at old state) | Exact (sigma point replay) |
| Zero-stamped GPS | Not handled | Fallback to wall clock via `gnss.use_wall_clock_stamp` |
| CPU cost | Proportional to delay and sensor rate | Constant (O(N) buffer lookup) |
| Configuration | `smooth_lagged_data: true` + `history_length` | Always on |

---

## Practical recommendations

**For robot_localization users:**

1. Fix the GPS driver to stamp messages at observation time if possible
2. Set `smooth_lagged_data: true` and `history_length: 0.5` as a minimum
3. Measure your actual delay and set `history_length` at least 2x that value
4. If using a u-blox ZED-F9P: enable the `timemark` feature to get hardware-timestamped fixes

**For FusionCore users:**

Delay compensation is automatic. If you see delay-correlated errors, the likely causes are:
- Zero-stamped GPS messages: enable `gnss.use_wall_clock_stamp: true`
- GPS delay exceeding 1 second: increase `imu.buffer_duration`
- Driver adding its own timestamp offset: check the driver documentation

---

## Related

- [Configuration reference: GNSS section](../configuration.md)
- [FusionCore vs robot_localization](../vs-robot-localization.md)
- [GPS position jumping near buildings](gps-multipath-position-jumping.md)
