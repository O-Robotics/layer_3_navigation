# ROS 2 GPS position jumping near buildings

Your robot drives well in open areas, then passes a building wall or enters an urban canyon, and the position estimate suddenly jumps 5-30 meters. This page explains why it happens, how to diagnose it in both robot_localization and FusionCore, and how to fix it.

---

## Why GPS position jumps near buildings

GPS signals travel from satellites to your receiver in a straight line under ideal conditions. Near buildings, signals bounce off walls before reaching the antenna (multipath). The receiver sees the direct signal plus one or more reflections and can't separate them. The position fix it reports is the result of the combined geometry: typically displaced 5-50 meters from the true position, often in the direction of the reflecting wall.

The GPS driver usually reports the same covariance it reports in open sky. It doesn't know the signal was reflected. From the driver's perspective, it received enough satellites with good signal strength.

This is what makes multipath hard: the GPS driver tells your filter "I have a good fix, 3m sigma" and the filter has no direct way to know it's wrong. The fix has to be rejected on the basis of what the filter predicted, not what the GPS reported.

---

## Diagnosing the problem

### Step 1: Confirm the GPS is the source

```bash
ros2 topic echo /gnss/fix --field position --once
```

Collect a few seconds of fixes while stationary near the building. If position is varying by more than 2x the reported sigma, the GPS signal is already contaminated.

Check fix quality:

```bash
ros2 topic echo /gnss/fix --field status --once
```

`status.status: 0` means autonomous (standard GPS). `status.status: 1` means SBAS-augmented. Fix quality is in `position_covariance_type`: `2` means diagonal known (driver computed covariance), `3` means full covariance. If it's `1` (approximated), the covariance is a guess.

### Step 2: Determine if the filter is accepting or rejecting the bad fix

**In robot_localization:**

Enabling `print_diagnostics: true` prints a line for each rejected measurement. A multipath spike that lands within the Mahalanobis threshold won't be printed because it wasn't rejected. If you don't see a rejection log, the fix was accepted.

**In FusionCore:**

```bash
ros2 topic echo /fusion/debug/gnss_status
```

Every fix produces one entry with `accepted: true/false`, `rejection_reason`, and `mahalanobis_sq`. A spike will show `CHI2_FAILED` and a mahalanobis_sq many times above the 16.27 threshold.

### Step 3: Measure the severity

Get a rosbag of a known path past the building:

```bash
ros2 bag record /gnss/fix /fusion/odom /odom/wheels /imu/data -o multipath_test
```

After the run, compare the filter output trajectory against Google Maps or a surveyed path. The deviation at the worst point tells you how far the fix pulled the filter.

---

## Fixes

### Fix 1: Tighten the rejection threshold (robot_localization)

robot_localization's Mahalanobis gate compares against the GPS-reported covariance. If the driver reports 3m sigma, a fix 6m away has Mahalanobis distance 2.0. The default threshold is often 10+ (very loose). Set it to chi-squared equivalent for a 2-DOF GPS measurement:

```yaml
odom1_pose_rejection_threshold: 3.72   # sqrt(chi2(2, 0.999)) = sqrt(13.82)
```

This rejects any GPS fix more than ~3.7 sigma from the filter's prediction. It helps with large spikes but still accepts small ones because the prediction is calibrated to the stated 3m sigma, not the actual multipath error.

### Fix 2: Increase GPS covariance to match actual outdoor error

If your GPS driver reports 3m sigma but actual multipath error can reach 30m, the gate is miscalibrated. You can override the covariance in a converter node or, if using robot_localization, increase the `initial_estimate_covariance` to force the filter to trust GPS less:

```yaml
initial_estimate_covariance: [1e-9, 0.0, ..., 25.0, 0.0, 25.0, ...]  # inflate GPS position diagonal
```

This is a manual approximation of what FusionCore's adaptive noise does automatically.

### Fix 3: Add a HDOP pre-filter

Most GPS receivers report HDOP (horizontal dilution of precision) in `NavSatFix.status.service` (or as a separate topic for u-blox receivers). HDOP above 2-3 typically indicates degraded geometry, which correlates with multipath risk. Filter at the node level:

```python
def gnss_callback(msg):
    if msg.position_covariance[0] > 25.0:  # var > 5m sigma
        return
    publisher.publish(msg)
```

This is a blunt instrument but catches the worst fixes before they reach the filter.

### Fix 4: Use FusionCore's adaptive noise and chi-squared gate

FusionCore's adaptive noise (`adaptive.gnss: true`) maintains a 50-sample innovation window per sensor. As the filter accumulates evidence that actual GPS error is higher than reported, it updates the noise model:

```
R_gps = (1 - alpha) * R_gps + alpha * C_hat
```

After ~50 fixes in a noisy area, the gate threshold is calibrated to actual error, not driver-reported error. A 30m multipath spike when the filter has learned 5m actual sigma has mahalanobis_sq = (30/5)^2 = 36, well above the 16.27 threshold.

```yaml
adaptive.gnss: true
adaptive.window: 50
adaptive.alpha: 0.01
outlier_threshold_gnss: 16.27   # chi2(3, 0.999): do not lower this
```

---

## What to do during the multipath pass

If the robot follows a predictable path past a known building:

**robot_localization:** no built-in inertial coast. The filter continues trying to fuse GPS during the multipath pass. Tune the rejection threshold and covariance to reject as many bad fixes as possible, but expect some degraded performance.

**FusionCore:** set `gnss.coast_n` to trigger coast mode after 3-5 consecutive rejections. During coast, the filter runs on wheel odometry + IMU alone and stops trying to fuse GPS:

```yaml
gnss.coast_n: 3            # enter coast after 3 consecutive GPS rejections
gnss.coast_timeout_s: 30.0 # also coast if GPS is silent for 30s
gnss.coast_q_factor: 10.0  # inflate process noise during coast for faster GPS reacquisition
```

When GPS returns to good quality after the building, the gate relaxes and reacquisition is automatic.

---

## Related

- [FusionCore vs robot_localization](../vs-robot-localization.md) - benchmark results on sequences with multipath
- [Configuration reference: GNSS section](../configuration.md)
- [Outlier gate diagnostics](../troubleshooting.md#encoder-or-gps-getting-rejected-outlier-gate)
