# EKF and UKF NaN divergence in ROS 2 sensor fusion

Your filter runs normally for seconds or minutes, then produces NaN on every output topic. The filter is dead and will not recover without a restart. This page explains the three causes and how to fix each.

---

## What NaN divergence looks like

```
[ERROR] [ekf_node]: nan detected in position output
[WARN] [fusioncore]: covariance regularization applied at step 4821
```

Or more subtly: `/fusion/odom` stops updating, RViz shows the robot at the origin, and `ros2 topic echo /fusion/odom --field pose.pose.position.x` prints `nan`.

---

## Cause 1: Covariance matrix loses positive semi-definiteness

The Kalman filter maintains a covariance matrix P that must be positive semi-definite at all times. If floating-point arithmetic accumulates errors (off-diagonal terms drift, rounding errors in predict steps), P can become indefinite. The next square root (Cholesky decomposition in UKF, or matrix inverse in EKF) fails and NaN propagates through everything downstream.

**Who is affected:** robot_localization UKF is highly sensitive to this. It diverges with NaN at 100 Hz IMU input on GPS-heavy sequences, typically within 30-60 seconds.

**Why it happens in robot_localization UKF specifically:** the sigma point propagation accumulates floating-point error faster than the scalar Mahalanobis thresholds can prevent. The covariance matrix drifts slightly indefinite per predict step and the error compounds.

**Fix in robot_localization:** switch to EKF mode. robot_localization EKF is significantly more numerically stable than its UKF. If UKF is required, lower the IMU rate: `imu0_queue_size: 10` or process IMU at 50 Hz instead of 100 Hz.

**Fix in FusionCore:** FusionCore normalizes the quaternion component of the state vector at every predict step and applies a symmetry correction `P = (P + P^T) / 2` to prevent covariance drift. If P loses positive semi-definiteness despite this, a regularization floor is applied before the Cholesky. NaN divergence has not occurred on any of the 12 NCLT sequences (940 total minutes at 100 Hz).

---

## Cause 2: Sensor covariance is zero or nearly zero

If your IMU driver publishes an `Imu` message with `angular_velocity_covariance[0] = 0` (or very close to zero), the measurement noise matrix R becomes singular. The Kalman gain `K = P*H^T * (H*P*H^T + R)^{-1}` involves inverting `H*P*H^T + R`. If R has a zero diagonal, the inversion fails or produces NaN.

Check your sensor covariances:

```bash
ros2 topic echo /imu/data --field angular_velocity_covariance --once
ros2 topic echo /imu/data --field linear_acceleration_covariance --once
ros2 topic echo /gnss/fix --field position_covariance --once
```

If you see all zeros, the driver is not filling covariances. Fix by overriding in the driver config or setting fallback values in FusionCore:

```yaml
imu.gyro_noise: 0.005   # ARW from datasheet: rad/s
imu.accel_noise: 0.1    # VRW from datasheet: m/s^2
```

robot_localization uses whatever covariance the driver publishes with no floor. If the driver publishes zero, rl will attempt to invert a singular matrix.

---

## Cause 3: Outlier not rejected, large measurement update

A 100m GPS spike arrives, the outlier gate doesn't reject it (either because gating is off or the threshold is too loose), and the filter tries to fuse a position 100m away from the current estimate. The Kalman gain is proportional to P and inversely proportional to innovation covariance. A huge innovation with nominal covariance produces a huge state update. The state jumps discontinuously, P updates incorrectly, and the next predict step produces NaN.

**Check:** is outlier rejection enabled and calibrated?

In robot_localization:
```yaml
odom1_pose_rejection_threshold: 3.72   # chi2(2, 0.999) = 13.82 -> sqrt = 3.72
```

If this is not set, rl accepts every GPS fix regardless of how far it is from the prediction.

In FusionCore:
```yaml
outlier_rejection: true
outlier_threshold_gnss: 16.27   # chi2(3, 0.999): do not lower this
```

FusionCore also bounds the covariance at each step independently of outlier rejection, so even if a large measurement update occurs, P cannot become indefinite on the next step.

---

## Diagnosis steps

1. Check when NaN first appears: `ros2 bag play your_bag.bag` and watch the filter output timestamp
2. Check what sensor message arrived just before NaN: look at the GPS or IMU message 1-2 steps earlier
3. Check covariance values on all sensor topics (see Cause 2 above)
4. Check whether outlier rejection thresholds are set (see Cause 3 above)
5. If using robot_localization UKF: try switching to EKF first to rule out Cause 1

---

## Related

- [GPS position jumping near buildings](gps-multipath-position-jumping.md)
- [FusionCore vs robot_localization: RL-UKF NaN section](../vs-robot-localization.md)
- [Outlier gate diagnostics](../troubleshooting.md#encoder-or-gps-getting-rejected-outlier-gate)
