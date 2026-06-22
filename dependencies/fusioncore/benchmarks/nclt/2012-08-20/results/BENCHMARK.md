# Benchmark Results: NCLT Sequence 2012-08-20

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|-------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 6.768 | 56.0% | 89.2% | 1.0544 | 0.88 | 15.981 |
| RL-EKF | 86.590 | 0.0% | 0.0% | 1.0178 | 11.20 | 15.291 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-08-20
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- Motion model: DifferentialDrive
- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)
