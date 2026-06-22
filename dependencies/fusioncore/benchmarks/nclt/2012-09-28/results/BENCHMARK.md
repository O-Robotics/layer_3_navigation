# Benchmark Results: NCLT Sequence 2012-09-28

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|-------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 9.104 | 34.4% | 73.7% | 0.8612 | 0.00 | 16.606 |
| RL-EKF | 74.366 | 1.0% | 2.8% | 0.7999 | 0.00 | 16.519 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-09-28
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- Motion model: DifferentialDrive
- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)
