# Benchmark Results: NCLT Sequence 2012-02-04

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|-------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 56.626 | 4.1% | 19.2% | 1.2840 | 0.00 | 36.902 |
| RL-EKF | 123.757 | 0.0% | 0.0% | 0.7384 | 0.01 | 26.675 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-02-04
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- Motion model: DifferentialDrive
- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)
