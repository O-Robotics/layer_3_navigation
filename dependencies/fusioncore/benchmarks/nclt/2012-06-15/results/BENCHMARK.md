# Benchmark Results: NCLT Sequence 2012-06-15

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|-------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 6.036 | 61.1% | 94.9% | 0.9172 | 0.96 | 16.073 |
| RL-EKF | 162.436 | 0.0% | 0.0% | 0.8840 | 25.86 | 15.496 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-06-15
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- Motion model: DifferentialDrive
- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)
