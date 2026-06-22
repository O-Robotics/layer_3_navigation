# Benchmark Results: NCLT Sequence 2012-05-11

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|-------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 5.498 | 70.9% | 95.0% | 0.7983 | 0.55 | 15.302 |
| RL-EKF | 36211.977 | 0.0% | 0.0% | 1737.8239 | 3617.85 | 175669.502 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-05-11
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- Motion model: DifferentialDrive
- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)
