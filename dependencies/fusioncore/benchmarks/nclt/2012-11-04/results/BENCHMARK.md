# Benchmark Results: NCLT Sequence 2012-11-04

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|-------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 34.254 | 0.0% | 11.9% | 1.1566 | 5.38 | 30.914 |
| RL-EKF | 40.729 | 0.0% | 3.1% | 0.9424 | 6.39 | 21.872 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-11-04
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- Motion model: DifferentialDrive
- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)
