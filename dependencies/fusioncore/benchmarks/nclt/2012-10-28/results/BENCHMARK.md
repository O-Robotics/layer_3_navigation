# Benchmark Results: NCLT Sequence 2012-10-28

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|-------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 5.807 | 56.4% | 93.0% | 0.9482 | 0.68 | 18.191 |
| RL-EKF | 1792.890 | 0.0% | 0.0% | 59.8544 | 211.30 | 18.152 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-10-28
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- Motion model: DifferentialDrive
- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)
