# Benchmark Results: NCLT Sequence 2012-03-31

## Metrics (SE3-aligned to RTK ground truth)

| Filter | ATE RMSE (m) | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) | RPE@10m RMSE (m) |
|--------|-------------|------------|-------------|-------------------|--------------|------------------|
| FusionCore | 8.411 | 51.5% | 89.3% | 0.7064 | 0.83 | 18.835 |
| RL-EKF | 115.407 | 0.0% | 1.2% | 0.5738 | 11.35 | 19.566 |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-03-31
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- Motion model: DifferentialDrive
- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)
