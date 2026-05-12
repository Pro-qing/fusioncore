# Benchmark Results: NCLT Sequence 2012-01-08

## Absolute Trajectory Error (ATE, SE3-aligned)

| Filter | RMSE (m) | Mean (m) | Max (m) | Rot RMSE (°) |
|--------|----------|----------|---------|---------------|
| FusionCore | 9.089 | 7.813 | 40.670 | 115.583 |
| RL-EKF | 61.099 | 54.283 | 139.572 | 115.726 |

## Pose Accuracy Distribution

| Filter | Within 5 m | Within 10 m | Path Length Ratio | Drift (m/km) |
|--------|------------|-------------|-------------------|---------------|
| FusionCore | 26.6% | 77.2% | 0.9649 | 1.02 |
| RL-EKF | 0.0% | 0.4% | 0.9237 | 6.87 |

## Relative Pose Error (RPE)

| Filter | RPE@10m RMSE | RPE@50m RMSE | RPE@100m RMSE |
|--------|-------------|-------------|---------------|
| FusionCore | 15.336 m | 65.374 m | 115.158 m |
| RL-EKF | 15.772 m | 71.784 m | 121.431 m |

## Methodology

- Dataset: NCLT (University of Michigan)
- Sequence: 2012-01-08
- Ground truth: RTK GPS (gps_rtk.csv) projected to local ENU
- Evaluation: [evo](https://github.com/MichaelGrupp/evo), SE(3) alignment
- Motion model: DifferentialDrive
- Sensor inputs: identical for all filters (IMU + wheel odom + GPS)
