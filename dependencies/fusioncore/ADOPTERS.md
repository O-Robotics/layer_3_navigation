# FusionCore Adopters

Real robots, real hardware, real deployments. If you're using FusionCore, open a
[Discussion](https://github.com/manankharwar/fusioncore/discussions/22) and we'll add you here.

---

## Running on real hardware

| Who | Platform | Sensors | Notes |
|-----|----------|---------|-------|
| Michał Bednarek ([@mbed92](https://github.com/mbed92)), PhD Robotics | Factory differential-drive robot | IMU + wheel odometry + Cartographer | ROS 2 Humble, Poland |
| Russ Hall ([@Russ76](https://github.com/Russ76)) | Andino robot (Raspberry Pi 4) | OAK-D stereo + IMU, Velodyne VLP-16 | Indoor SLAM with rtabmap, ROS 2 Jazzy |
| Pranav Shah ([@pranavpshah](https://github.com/pranavpshah)), Icarus Robotics | Space robot | ORB-SLAM3 stereo + VectorNav 9-axis IMU | GPS-denied environment |

## Actively integrating

| Who | Platform | Sensors | Notes |
|-----|----------|---------|-------|
| Pasquale Cannavacciuolo ([@pakyCannavacciuolo05](https://github.com/pakyCannavacciuolo05)), UniNa Corse — Università degli Studi di Napoli Federico II | Formula Student driverless race car | Xsens MTi-680G IMU/GNSS + custom RK4 bicycle model + kiss-icp VSLAM | 2nd place FSAE Italy 2025 driverless category |
| Sam ([@samuk](https://github.com/samuk)), Agroecology Lab | Outdoor agricultural field robot | Dual u-blox F9P RTK GPS + BNO085 IMU + tracked platform | UK, integration in progress |
| Oakland University Robotics Association ([@ZacharyLain](https://github.com/ZacharyLain)) | "Erdferkel" differential-drive robot | u-blox F9P GPS + Bosch BNO085 IMU + wheel encoders | IGVC competition robot, Michigan |
| Cedric Hauber ([@cedbossneo](https://github.com/cedbossneo)), MowgliNext | Autonomous mowing robot | RTK GPS + IMU + wheel odometry | ROS 2, contributed encoder2 + min_fix_type + ublox covariance floor |
| Jakub Kaflik ([@jkaflik](https://github.com/jkaflik)), OpenMowerNext | Autonomous lawn mower | u-blox F9P RTK GPS + IMU + wheel odometry | PR #45: replacing robot_localization as default localization stack |

## Under evaluation

| Who | Platform | Notes |
|-----|----------|-------|
| Boluwatife Olabiran ([@privvyledge](https://github.com/privvyledge)) | F1/10 scale car + full-size autonomous van | Comparing FusionCore vs Wolf vs TIER IV EagleEye vs robot_localization. ZED + GPS + 360 LiDAR. |
| Joseph Duchesne ([@josephduchesne](https://github.com/josephduchesne)), Upside Robotics | Mecanum/holonomic robot (Duatic) | Dual GPS antenna support |

---

Running FusionCore? Drop a note in [Discussions #22](https://github.com/manankharwar/fusioncore/discussions/22).
