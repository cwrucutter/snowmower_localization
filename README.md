# snowmower_localization
This repository contains an Extended Kalman Filter (EKF) that combines measurements from two wheel encoders, an IMU, and DecaWave beacons, along with a model of differential drive vehicle kinematics, into an estimation of the robot's location.

## Note on Tests
In order to do unit tests on Eigen Matrix objects with meaningful output in the case of a failure, the following packages must be added to your workspace:
* [eigen_checks](https://github.com/ethz-asl/eigen_checks)

* [glog_catkin](https://github.com/ethz-asl/glog_catkin)

* [catkin_simple](https://github.com/catkin/catkin_simple)
