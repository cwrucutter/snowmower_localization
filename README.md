# snowmower_localization
This repository contains an Extended Kalman Filter (EKF) that fuses wheel encoder, IMU, visual odometry, and DecaWave beacon measurements, along with a model of differential drive vehicle kinematics, into an estimation of the robots location.

## To Do ##
* **DONE** Create an EKF class.

* **DONE** Include publishers and subscribers.

* Add transform broadcaster to pubState function.

* Populate odometry message with Covariances.

* Create callback functions for subscribers.

* **DONE** Code up the equations for the system model.

* Verify system model.

* **DONE** Code decawave measurement model.

* Verify decawave measurement model.

* Code up IMU measurement model.

* Verify IMU measurement model.

* Code up Encoder measurement model.

* Verify Encoder measurement model.

* Determine covariance matricies.

* Set frame_id and child_frame_id using parameters.