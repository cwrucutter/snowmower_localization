# snowmower_localization
This repository contains an Extended Kalman Filter (EKF) that fuses wheel encoder, IMU, visual odometry, and DecaWave beacon measurements, along with a model of differential drive vehicle kinematics, into an estimation of the robots location.

## To Do ##
* **DONE** Create an EKF class.

* **DONE** Include publishers and subscribers.

* Remove dependency on boost library in pubState function.

* Add transform broadcaster to pubState function.

* Populate odometry message with Covariances.

* Create callback functions for subscribers.

* **DONE** Code up the equations for the system model.

* Verify system model.

* **DONE** Code decawave measurement model.

* Add offset to decawave (base_decawave is not at base_link!).

* Verify decawave measurement model.

* **DONE** Code up IMU measurement model.

* Verify IMU measurement model.

* **DONE** Code up Encoder measurement model.

* Verify Encoder measurement model.

* Determine covariance matricies for system and measurement models.

* Set frame_id and child_frame_id using parameters.

* **DONE** Write code to determine dt (The plan is to run a system and measurement update each time a measurement is recieved.)