# snowmower_localization
This repository contains an Extended Kalman Filter (EKF) that fuses wheel encoder, IMU, visual odometry, and DecaWave beacon measurements, along with a model of differential drive vehicle kinematics, into an estimation of the robots location.

## Tests
In order to do unit tests on Eigen Matrix objects with meaningful output in the case of a failure, the following packages must be added to your workspace:
* [eigen_checks](https://github.com/ethz-asl/eigen_checks)

* [glog_catkin](https://github.com/ethz-asl/glog_catkin)

* [catkin_simple](https://github.com/catkin/catkin_simple)

## To Do
* **DONE** Create an EKF class.

* **DONE** Include publishers and subscribers.

* Remove dependency on boost library in pubState function.

* **DONE** Add second ekf object that only updates in odometry callback functions. Call them ekf_odom_ and ekf_map_.

* **DONE** Add transform broadcaster to pubState function.

* Populate odometry message with Covariances.

* **DONE** Initialize everything!

* **DONE** Create callback functions for subscribers.

* Interpret value of -1 from decawave anchor as invalid and calculate measurment update accordingly.

* Make decawave callback function open to any number of beacons.

* **DONE** Code up the equations for the system model.

* **DONE** Account for omega in denominator in F function.

* **DONE** Write tests for system f.

* **DONE** Write tests for system F

* **DONE** Code decawave measurement model.

* **DONE** Add offset to decawave (base_decawave is not at base_link!).

* **DONE** Write tests for decawave h.

* Write tests for decawave H.

* **DONE** Code up IMU measurement model.

* **DONE** Add IMU bias as a state.

* Write tests for IMU h.

* Write tests for IMU H.

* **DONE** Code up Encoder measurement model.

* **DONE** Initialize time and pervious tick count for encoders.

* **DONE** Write tests for Encoder h.

* Write tests for Encoder H.

* Determine covariance matricies for system and measurement models.

* Make Covariance matricies variable, a la Eric Perko.

* **DONE** Set frame_id and child_frame_id using parameters.

* **DONE** Write code to determine dt (The plan is to run a system and measurement update each time a measurement is recieved.)

* Learn how to forward declare instead of #include in ekf.h

* **DONE** Should v_r, d_r, or number of ticks be the measurement for encoders? Should that process happen in ekf or ekf_node?