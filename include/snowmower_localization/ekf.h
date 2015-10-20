/* ekf.h */
#include <ros/ros.h>
#include <Eigen/Core>

#ifndef __EKF_H_INCLUDED__
#define __EKF_H_INCLUDED__

using namespace Eigen;

class Ekf {

 typedef Matrix<double, 5, 1> Vector5d;
 typedef Matrix<double, 5, 5> Matrix5d;

 private:
  /*************
    Parameters
  *************/
  // time interval for system update in seconds
  double dt_;
  // Wheel Track
  double b_;
  // Left and right encoder ticks per revolution of the wheel
  int tprRight_;
  int tprLeft_;

  // System Model Covariance Matrix
  Matrix5d Q_;

  // Decawave Beacon Locations (in meters)
  double dw1x_, dw1y_, dw2x_, dw2y_, dw3x_, dw3y_, dw4x_, dw4y_;
  // Decawave Covariance Matrix
  Matrix4d RDecaWave_;

  /*******************
    Member Variables
  *******************/
  // Node specific stuff
  ros::NodeHandle public_nh_;
  ros::NodeHandle private_nh_;

  // State Vector and Covariance Matrix
  Vector5d state_;
  Matrix5d cov_;

  /*******************
    Member Fucntions
  *******************/
  // System update
  void systemUpdate();

  // Map frame measurement updates
  void measurementUpdateGPS(double x, double y);
  
  Vector4d  hDecaWave(Vector5d state);
  void measurementUpdateDecaWave(Vector4d z);
  
  void measurementUpdateVisualBeacons(double angles[]);
  
  // Odom frame measurement updates
  void measurementUpdateEncoders(double vR, double vL);
  
  void measurementUpdateIMU(double wZ, double aX, double aY);
  
  void measurementUpdateVisualOdometry();
  

 public:
  Ekf();
  ~Ekf();

};

#endif
