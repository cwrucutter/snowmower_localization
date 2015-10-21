/* ekf.h */
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <snowmower_msgs/EncMsg.h>
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
  ros::Publisher statePub_;
  ros::Subscriber imuSub_;
  ros::Subscriber encSub_;


  // State Vector and Covariance Matrix
  Vector5d state_;
  Matrix5d cov_;

  /*******************
    Member Fucntions
  *******************/
  // System update
  void systemUpdate(double dt);

  // Map frame measurement updates
  Vector4d  hDecaWave(Vector5d state);
  void measurementUpdateDecaWave(Vector4d z); // z is d1-d4
  
  // Odom frame measurement updates
  void encSubCB(const snowmower_msgs::EncMsg& msg);
  void measurementUpdateEncoders(Vector2d z); // z is encL and encR
  
  void imuSubCB(const sensor_msgs::Imu& msg);
  void measurementUpdateIMU(double z); // z is omega_z
  
  void measurementUpdateVisualOdometry();

  // Publish the state as an odom message on the topic odom_ekf. Alos well broadcast a tansform.
  void publishState(); 
  
  // Initialization process
  void init();
 public:
  Ekf();
  ~Ekf();

};

#endif
