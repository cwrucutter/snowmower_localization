/******************************************************************************
ekf.h
The Ekf class contains all of the mathematical functions needed to perform a
system update and several measurement updates for an extended Kalman filter.
*******************************************************************************
The MIT License (MIT)

  Copyright (c) 2015 Matthew A. Klein

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#include <ros/ros.h>
#include <Eigen/Core>
#include <sensor_msgs/Imu.h>
#include <snowmower_msgs/EncMsg.h>
#include <snowmower_msgs/DecaWaveMsg.h>

#ifndef __EKF_H_INCLUDED__
#define __EKF_H_INCLUDED__

using namespace Eigen;

class Ekf {

 typedef Matrix<double, 5, 1> Vector5d;
 typedef Matrix<double, 5, 5> Matrix5d;
 typedef Matrix<double, 1, 5> Matrix15;
 typedef Matrix<double, 2, 5> Matrix25;
 typedef Matrix<double, 1, 1> Matrix11;
 typedef Matrix<double, 4, 5> Matrix45;

 public:
  /*************
    Parameters
  *************/
  // Wheel Track
  double b_;
  // Left and right encoder ticks per meter traveled
  int tpmRight_;
  int tpmLeft_;

  // System Model Covariance Matrix
  Matrix5d Q_;

  // Decawave Beacon Locations (in meters)
  double dw1x_, dw1y_, dw2x_, dw2y_, dw3x_, dw3y_, dw4x_, dw4y_;
  // Decawave Covariance Matrix
  Matrix4d RDecaWave_;
  Matrix2d REnc_;
  double RIMU_;

  /*******************
    Member Variables
  *******************/
  // Node specific stuff
  ros::NodeHandle public_nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher statePub_;
  ros::Subscriber dwSub_;
  ros::Subscriber imuSub_;
  ros::Subscriber encSub_;


  // State Vector and Covariance Matrix
  Vector5d state_;
  Matrix5d cov_;

  // Store the time of the last update. This is used to determine dt.
  ros::Time lastTime_;
  /*******************
    Member Fucntions
  *******************/
  // System update
  Vector5d fSystem(Vector5d state, double dt);
  Matrix5d FSystem(Vector5d state, double dt);
  void systemUpdate(double dt);

  // Map frame measurement updates
  void dwSubCB(const snowmower_msgs::DecaWaveMsg& msg);
  Vector4d hDecaWave(Vector5d state);
  Matrix45 HDecaWave(Vector5d state);
  void measurementUpdateDecaWave(Vector4d z); // z is d1-d4
  
  // Odom frame measurement updates
  void encSubCB(const snowmower_msgs::EncMsg& msg);
  Vector2d hEnc(Vector5d state);
  Matrix25 HEnc(Vector5d state);
  void measurementUpdateEncoders(Vector2d z); // z is encL and encR
  
  void imuSubCB(const sensor_msgs::Imu& msg);
  double hIMU(Vector5d state);
  Matrix15 HIMU(Vector5d state);
  void measurementUpdateIMU(double z); // z is omega_z
  
  void measurementUpdateVisualOdometry();

  // Determine time since the last time dt() was called.
  double dt(ros::Time currentTime);

  // Publish the state as an odom message on the topic odom_ekf. Alos well broadcast a tansform.
  void publishState(); 
  
  // Initialization process
  void init();

  //public:
  Ekf();
  ~Ekf();

  // Getters and Setters
  Vector5d getState(); // Returns state vector
  Matrix5d getCov();   // Returns covariance matrix
};

#endif
