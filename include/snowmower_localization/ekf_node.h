/******************************************************************************
ekf_node.h
The EkfNode class interfaces an Ekf clas with ROS communication functions.
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
#include <sensor_msgs/Imu.h>
#include <snowmower_msgs/EncMsg.h>
#include <snowmower_msgs/DecaWaveMsg.h>
#include "snowmower_localization/ekf.h"

#ifndef __EKF_NODE_H_INCLUDED__
#define __EKF_NODE_H_INCLUDED__

class EkfNode {

 private:
  Ekf ekf_odom_;
  Ekf ekf_map_;

  // Node specific stuff
  ros::NodeHandle public_nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher stateMapPub_;
  ros::Publisher stateOdomPub_;
  ros::Subscriber dwSub_;
  ros::Subscriber imuSub_;
  ros::Subscriber encSub_;

  // Sensor callback functions
  void dwSubCB(const snowmower_msgs::DecaWaveMsg& msg);
  void encSubCB(const snowmower_msgs::EncMsg& msg);
  void imuSubCB(const sensor_msgs::Imu& msg);

  // Publish the state as an odom message on the topic odom_ekf. Also well broadcast a tansform.
  void publishState(ros::Time now); 

  std::string base_frame_; // Frame of the robot
  std::string odom_frame_; // Frame of odom
  std::string map_frame_;  // Frame of the map

  // Check if an encoder message has been recieved yet. First time, just
  // initialize zPre_, lastSysTime_, and lastEncTime.
  bool firstRunEnc_;
  bool firstRunSys_;
  // Store the time of the last System update. This is used to determine dt.
  ros::Time lastSysTime_;
  // Store the time of the last Encoder update. This is used to determine speed
  ros::Time lastEncTime_;
  // Sore last Encoder tick count. Needed to predict new encoder reading.
  Vector2i zPre_;

  // Determine time since the last time dtSys() was called.
  double dtSystem(ros::Time currentSysTime);
  // Determine time since the last time dtEnc() was called.
  double dtEncoder(ros::Time currentEncTime);

  
  // Initialization process
  void init();

 public:
  EkfNode();
  ~EkfNode();
};

#endif
