/******************************************************************************
ekf_node.cpp
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
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <snowmower_msgs/EncMsg.h>
#include <snowmower_msgs/DecaWaveMsg.h>
#include <nav_msgs/Odometry.h>
#include "snowmower_localization/ekf_node.h"

// DecaWave callback function
void EkfNode::dwSubCB(const snowmower_msgs::DecaWaveMsg& msg){
  ekf_.systemUpdate(dt(msg.header.stamp));
  Vector4d z;
  z << msg.dist[1], msg.dist[2], msg.dist[3],msg.dist[4];
  ekf_.measurementUpdateDecaWave(z);
  publishState();
}

// Wheel Encoder callback function
void EkfNode::encSubCB(const snowmower_msgs::EncMsg& msg){
  ekf_.systemUpdate(dt(msg.header.stamp));
  Vector2d z;
  z << msg.right, msg.left;
  ekf_.measurementUpdateEncoders(z);
  publishState();
}

// Imu callback function
void EkfNode::imuSubCB(const sensor_msgs::Imu& msg){
  ekf_.systemUpdate(dt(msg.header.stamp));
  double z = msg.angular_velocity.z;
  ekf_.measurementUpdateIMU(z);
  publishState();
}

  // Determine time since the last time dt() was called.
double EkfNode::dt(ros::Time currentTime){
  ros::Duration dt;
  dt = currentTime - lastTime_;
  lastTime_ = currentTime;
  return dt.toSec();
}


  // Publish the state as an odom message on the topic odom_ekf. Alos well broadcast a transform.
void EkfNode::publishState(){
  // Create an Odometry message to publish
  nav_msgs::Odometry state_msg;

  // Populate timestamp, position frame, and velocity frame
  state_msg.header.stamp = ros::Time::now();
  state_msg.header.frame_id = "map";
  state_msg.child_frame_id = "base_link";

  // Populate the position and orientation
  state_msg.pose.pose.position.x = ekf_.state_(0); // x
  state_msg.pose.pose.position.y = ekf_.state_(1); // y
  state_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(ekf_.state_(2)); // theta

  boost::array<double,36> temp;
  // Populate the covariance matrix
  state_msg.pose.covariance = temp;

  // Populate the linear and angular velocities
  state_msg.twist.twist.linear.x = ekf_.state_(3); // v
  state_msg.twist.twist.angular.z = ekf_.state_(4); // omega

  // Populate the covariance matrix
  state_msg.twist.covariance = temp;

  // Publish the message!
  statePub_.publish(state_msg);

  // Now for the transform
  // ...
} 

/* Constructor */
EkfNode::EkfNode(): private_nh_("~") {

  // Create a publisher object to publish the determined state of the robot. Odometry messages contain both Pose and Twist with covariance. In this simulator, we will not worry about the covariance.
  statePub_ = public_nh_.advertise<nav_msgs::Odometry>("odom_ekf",1);

  // Create a subscriber object to subscribe to the topic 
  dwSub_ = public_nh_.subscribe("dw_beacons",1,&EkfNode::dwSubCB,this);
  imuSub_ = public_nh_.subscribe("imu/data",1,&EkfNode::imuSubCB,this);
  encSub_ = public_nh_.subscribe("enc",1,&EkfNode::encSubCB,this);
}

/* Destructor */
EkfNode::~EkfNode() {

}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "ekf_node");
  // Create an EkfNode Object
  EkfNode ekfNode;
  // And spin
  ros::spin();

  return 0;
}
