/******************************************************************************
ekf_node.cpp
The EkfNode class interfaces an Ekf class with ROS communication functions.
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
  double dtSys = dtSystem(msg.header.stamp);
  Vector4d z;
  z << msg.dist[1], msg.dist[2], msg.dist[3],msg.dist[4];
  ekf_.systemUpdate(dtSys);
  ekf_.measurementUpdateDecaWave(z);
  publishState();
}

// Wheel Encoder callback function
void EkfNode::encSubCB(const snowmower_msgs::EncMsg& msg){
  double dtSys = dtSystem(msg.header.stamp);
  double dtEnc = dtEncoder(msg.header.stamp);
  ROS_INFO_STREAM("dtSys = " << dtSys);
  ROS_INFO_STREAM("dtEnc = " << dtEnc);
  Vector2i z;
  z << msg.right, msg.left;
  // If first Run, only initialize zPre_, lastSysTime_, and lastEncTime_.
  if (!firstRunEnc_) {
    ekf_.systemUpdate(dtSys);
    ekf_.measurementUpdateEncoders(z, zPre_, dtEnc);
    publishState();
  }
  // Store current measurement as zPre_
  zPre_ = z;
  firstRunEnc_ = false;
}

// Imu callback function
void EkfNode::imuSubCB(const sensor_msgs::Imu& msg){
  double dtSys = dtSystem(msg.header.stamp);
  double z = msg.angular_velocity.z;
  ekf_.systemUpdate(dtSys);
  ekf_.measurementUpdateIMU(z);
  publishState();
}

// Determine time since the last time dtSystem() was called.
double EkfNode::dtSystem(ros::Time currentSysTime){
  ros::Duration dtSys;
  dtSys = currentSysTime - lastSysTime_;
  lastSysTime_ = currentSysTime;
  return dtSys.toSec();
}

// Determine time since the last time dtEnc() was called.
double EkfNode::dtEncoder(ros::Time currentEncTime){
  ros::Duration dtEnc;
  dtEnc = currentEncTime - lastEncTime_;
  lastEncTime_ = currentEncTime;
  return dtEnc.toSec();
}


  // Publish the state as an odom message on the topic odom_ekf. Alos well broadcast a transform.
void EkfNode::publishState(){
  // Create an Odometry message to publish
  nav_msgs::Odometry state_msg;

  // Populate timestamp, position frame, and velocity frame
  state_msg.header.stamp = ros::Time::now();
  state_msg.header.frame_id = map_frame_;
  state_msg.child_frame_id = base_frame_;

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
  // Print for debugging purposes
  ROS_INFO_STREAM("\nstate = \n" << ekf_.state_);
  ROS_INFO_STREAM("\ncovariance = \n" << ekf_.cov_);
  // Now for the transform
  // ...
} 

  // Initialization process
void EkfNode::init() {
  // Set first run to true for encoders. Once a message is received, this will
  // be set to false.
  firstRunEnc_ = true;
  // Set Times
  ros::Time currTime = ros::Time::now();
  lastEncTime_ = currTime;
  lastSysTime_ = currTime;
  // Use the ROS parameter server to initilize parameters
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";

  // Create temp array to initialize state
  double state_temp[] = {0, 0, 0, 0, 0, 0};
  // And a std::vector, which will be used to initialize an Eigen Matrix
  std::vector<double> state (state_temp, state_temp + sizeof(state_temp) / sizeof(double));
  // Check the parameter server and initialize state
  if(!private_nh_.getParam("state", state)) {
    ROS_WARN_STREAM("No state found. Using default.");
  }
  // Check to see if the size is not equal to 6
  if (state.size() != 6) {
    ROS_WARN_STREAM("state isn't 6 elements long!");
  }
  // And initialize the Matrix
  typedef Matrix<double, 6, 1> Vector6d;
  Vector6d stateMat(state.data());
  std::cout << "state = " << std::endl;
  std::cout << stateMat << std::endl;
  ekf_.initState(stateMat);

  // Create temp array to initialize covariance
  double cov_temp[] = {0.01, 0, 0, 0, 0, 0,
		       0, 0.01, 0, 0, 0, 0,
		       0, 0, 0.01, 0, 0, 0,
		       0, 0, 0, 0.01, 0, 0,
		       0, 0, 0, 0, 0.01, 0,
		       0, 0, 0, 0, 0, 0.01};
  // And a std::vector, which will be used to initialize an Eigen Matrix
  std::vector<double> cov (cov_temp, cov_temp + sizeof(cov_temp) / sizeof(double));
  // Check the parameter server and initialize cov
  if(!private_nh_.getParam("covariance", cov)) {
    ROS_WARN_STREAM("No covariance found. Using default.");
  }
  // Check to see if the size is not equal to 36
  if (cov.size() != 36) {
    ROS_WARN_STREAM("cov isn't 36 elements long!");
  }
  // And initialize the Matrix
  typedef Matrix<double, 6, 6, RowMajor> Matrix66;
  Matrix66 covMat(cov.data());
  std::cout << "covariance = " << std::endl;
  std::cout << covMat << std::endl;
  ekf_.initCov(covMat);

  // Create temp array to initialize Q
  double Q_temp[] = {0.01, 0, 0, 0, 0, 0,
		     0, 0.01, 0, 0, 0, 0,
		     0, 0, 0.01, 0, 0, 0,
		     0, 0, 0, 0.01, 0, 0,
		     0, 0, 0, 0, 0.01, 0,
		     0, 0, 0, 0, 0, 0.01};
  // And a std::vector, which will be used to initialize an Eigen Matrix
  std::vector<double> Q (Q_temp, Q_temp + sizeof(Q_temp) / sizeof(double));
  // Check the parameter server and initialize Q
  if(!private_nh_.getParam("Q", Q)) {
    ROS_WARN_STREAM("No Q found. Using default.");
  }
  // Check to see if the size is not equal to 36
  if (Q.size() != 36) {
    ROS_WARN_STREAM("Q isn't 36 elements long!");
  }
  // And initialize the Matrix
  Matrix66 QMat(Q.data());
  std::cout << "Q = " << std::endl;
  std::cout << QMat << std::endl;
  ekf_.initSystem(QMat);

  // Create temp array to initialize R for DecaWave
  double RDW_temp[] = {0.01, 0, 0, 0,
		       0, 0.01, 0, 0,
		       0, 0, 0.01, 0,
		       0, 0, 0, 0.01};
  // And a std::vector, which will be used to initialize an Eigen Matrix
  std::vector<double> RDW (RDW_temp, RDW_temp + sizeof(RDW_temp) / sizeof(double));
  // Check the parameter server and initialize RDW
  if(!private_nh_.getParam("DW_R", RDW)) {
    ROS_WARN_STREAM("No DW_R found. Using default.");
  }
  // Check to see if the size is not equal to 16
  if (RDW.size() != 16) {
    ROS_WARN_STREAM("DW_R isn't 16 elements long!");
  }
  // And initialize the Matrix
  typedef Matrix<double, 4, 4, RowMajor> Matrix44;
  Matrix44 RDWMat(RDW.data());
  std::cout << "RDecaWave = " << std::endl;
  std::cout << RDWMat << std::endl;

  // Create temp array to initialize beacon locations for DecaWave
  double DWBL_temp[] = {0, 0,
		       5, 0,
		       5, 15,
		       0, 15};
  // And a std::vector, which will be used to initialize an Eigen Matrix
  std::vector<double> DWBL (DWBL_temp, DWBL_temp + sizeof(DWBL_temp) / sizeof(double));
  // Check the parameter server and initialize DWBL
  if(!private_nh_.getParam("DW_Beacon_Loc", DWBL)) {
    ROS_WARN_STREAM("No DW_Beacon_Loc found. Using default.");
  }
  // Check to see if the size is not equal to 8
  if (DWBL.size() != 8) {
    ROS_WARN_STREAM("DW_Beacon_Loc isn't 8 elements long!");
  }
  // And initialize the Matrix
  typedef Matrix<double, 4, 2, RowMajor> Matrix42;
  Matrix42 DWBLMat(DWBL.data());
  std::cout << "DW_Beacon_Loc = " << std::endl;
  std::cout << DWBLMat << std::endl;

  MatrixXd DecaWaveBeaconLoc(4,2);
  MatrixXd DecaWaveOffset(2,1);
  double DW_offset_x;
  double DW_offset_y;
  if(!private_nh_.getParam("DW_offset_x", DW_offset_x))
    DW_offset_x = 0.0;
  if(!private_nh_.getParam("DW_offset_y", DW_offset_y))
    DW_offset_y = 0.0;
  DecaWaveOffset << DW_offset_x, DW_offset_y;
  std::cout << "DecaWaveOffset = " << std::endl;
  std::cout << DecaWaveOffset << std::endl;

  ekf_.initDecaWave(RDWMat, DWBLMat, DecaWaveOffset);

  // Create temp array to initialize R for DecaWave
  double REnc_temp[] = {0.01, 0,
			0, 0.01};
  // And a std::vector, which will be used to initialize an Eigen Matrix
  std::vector<double> REnc (REnc_temp, REnc_temp + sizeof(REnc_temp) / sizeof(double));
  // Check the parameter server and initialize RDW
  if(!private_nh_.getParam("Enc_R", REnc)) {
    ROS_WARN_STREAM("No Enc_R found. Using default.");
  }
  // Check to see if the size is not equal to 4
  if (REnc.size() != 4) {
    ROS_WARN_STREAM("Enc_R isn't 4 elements long!");
  }
  // And initialize the Matrix
  typedef Matrix<double, 2, 2, RowMajor> Matrix22;
  Matrix22 REncMat(REnc.data());
  std::cout << "R_Enc = " << std::endl;
  std::cout << REncMat << std::endl;

  double b, tpmRight, tpmLeft;
  if(!private_nh_.getParam("track_width", b))
    b = 1;
  if(!private_nh_.getParam("tpm_right", tpmRight))
    tpmRight = 1;
  if(!private_nh_.getParam("tpm_left", tpmLeft))
    tpmLeft = 1;
  ekf_.initEnc(REncMat, b, tpmRight, tpmLeft);
  std::cout << "track_width = " << b << std::endl;
  std::cout << "tpm_right   = " << tpmRight << std::endl;
  std::cout << "tpm_left    = " << tpmLeft << std::endl;

  double RIMU;
  if(!private_nh_.getParam("R_IMU", RIMU))
    RIMU = 0.01;
  ekf_.initIMU(RIMU);
  std::cout << "R_IMU = " << RIMU << std::endl;

  ROS_INFO_STREAM("Finished with initialization.");
}


/* Constructor */
EkfNode::EkfNode(): private_nh_("~") {

  // Create a publisher object to publish the determined state of the robot. Odometry messages contain both Pose and Twist with covariance. In this simulator, we will not worry about the covariance.
  statePub_ = public_nh_.advertise<nav_msgs::Odometry>("odom_ekf",1);

  // Create a subscriber object to subscribe to the topic 
  dwSub_ = public_nh_.subscribe("dw_beacons",1,&EkfNode::dwSubCB,this);
  imuSub_ = public_nh_.subscribe("imu/data",1,&EkfNode::imuSubCB,this);
  encSub_ = public_nh_.subscribe("enc",1,&EkfNode::encSubCB,this);

  // Wait for time to not equal zero. A zero time means that no message has
  // been received on the /clock topic
  ros::Time timeZero(0.0);
  while (ros::Time::now() == timeZero) { }
  // Sleep for a small time to make sure publishing and subscribing works.
  ros::Duration(0.1).sleep();
  // Initialize 
  init();
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
