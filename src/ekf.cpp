#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <boost/array.hpp>
#include <Eigen/LU>
#include "snowmower_localization/ekf.h"

// System update
void Ekf::systemUpdate(double dt){
  // Update the state estimate using x_p (the vector) and u
  double x = state_(0);
  double y = state_(1);
  double theta = state_(2);
  double v = state_(3);
  double omega = state_(4);

  // Update the State

  ////////////////////////////////////////////////////////////
  // First determine the correction factor. This is described in Wang 1988. The limit of cf as omega approaches 0 is 1. The following if statement protects against dividing by zero.
  double cf;
  if (omega == 0){
    cf = 1;
  }
  else{
    cf = sin(omega*dt/2)/(omega*dt/2);
  }
  
  // deltaX and deltaY are used a lot later. So, it makes sense to calculate them once now.
  double deltaX = cf*v*dt*cos(theta+omega*dt/2);
  double deltaY = cf*v*dt*sin(theta+omega*dt/2);
  double deltaTheta = omega*dt;

  x = x + deltaX;
  y = y + deltaY;
  theta = theta + deltaTheta;
  v = v;
  omega = omega;

  state_ << x, y, theta, v, omega;

  // Calculate F (using the updated state variables)
  double F13 =  -2*v*sin(omega*dt/2)*sin(omega*dt/2+theta)/omega;
  double F14 =   2 * sin(omega*dt/2)*cos(omega*dt/2+theta)/omega;
  double F23 =   2*v*sin(omega*dt/2)*cos(omega*dt/2+theta)/omega;
  double F24 =   2 * sin(omega*dt/2)*sin(omega*dt/2+theta)/omega;
  double F15 =  v*dt*cos(omega*dt/2)*cos(omega*dt/2+theta)/omega - 
                 2*v*sin(omega*dt/2)*cos(omega*dt/2+theta)/pow(omega,2) -
	        v*dt*sin(omega*dt/2)*sin(omega*dt/2+theta)/omega;
  double F25 =  v*dt*sin(omega*dt/2)*cos(omega*dt/2+theta)/omega - 
                 2*v*sin(omega*dt/2)*sin(omega*dt/2+theta)/pow(omega,2) -
	        v*dt*cos(omega*dt/2)*sin(omega*dt/2+theta)/omega;
  // And construct the final matrix
  MatrixXd F(5,5);
  F << 1,   0, F13, F14, F15,
       0,   1, F23, F24, F25,
       0,   0,   1,   0,  dt,
       0,   0,   0,   1,   0,
       0,   0,   0,   0,   1;

  cov_ = F*cov_*F.transpose() + Q_;
}

/***********************
 Equations for DecaWave
 1. h(x)
 2. measurement update
 3. callback function
***********************/

// 1. h(x)
Vector4d  Ekf::hDecaWave(Vector5d state) {
  double d1 = sqrt(pow(dw1x_-state(0),2)+pow(dw1y_-state(1),2));
  double d2 = sqrt(pow(dw2x_-state(0),2)+pow(dw2y_-state(1),2));
  double d3 = sqrt(pow(dw3x_-state(0),2)+pow(dw3y_-state(1),2));
  double d4 = sqrt(pow(dw4x_-state(0),2)+pow(dw4y_-state(1),2));
  Vector4d h;
  h << d1, d2, d3, d4;
  return h;
}

// 2. measurement update
void Ekf::measurementUpdateDecaWave(Vector4d z){
  double x = state_(0);
  double y = state_(1);

  // Calculate H
  double H11 = -pow(pow(dw1x_-x,2)+pow(dw1y_-y,2),-0.5)*(dw1x_-x);
  double H12 = -pow(pow(dw1x_-x,2)+pow(dw1y_-y,2),-0.5)*(dw1y_-y);
  double H21 = -pow(pow(dw2x_-x,2)+pow(dw2y_-y,2),-0.5)*(dw2x_-x);
  double H22 = -pow(pow(dw2x_-x,2)+pow(dw2y_-y,2),-0.5)*(dw2y_-y);
  double H31 = -pow(pow(dw3x_-x,2)+pow(dw3y_-y,2),-0.5)*(dw3x_-x);
  double H32 = -pow(pow(dw3x_-x,2)+pow(dw3y_-y,2),-0.5)*(dw3y_-y);
  double H41 = -pow(pow(dw4x_-x,2)+pow(dw4y_-y,2),-0.5)*(dw4x_-x);
  double H42 = -pow(pow(dw4x_-x,2)+pow(dw4y_-y,2),-0.5)*(dw4y_-y);

  MatrixXd H(4,5);
  H << H11, H12, 0, 0, 0,
       H21, H22, 0, 0, 0,
       H31, H32, 0, 0, 0,
       H41, H42, 0, 0, 0;

  // Find Kalman Gain
  MatrixXd K(5,4);
  K = cov_*H.transpose()*(H*cov_*H.transpose()+RDecaWave_).inverse();

  // Find new state
  state_ = state_ + K*(z - hDecaWave(state_));

  // Find new covariance
  cov_ = cov_ - K*H*cov_;
}

// 3. callback function
void Ekf::dwSubCB(const snowmower_msgs::DecaWaveMsg& msg){

}

/***********************
 Equations for Encoders
 1. h(x)
 2. measurement update
 3. callback function
***********************/

// 1. h(x)
Vector2d Ekf::hEnc(Vector5d state){
  Vector2d h;
  double h1 = state(3)+b_/2*state(4); // v+(b/2)*omega
  double h2 = state(3)-b_/2*state(4); // v-(b/2)*omega
  h << h1, h2;
  return h;
}

// 2. measurement update
void Ekf::measurementUpdateEncoders(Vector2d z){ // z is encL and encR
  // Calculate H
  double H15 = b_/2;
  double H25 = -b_/2;

  MatrixXd H(2,5);
  H << 0, 0, 0, 1, H15,
       0, 0, 0, 1, H25;

  // Find Kalman Gain
  MatrixXd K(5,2);
  K = cov_*H.transpose()*(H*cov_*H.transpose()+RDecaWave_).inverse();

  // Find new state
  state_ = state_ + K*(z - hEnc(state_));

  // Find new covariance
  cov_ = cov_ - K*H*cov_;
}

// 3. callback function
void Ekf::encSubCB(const snowmower_msgs::EncMsg& msg){

}

/***********************
 Equations for IMU
 1. h(x)
 2. measurement update
 3. callback function
***********************/

// 1. h(x)
double Ekf::hIMU(Vector5d state){
  return state(4); // return omega
}

void Ekf::measurementUpdateIMU(double z){ // z is omega_z

  MatrixXd H(1,5);
  H << 0, 0, 0, 0, 1;
  // Find Kalman Gain
  MatrixXd K(5,1);
  // Must create a 1x1 matrix container for the double RIMU_
  MatrixXd R(1,1);
  R << RIMU_;
  K = cov_*H.transpose()*(H*cov_*H.transpose()+R).inverse();

  // Find new state
  state_ = state_ + K*(z - hIMU(state_));

  // Find new covariance
  cov_ = cov_ - K*H*cov_;
}

void Ekf::imuSubCB(const sensor_msgs::Imu& msg){

}

  // Determine time since the last time dt() was called.
double Ekf::dt(ros::Time currentTime){
  ros::Duration dt;
  dt = currentTime - lastTime_;
  lastTime_ = currentTime;
  return dt.toSec();
}


  // Publish the state as an odom message on the topic odom_ekf. Alos well broadcast a transform.
void Ekf::publishState(){
  // Create an Odometry message to publish
  nav_msgs::Odometry state_msg;

  // Populate timestamp, position frame, and velocity frame
  state_msg.header.stamp = ros::Time::now();
  state_msg.header.frame_id = "map";
  state_msg.child_frame_id = "base_link";

  // Populate the position and orientation
  state_msg.pose.pose.position.x = state_(0); // x
  state_msg.pose.pose.position.y = state_(1); // y
  state_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state_(2)); // theta

  boost::array<double,36> temp;
  // Populate the covariance matrix
  state_msg.pose.covariance = temp;

  // Populate the linear and angular velocities
  state_msg.twist.twist.linear.x = state_(3); // v
  state_msg.twist.twist.angular.z = state_(4); // omega

  // Populate the covariance matrix
  state_msg.twist.covariance = temp;

  // Publish the message!
  statePub_.publish(state_msg);

  // Now for the transform

} 

void Ekf::init(){
  // System Model Covariance Matrix
  Q_ << 0.01, 0,    0,    0,    0,
        0,    0.01, 0,    0,    0,
        0,    0,    0.01, 0,    0,
        0,    0,    0,    0.01, 0,
        0,    0,    0,    0,    0.01;

  // Decawave Beacon Locations (in meters)
  dw1x_ = 0;
  dw1y_ = 0;
  dw2x_ = 5;
  dw2y_ = 0;
  dw3x_ = 5;
  dw3y_ = 15;
  dw4x_ = 0;
  dw4y_ = 15;
  // Decawave Covariance Matrix
  RDecaWave_ << 0.1, 0.0, 0.0, 0.0,
                0.0, 0.1, 0.0, 0.0,
                0.0, 0.0, 0.1, 0.0,
                0.0, 0.0, 0.0, 0.1;

  // IMU Covariance "Matrix"
  RIMU_ = .01;

  // Wheel Track Width (in meteres)
  b_ = .7;
  // Encoder ticks per revolutions (left and right)
  tpmRight_ = 50000;
  tpmLeft_  = 50000;

  // Initialize lastTime_
  // State should also be initialized
  lastTime_ = ros::Time::now();
}

/* Constructor */
Ekf::Ekf(): private_nh_("~") {

  // Create a publisher object to publish the determined state of the robot. Odometry messages contain both Pose and Twist with covariance. In this simulator, we will not worry about the covariance.
  statePub_ = public_nh_.advertise<nav_msgs::Odometry>("odom_ekf",1);

  // Create a subscriber object to subscribe to the topic 
  dwSub_ = public_nh_.subscribe("dw_beacons",1,&Ekf::dwSubCB,this);
  imuSub_ = public_nh_.subscribe("imu/data",1,&Ekf::imuSubCB,this);
  encSub_ = public_nh_.subscribe("enc",1,&Ekf::encSubCB,this);

  init();
};

/* Destructor */
Ekf::~Ekf() {

};

int main(int argc, char **argv) {

  //Initialize ROS
  ros::init(argc, argv, "ekf");

  // Create an EKF object
  Ekf ekf;

  ros::spin();

  return 0;
}
