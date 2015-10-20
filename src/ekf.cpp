#include <ros/ros.h>
#include <math.h>
#include <Eigen/LU>
#include "snowmower_localization/ekf.h"

// System update
void Ekf::systemUpdate(){
  // Update the state estimate using x_p (the vector) and u
  double x = state_(1);
  double y = state_(2);
  double theta = state_(3);
  double v = state_(4);
  double omega = state_(5);

  // Update the State

  ////////////////////////////////////////////////////////////
  // First determine the correction factor. This is described in Wang 1988. The limit of cf as omega approaches 0 is 1. The following if statement protects against dividing by zero.
  double cf;
  if (omega == 0){
    cf = 1;
  }
  else{
    cf = sin(omega*dt_/2)/(omega*dt_/2);
  }
  
  // deltaX and deltaY are used a lot later. So, it makes sense to calculate them once now.
  double deltaX = cf*v*dt_*cos(theta+omega*dt_/2);
  double deltaY = cf*v*dt_*sin(theta+omega*dt_/2);

  x = x + deltaX;
  y = y + deltaY;
  theta = theta + omega*dt_;
  v = v;
  omega = omega;

  state_ << x, y, theta, v, omega;

  // Calculate F (using the updated state variables)
  double F13 =  -2*v*sin(omega*dt_/2)*sin(omega*dt_/2+theta)/omega;
  double F14 =   2 * sin(omega*dt_/2)*cos(omega*dt_/2+theta)/omega;
  double F23 =   2*v*sin(omega*dt_/2)*cos(omega*dt_/2+theta)/omega;
  double F24 =   2 * sin(omega*dt_/2)*sin(omega*dt_/2+theta)/omega;
  double F15 = v*dt_*cos(omega *dt_/2)*cos(omega*dt_/2+theta)/omega - 
                 2*v*sin(omega*dt_/2)*cos(omega*dt_/2+theta)/pow(omega,2) -
	       v*dt_*sin(omega*dt_/2)*sin(omega*dt_/2+theta)/omega;
  double F25 = v*dt_*sin(omega*dt_/2)*cos(omega*dt_/2+theta)/omega - 
                 2*v*sin(omega*dt_/2)*sin(omega*dt_/2+theta)/pow(omega,2) -
	       v*dt_*cos(omega*dt_/2)*sin(omega*dt_/2+theta)/omega;
  // And construct the final matrix
  MatrixXd F(5,5);
  F << 1,   0, F13, F14, F15,
       0,   1, F23, F24, F25,
       0,   0,   1,   0, dt_,
       0,   0,   0,   1,   0,
       0,   0,   0,   0,   1;

  cov_ = F*cov_*F.transpose() + Q_;
}


Vector4d  Ekf::hDecaWave(Vector5d state) {
  double d1 = sqrt(pow(dw1x_-state(1),2)+pow(dw1y_-state(2),2));
  double d2 = sqrt(pow(dw2x_-state(1),2)+pow(dw2y_-state(2),2));
  double d3 = sqrt(pow(dw3x_-state(1),2)+pow(dw3y_-state(2),2));
  double d4 = sqrt(pow(dw4x_-state(1),2)+pow(dw4y_-state(2),2));
  Vector4d z;
  z << d1, d2, d3, d4;
  return z;
}

void Ekf::measurementUpdateDecaWave(Vector4d z){
  double x = state_(1);
  double y = state_(2);

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
  MatrixXd K(5,5);
  K = cov_*H.transpose()*(H*cov_*H.transpose()+RDecaWave_).inverse();

  // Find new state
  state_ = state_ + K*(z - hDecaWave(state_));

  // Find new covariance
  cov_ = cov_ - K*H*cov_;
};


/* Constructor */
Ekf::Ekf(): private_nh_("~") {
  // Set dt_
  dt_ = 0.001;

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
