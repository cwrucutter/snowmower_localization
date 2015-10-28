/******************************************************************************
ekf.cpp
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
#include <math.h>
#include <boost/array.hpp>
#include <Eigen/LU>
#include "snowmower_localization/ekf.h"
#include <iostream>

typedef Matrix<double, 5, 1> Vector5d;
typedef Matrix<double, 5, 5> Matrix5d;
typedef Matrix<double, 1, 5> Matrix15;
typedef Matrix<double, 2, 5> Matrix25;
typedef Matrix<double, 5, 2> Matrix52;
typedef Matrix<double, 1, 1> Matrix11;
typedef Matrix<double, 4, 5> Matrix45;
typedef Matrix<double, 5, 4> Matrix54;
typedef Matrix<double, 4, 2> Matrix42;
/***********************
 Equations for System
 1. f(x,dt)
 2. F(x,dt)
 3. System update
***********************/
// 1. f(x,dt)
Vector5d Ekf::fSystem(Vector5d state, double dt){
  // Update the state estimate using x_p (the vector) and u
  double x = state(0);
  double y = state(1);
  double theta = state(2);
  double v = state(3);
  double omega = state(4);

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

  MatrixXd stateNew(5,1);
  stateNew << x, y, theta, v, omega;
  return stateNew;
}

// 2. F(x,dt)
Matrix5d Ekf::FSystem(Vector5d state, double dt){
  double x = state(0);
  double y = state(1);
  double theta = state(2);
  double v = state(3);
  double omega = state(4);

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
  return F;
}

// 3. System update
void Ekf::systemUpdate(double dt){
  // update state
  state_ = fSystem(state_,dt);
  // Then calculate F
  MatrixXd F(5,5);
  F = FSystem(state_,dt);
  // Then update covariance
  cov_ = F*cov_*F.transpose() + Q_;
}

/***********************
 Equations for DecaWave
 1. h(x)
 2. H(x)
 3. Kalman Gain, K
 4. State update
 5. Covariance update
 6. Total measurement update
***********************/

// 1. h(x)
Vector4d  Ekf::hDecaWave(Vector5d state, Matrix42 DecaWaveBeaconLoc,
			 Vector2d DecaWaveOffset) {
  // Break out matricies for easier to read equations below
  double x = state(0);
  double y = state(1);
  double theta = state(2);
  double dw1x = DecaWaveBeaconLoc(0,0);
  double dw1y = DecaWaveBeaconLoc(0,1);
  double dw2x = DecaWaveBeaconLoc(1,0);
  double dw2y = DecaWaveBeaconLoc(1,1);
  double dw3x = DecaWaveBeaconLoc(2,0);
  double dw3y = DecaWaveBeaconLoc(2,1);
  double dw4x = DecaWaveBeaconLoc(3,0);
  double dw4y = DecaWaveBeaconLoc(3,1);
  double xOff = DecaWaveOffset(0);
  double yOff = DecaWaveOffset(1);

  double d1 = sqrt(pow(dw1x-x,2)+pow(dw1y-y,2));
  double d2 = sqrt(pow(dw2x-x,2)+pow(dw2y-y,2));
  double d3 = sqrt(pow(dw3x-x,2)+pow(dw3y-y,2));
  double d4 = sqrt(pow(dw4x-x,2)+pow(dw4y-y,2));
  Vector4d h;
  h << d1, d2, d3, d4;
  return h;
}

// 2. H(x)
Matrix45 Ekf::HDecaWave(Vector5d state, Matrix42 DecaWaveBeaconLoc,
			 Vector2d DecaWaveOffset) {
  // Break out matricies for easier to read equations below
  double x = state(0);
  double y = state(1);
  double theta = state(2);
  double dw1x = DecaWaveBeaconLoc(0,0);
  double dw1y = DecaWaveBeaconLoc(0,1);
  double dw2x = DecaWaveBeaconLoc(1,0);
  double dw2y = DecaWaveBeaconLoc(1,1);
  double dw3x = DecaWaveBeaconLoc(2,0);
  double dw3y = DecaWaveBeaconLoc(2,1);
  double dw4x = DecaWaveBeaconLoc(3,0);
  double dw4y = DecaWaveBeaconLoc(3,1);
  double xOff = DecaWaveOffset(0);
  double yOff = DecaWaveOffset(1);

  // Calculate H
  double H11 = -pow(pow(dw1x-x,2)+pow(dw1y-y,2),-0.5)*(dw1x-x);
  double H12 = -pow(pow(dw1x-x,2)+pow(dw1y-y,2),-0.5)*(dw1y-y);
  double H21 = -pow(pow(dw2x-x,2)+pow(dw2y-y,2),-0.5)*(dw2x-x);
  double H22 = -pow(pow(dw2x-x,2)+pow(dw2y-y,2),-0.5)*(dw2y-y);
  double H31 = -pow(pow(dw3x-x,2)+pow(dw3y-y,2),-0.5)*(dw3x-x);
  double H32 = -pow(pow(dw3x-x,2)+pow(dw3y-y,2),-0.5)*(dw3y-y);
  double H41 = -pow(pow(dw4x-x,2)+pow(dw4y-y,2),-0.5)*(dw4x-x);
  double H42 = -pow(pow(dw4x-x,2)+pow(dw4y-y,2),-0.5)*(dw4y-y);

  MatrixXd H(4,5);
  H << H11, H12, 0, 0, 0,
       H21, H22, 0, 0, 0,
       H31, H32, 0, 0, 0,
       H41, H42, 0, 0, 0;

  return H;
}

// 3. Kalman Gain, K
Matrix54 Ekf::KDecaWave(Matrix5d cov, Matrix45 H, Matrix4d R){
  // Find Kalman Gain
  MatrixXd K(5,4);
  K = cov*H.transpose()*(H*cov*H.transpose()+R).inverse();
  return K;
}

// 4. State update
Vector5d Ekf::stateUpdateDecaWave(Vector5d state, Matrix54 K, Vector4d z, Vector4d h){
  // Find new state
  VectorXd stateNew(5,1);
  stateNew = state + K*(z - h);
  return stateNew;
}

// 5. Covariance update
Matrix5d Ekf::covUpdateDecaWave(Matrix5d cov, Matrix54 K, Matrix45 H){
  // Find new covariance
  MatrixXd covNew(5,5);
  covNew = cov - K*H*cov;
  return covNew;
}


// 6. Total measurement update
void Ekf::measurementUpdateDecaWave(Vector4d z){
  // Determine h and H
  Vector4d h;
  h = hDecaWave(state_, DecaWaveBeaconLoc_, DecaWaveOffset_);
  MatrixXd H(4,5);
  H = HDecaWave(state_, DecaWaveBeaconLoc_, DecaWaveOffset_);
  // Find Kalman Gain
  MatrixXd K(5,4);
  K = KDecaWave(cov_, H, RDecaWave_);
  // Find new state
  state_ = stateUpdateDecaWave(state_, K, z, h);
  // Find new covariance
  cov_ = covUpdateDecaWave(cov_, K, H);
}

/***********************
 Equations for Encoders
 1. h(x)
 2. H(x)
 3. Kalman Gain, K
 4. State update
 5. Covariance update
 6. Total measurement update
***********************/

// 1. h(x)
Vector2d Ekf::hEnc(Vector5d state, double b, double tpmRight, double tpmLeft){
  Vector2d h;
  double h1 = state(3)+b_/2*state(4); // v+(b/2)*omega
  double h2 = state(3)-b_/2*state(4); // v-(b/2)*omega
  h << h1, h2;
  return h;
}

// 2. H(x)
Matrix25 Ekf::HEnc(Vector5d state, double b, double tpmRight, double tpmLeft){
  // Calculate H
  double H15 = b_/2;
  double H25 = -b_/2;

  MatrixXd H(2,5);
  H << 0, 0, 0, 1, H15,
       0, 0, 0, 1, H25;
  return H;
}

// 3. Kalman Gain, K
Matrix52 Ekf::KEnc(Matrix5d cov, Matrix25 H, Matrix2d R){
  // Find Kalman Gain
  MatrixXd K(5,2);
  K = cov*H.transpose()*(H*cov*H.transpose()+R).inverse();
  return K;
}

// 4. State update
Vector5d Ekf::stateUpdateEnc(Vector5d state, Matrix52 K, Vector2d z, Vector2d h){
  // Find new state
  MatrixXd stateNew(5,1);
  stateNew = state + K*(z - h);
  return stateNew;
}

// 5. Covariance update
Matrix5d Ekf::covUpdateEnc(Matrix5d cov, Matrix52 K, Matrix25 H){
  // Find new covariance
  MatrixXd covNew(5,5);
  covNew = cov - K*H*cov;
  return covNew;
}


// 6. Total measurement update
void Ekf::measurementUpdateEncoders(Vector2d z){ // z is encL and encR
  // Determine h and H
  Vector2d h;
  h = hEnc(state_, b_, tpmRight_, tpmLeft_);
  MatrixXd H(2,5);
  H = HEnc(state_, b_, tpmRight_, tpmLeft_);
  // Find Kalman Gain
  MatrixXd K(5,2);
  K = KEnc(cov_, H, REnc_);
  // Find new state
  state_ = stateUpdateEnc(state_, K, z, h);
  // Find new covariance
  cov_ = covUpdateEnc(cov_, K, H);
}

/***********************
 Equations for IMU
 1. h(x)
 2. H(x)
 3. Kalman Gain, K
 4. State update
 5. Covariance update
 6. Total measurement update
***********************/

// 1. h(x)
double Ekf::hIMU(Vector5d state){
  return state(4); // return omega
}

// 2. H(x)
Matrix15 Ekf::HIMU(Vector5d state){
  MatrixXd H(1,5);
  H << 0, 0, 0, 0, 1;
  return H;
}

// 3. Kalman Gain, K
Vector5d Ekf::KIMU(Matrix5d cov, Matrix15 H, double R){
  // Find Kalman Gain
  // Must create a 1x1 matrix container for the double RIMU_
  MatrixXd RMatrix(1,1);
  RMatrix << R;
  MatrixXd K(5,1);
  K = cov*H.transpose()*(H*cov*H.transpose()+RMatrix).inverse();
  return K;
}

// 4. State update
Vector5d Ekf::stateupdateIMU(Vector5d state, Vector5d K, double z, double h){
  // Find new state
  MatrixXd stateNew(5,1);
  stateNew = state + K*(z - h);
  return stateNew;
}

// 5. Covariance update
Matrix5d Ekf::covUpdateIMU(Matrix5d cov, Vector5d K, Matrix15 H){
  // Find new covariance
  MatrixXd covNew(5,5);
  covNew = cov - K*H*cov;
  return covNew;
}

// 3. Total measurement update
void Ekf::measurementUpdateIMU(double z){ // z is omega_z
  // Determine h and H
  double h;
  h = hIMU(state_);
  MatrixXd H(1,5);
  H = HIMU(state_);
  // Find Kalman Gain
  MatrixXd K(5,1);
  K = KIMU(cov_, H, RIMU_);
  // Find new state
  state_ = stateupdateIMU(state_, K, z, h);
  // Find new covariance
  cov_ = covUpdateIMU(cov_, K, H);
}

void Ekf::initState(Vector5d state) {
  state_ = state;
}
void Ekf::initCov(Matrix5d cov) {
  cov_ = cov;
}
void Ekf::initSystem(Matrix5d Q) {
  Q_ = Q;
}
void Ekf::initDecaWave(Matrix4d R, Matrix42 DecaWaveBeaconLoc,
		       Vector2d DecaWaveOffset) {
  RDecaWave_ = R;
  DecaWaveBeaconLoc_ = DecaWaveBeaconLoc;
  DecaWaveOffset_ = DecaWaveOffset;
}

void Ekf::initEnc(Matrix2d R, double b, double tpmRight, double tpmLeft) {
  REnc_ = R;
  b_ = b;
  tpmRight_ = tpmRight;
  tpmLeft_ = tpmLeft;
}

void Ekf::initIMU(double R) {
  RIMU_ = R;
}


/* Constructor */
Ekf::Ekf() {

};

/* Destructor */
Ekf::~Ekf() {

};
