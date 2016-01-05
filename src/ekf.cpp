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

#include <math.h>
#include <boost/array.hpp>
#include <Eigen/LU>
#include "snowmower_localization/ekf.h"
#include <iostream>

// (6x1) state_, fSystem, KIMU, stateUpdate
typedef Matrix<double, 6, 1> Vector6d;
// (6x6) cov_, Q_, FSystem, covUpdate 
typedef Matrix<double, 6, 6, RowMajor> Matrix6d; 
// (1x6) HIMU
typedef Matrix<double, 1, 6> Matrix16;
// (2x6) HEnc
typedef Matrix<double, 2, 6, RowMajor> Matrix26;
// (6x2) KEnc
typedef Matrix<double, 6, 2, RowMajor> Matrix62;
// (1x1) not in use (zIMU, hIMU, RIMU_)
typedef Matrix<double, 1, 1> Matrix11;
// (4x6) HDW
typedef Matrix<double, 4, 6, RowMajor> Matrix46;
// (6x4) KDW
typedef Matrix<double, 6, 4, RowMajor> Matrix64;
// (4x2) Beacon (x,y) locations
typedef Matrix<double, 4, 2, RowMajor> Matrix42;
                          // Vector4d  // zDW, hDW
                          // Matrix4d  // RDW_
                          // Vector2d  // zEnc, hEnc
                          // Matrix2d  // REnc

/***********************
 Equations for System
 1. f(x,dt)
 2. F(x,dt)
 3. System update
***********************/
// 1. f(x,dt)
Vector6d Ekf::fSystem(Vector6d state, double dt){
  // Update the state estimate using x_p (the vector) and u
  double x = state(0);
  double y = state(1);
  double theta = state(2);
  double v = state(3);
  double omega = state(4);
  double bias = state(5);

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
  // Force theta to lie in the interval [-PI, PI)
  theta = fmod(theta + M_PI, 2.0 * M_PI) - M_PI;
  if (theta < -M_PI)
    theta += 2.0 * M_PI;
  v = v;
  omega = omega;
  bias = bias;

  Vector6d stateNew;
  stateNew << x, y, theta, v, omega, bias;
  return stateNew;
}

// 2. F(x,dt)
Matrix6d Ekf::FSystem(Vector6d state, double dt){
  double theta = state(2);
  double v = state(3);
  double omega = state(4);

  // First determine the correction factor. This is described in Wang 1988. The limit of cf as omega approaches 0 is 1. The following if statement protects against dividing by zero.
  double cf;
  if (omega == 0){
    cf = dt/2.0;
  }
  else{
    cf = sin(omega*dt/2)/omega;
  }


  // Calculate F (using the updated state variables)
  double F13 =  -2*v*cf*sin(omega*dt/2+theta);
  double F14 =   2 * cf*cos(omega*dt/2+theta);
  double F23 =   2*v*cf*cos(omega*dt/2+theta);
  double F24 =   2 * cf*sin(omega*dt/2+theta);

  double F151 =    v*dt*cos(omega*dt/2)*cos(omega*dt/2+theta)/omega;
  double F152 =   -v*dt*cf*sin(omega*dt/2+theta); 
  double F153 =    -2*v*sin(omega*dt/2)*cos(omega*dt/2+theta)/pow(omega,2);

  // If omega = 0, F151 and F153 are infinite but opposite of each other.
  double F15;
  if (omega == 0) {
    F15 = F152;
  }
  else {
    // F151 and F153 are huge numbers that cancel out. Add them first.
    F15 = F151 + F153;
    // Then add F152, a relatively small number.
    F15 = F15 + F152;
  }

  double F251 =    v*dt*cos(omega*dt/2)*sin(omega*dt/2+theta)/omega;
  double F252 =    v*dt*cf*cos(omega*dt/2+theta);
  double F253 =    -2*v*sin(omega*dt/2)*sin(omega*dt/2+theta)/pow(omega,2);

  // If omega = 0, F251 and F253 are infinite but opposite of each other.
  double F25;
  if (omega == 0) {
    F25 = F252;
  }
  else {
    // F251 and F253 are huge numbers that cancel out. Add them first.
    F25 = F251 + F253;
    // Then add F252, a relatively small number.
    F25 = F25 + F252;
  }

  // And construct the final matrix
  Matrix6d F;
  F << 1,   0, F13, F14, F15,   0,
       0,   1, F23, F24, F25,   0,
       0,   0,   1,   0,  dt,   0,
       0,   0,   0,   1,   0,   0,
       0,   0,   0,   0,   1,   0,
       0,   0,   0,   0,   0,   1; 
  return F;
}

// 3. System update
void Ekf::systemUpdate(double dt){
  // update state
  state_ = fSystem(state_,dt);
  // Then calculate F
  Matrix6d F;
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
Vector4d  Ekf::hDecaWave(Vector6d state, Matrix42 DecaWaveBeaconLoc,
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

  // Calculate tag location based off robot location, theta, and tag offset
  double xTag = x + xOff*cos(theta) - yOff*sin(theta);
  double yTag = y + xOff*sin(theta) + yOff*cos(theta);

  double d1 = sqrt(pow(dw1x-xTag,2)+pow(dw1y-yTag,2));
  double d2 = sqrt(pow(dw2x-xTag,2)+pow(dw2y-yTag,2));
  double d3 = sqrt(pow(dw3x-xTag,2)+pow(dw3y-yTag,2));
  double d4 = sqrt(pow(dw4x-xTag,2)+pow(dw4y-yTag,2));
  Vector4d h;
  h << d1, d2, d3, d4;
  return h;
}

2// 2. H(x)
Matrix46 Ekf::HDecaWave(Vector6d state, Matrix42 DecaWaveBeaconLoc,
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

  // Calculate tag location based off robot location, theta, and tag offset
  double xTag = x + xOff*cos(theta) - yOff*sin(theta);
  double yTag = y + xOff*sin(theta) + yOff*cos(theta);

  // Calculate H
  double H11 = -(dw1x-xTag)/pow(pow(dw1x-xTag,2)+pow(dw1y-yTag,2),0.5);
  double H12 = -(dw1y-yTag)/pow(pow(dw1x-xTag,2)+pow(dw1y-yTag,2),0.5);
  double H13 =  
     ((dw1x-xTag)*( xOff*sin(theta) + yOff*cos(theta))
     +(dw1y-yTag)*(-xOff*cos(theta) + yOff*sin(theta)))
     /pow(pow(dw1x-xTag,2)+pow(dw1y-yTag,2),0.5);

  double H21 = -(dw2x-xTag)/pow(pow(dw2x-xTag,2)+pow(dw2y-yTag,2),0.5);
  double H22 = -(dw2y-yTag)/pow(pow(dw2x-xTag,2)+pow(dw2y-yTag,2),0.5);
  double H23 =  
     ((dw2x-xTag)*( xOff*sin(theta) + yOff*cos(theta))
     +(dw2y-yTag)*(-xOff*cos(theta) + yOff*sin(theta)))
     /pow(pow(dw2x-xTag,2)+pow(dw2y-yTag,2),0.5);

  double H31 = -(dw3x-xTag)/pow(pow(dw3x-xTag,2)+pow(dw3y-yTag,2),0.5);
  double H32 = -(dw3y-yTag)/pow(pow(dw3x-xTag,2)+pow(dw3y-yTag,2),0.5);
  double H33 =  
     ((dw3x-xTag)*( xOff*sin(theta) + yOff*cos(theta))
     +(dw3y-yTag)*(-xOff*cos(theta) + yOff*sin(theta)))
     /pow(pow(dw3x-xTag,2)+pow(dw3y-yTag,2),0.5);

  double H41 = -(dw4x-xTag)/pow(pow(dw4x-xTag,2)+pow(dw4y-yTag,2),0.5);
  double H42 = -(dw4y-yTag)/pow(pow(dw4x-xTag,2)+pow(dw4y-yTag,2),0.5);
  double H43 =  
     ((dw4x-xTag)*( xOff*sin(theta) + yOff*cos(theta))
     +(dw4y-yTag)*(-xOff*cos(theta) + yOff*sin(theta)))
     /pow(pow(dw4x-xTag,2)+pow(dw4y-yTag,2),0.5);

  Matrix46 H;
  H << H11, H12, H13, 0, 0, 0,
       H21, H22, H23, 0, 0, 0,
       H31, H32, H33, 0, 0, 0,
       H41, H42, H43, 0, 0, 0;

  return H;
}

// 3. Kalman Gain, K
Matrix64 Ekf::KDecaWave(Matrix6d cov, Matrix46 H, Matrix4d R){
  // Find Kalman Gain
  Matrix64 K;
  K = cov*H.transpose()*(H*cov*H.transpose()+R).inverse();
  return K;
}

// 4. State update
Vector6d Ekf::stateUpdateDecaWave(Vector6d state, Matrix64 K, Vector4d z, Vector4d h){
  // Find new state
  Vector6d stateNew;
  stateNew = state + K*(z - h);
  return stateNew;
}

// 5. Covariance update
Matrix6d Ekf::covUpdateDecaWave(Matrix6d cov, Matrix64 K, Matrix46 H){
  // Find new covariance
  Matrix6d covNew;
  covNew = cov - K*H*cov;
  return covNew;
}


// 6. Total measurement update
void Ekf::measurementUpdateDecaWave(Vector4d z){
  // Determine h and H
  Vector4d h;
  h = hDecaWave(state_, DecaWaveBeaconLoc_, DecaWaveOffset_);
  Matrix46 H;
  H = HDecaWave(state_, DecaWaveBeaconLoc_, DecaWaveOffset_);
  // Find Kalman Gain
  Matrix64 K;
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
Vector2d Ekf::hEnc(Vector6d state, double b, double tpmRight, double tpmLeft,
		   int ticksPreRight, int ticksPreLeft, double dt){
  Vector2d h;
  double h1 = (tpmRight * dt) * (state(3) + b/2*state(4)) + ticksPreRight;
  double h2 = (tpmLeft * dt) * (state(3) - b/2*state(4)) + ticksPreLeft;
  h << h1, h2;
  return h;
}

// 2. H(x)
Matrix26 Ekf::HEnc(Vector6d state, double b, double tpmRight, double tpmLeft,
		   double dt){
  // Calculate H
  double H14 = tpmRight * dt;
  double H15 = b/2 * tpmRight * dt;
  double H24 = tpmLeft * dt;
  double H25 = -b/2 * tpmLeft *dt;

  Matrix26 H;
  H << 0, 0, 0, H14, H15, 0,
       0, 0, 0, H24, H25, 0;
  return H;
}

// 3. Kalman Gain, K
Matrix62 Ekf::KEnc(Matrix6d cov, Matrix26 H, Matrix2d R){
  // Find Kalman Gain
  Matrix62 K;
  K = cov*H.transpose()*(H*cov*H.transpose()+R).inverse();
  return K;
}

// 4. State update
Vector6d Ekf::stateUpdateEnc(Vector6d state, Matrix62 K, Vector2i z, Vector2d h){
  // Find new state
  Vector6d stateNew;
  stateNew = state + K*(z.cast<double>() - h);
  return stateNew;
}

// 5. Covariance update
Matrix6d Ekf::covUpdateEnc(Matrix6d cov, Matrix62 K, Matrix26 H){
  // Find new covariance
  Matrix6d covNew;
  covNew = cov - K*H*cov;
  return covNew;
}

// 6. Total measurement update
void Ekf::measurementUpdateEncoders(Vector2i z, Vector2i zPre, double dt){
  // z is encL and encR
  // Determine h and H
  Vector2d h;
  h = hEnc(state_, b_, tpmRight_, tpmLeft_, zPre(0), zPre(1), dt);
  Matrix26 H;
  H = HEnc(state_, b_, tpmRight_, tpmLeft_, dt);
  // Find Kalman Gain
  Matrix62 K;
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
double Ekf::hIMU(Vector6d state){
  return state(4) + state(5); // return omega + bias
}

// 2. H(x)
Matrix16 Ekf::HIMU(Vector6d state){
  Matrix16 H;
  H << 0, 0, 0, 0, 1, 1;
  return H;
}

// 3. Kalman Gain, K
Vector6d Ekf::KIMU(Matrix6d cov, Matrix16 H, double R){
  // Find Kalman Gain
  // Must create a 1x1 matrix container for the double RIMU_
  MatrixXd RMatrix(1,1);
  RMatrix << R;
  Vector6d K;
  K = cov*H.transpose()*(H*cov*H.transpose()+RMatrix).inverse();
  return K;
}

// 4. State update
Vector6d Ekf::stateupdateIMU(Vector6d state, Vector6d K, double z, double h){
  // Find new state
  Vector6d stateNew;
  stateNew = state + K*(z - h);
  return stateNew;
}

// 5. Covariance update
Matrix6d Ekf::covUpdateIMU(Matrix6d cov, Vector6d K, Matrix16 H){
  // Find new covariance
  Matrix6d covNew;
  covNew = cov - K*H*cov;
  return covNew;
}

// 3. Total measurement update
void Ekf::measurementUpdateIMU(double z){ // z is omega_z
  // Determine h and H
  double h;
  h = hIMU(state_);
  Matrix16 H;
  H = HIMU(state_);
  // Find Kalman Gain
  Vector6d K;
  K = KIMU(cov_, H, RIMU_);
  // Find new state
  state_ = stateupdateIMU(state_, K, z, h);
  // Find new covariance
  cov_ = covUpdateIMU(cov_, K, H);
}

void Ekf::initState(Vector6d state) {
  state_ = state;
}
void Ekf::initCov(Matrix6d cov) {
  cov_ = cov;
}
void Ekf::initSystem(Matrix6d Q) {
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
