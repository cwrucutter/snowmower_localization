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

#include <Eigen/Core>

#ifndef __EKF_H_INCLUDED__
#define __EKF_H_INCLUDED__

using namespace Eigen;

class Ekf {

 public:
  typedef Matrix<double, 5, 1> Vector5d;
  typedef Matrix<double, 5, 5> Matrix5d;
  typedef Matrix<double, 1, 5> Matrix15;
  typedef Matrix<double, 2, 5> Matrix25;
  typedef Matrix<double, 5, 2> Matrix52;
  typedef Matrix<double, 1, 1> Matrix11;
  typedef Matrix<double, 4, 5> Matrix45;
  typedef Matrix<double, 5, 4> Matrix54;
  typedef Matrix<double, 4, 2> Matrix42;

  /*************
    Parameters
  *************/
  // System Model Covariance Matrix
  Matrix5d Q_;
 
  // Decawave Beacon Locations (meters) x values in col 1, y values in col 2
  Matrix42 DecaWaveBeaconLoc_;
  // Decawave offset - Distance from base_link to decawave_link frames in m.
  Vector2d DecaWaveOffset_;
  // Decawave Covariance Matrix
  Matrix4d RDecaWave_;
  // Wheel Track
  double b_;
  // Left and right encoder ticks per meter traveled
  int tpmRight_;
  int tpmLeft_;
  // Encoder Covariance Matrix
  Matrix2d REnc_;
  // IMU Covariance Matrix
  double RIMU_;

  /*******************
    Member Variables
  *******************/
  // State Vector and Covariance Matrix
  Vector5d state_;
  Matrix5d cov_;

  /*******************
    Member Functions
  *******************/
  void initState(Vector5d state);
  void initCov(Matrix5d cov);

  // System update
  void initSystem(Matrix5d Q);
  Vector5d fSystem(Vector5d state, double dt);
  Matrix5d FSystem(Vector5d state, double dt);
  void systemUpdate(double dt);

  // Absolute measurement updates (e.g. GPS, beacons)
  // DecaWave
  void initDecaWave(Matrix4d R, Matrix42 DecaWaveBeaconLoc,
		    Vector2d DecaWaveOffset);
  Vector4d hDecaWave(Vector5d state, Matrix42 DecaWaveBeaconLoc,
		     Vector2d DecaWaveOffset);
  Matrix45 HDecaWave(Vector5d state, Matrix42 DecaWaveBecaonLoc,
		     Vector2d DecaWaveOffset);
  Matrix54 KDecaWave(Matrix5d cov, Matrix45 H, Matrix4d R);
  Vector5d stateUpdateDecaWave(Vector5d state, Matrix54 K, Vector4d z,
			       Vector4d h);
  Matrix5d covUpdateDecaWave(Matrix5d cov, Matrix54 K, Matrix45 H);
  void measurementUpdateDecaWave(Vector4d z); // z is d1-d4
  
  // Relative measurement updates (e.g. encoders, IMU)
  // Wheel Encoders
  void initEnc(Matrix2d R, double b, double tpmRight, double tpmLeft);
  Vector2d hEnc(Vector5d state, double b, double tpmRight,
		double tpmLeft);
  Matrix25 HEnc(Vector5d state, double b, double tpmright,
		double tpmLeft);
  Matrix52 KEnc(Matrix5d cov, Matrix25 H, Matrix2d R);
  Vector5d stateUpdateEnc(Vector5d state, Matrix52 K, Vector2d z,
			  Vector2d h);
  Matrix5d covUpdateEnc(Matrix5d cov, Matrix52 K, Matrix25 H);
  void measurementUpdateEncoders(Vector2d z); // z is encL and encR
  // IMU
  void initIMU(double R);
  double hIMU(Vector5d state);
  Matrix15 HIMU(Vector5d state);
  Vector5d KIMU(Matrix5d cov, Matrix15 H, double R);
  Vector5d stateupdateIMU(Vector5d state, Vector5d K, double z,
			  double h);
  Matrix5d covUpdateIMU(Matrix5d cov, Vector5d K, Matrix15 H);
  void measurementUpdateIMU(double z); // z is omega_z
  
  //public:
  Ekf();
  ~Ekf();
};

#endif
