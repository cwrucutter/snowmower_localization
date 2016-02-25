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

  /*************
    Parameters
  *************/
  // System Model Covariance Matrix
  Matrix6d Q_;
 
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
  Vector6d state_;
  Matrix6d cov_;

  /*******************
    Member Functions
  *******************/
  void initState(Vector6d state);
  void initCov(Matrix6d cov);

  // System update
  void initSystem(Matrix6d Q);
  Vector6d fSystem(Vector6d state, double dt);
  Matrix6d FSystem(Vector6d state, double dt);
  void systemUpdate(double dt);

  // Absolute measurement updates (e.g. GPS, beacons)
  // DecaWave
  void initDecaWave(Matrix4d R, Matrix42 DecaWaveBeaconLoc,
		    Vector2d DecaWaveOffset);
  Vector4d hDecaWave(Vector6d state, Matrix42 DecaWaveBeaconLoc,
		     Vector2d DecaWaveOffset);
  Matrix46 HDecaWave(Vector6d state, Matrix42 DecaWaveBecaonLoc,
		     Vector2d DecaWaveOffset);
  MatrixXd KG(const MatrixXd& cov, const MatrixXd& H, const MatrixXd& R);
  /*! -------------------------------------------------------------------------
   * @fn MatrixXd Ekf::selectiveMeasurementKG(const MatrixXd& cov, 
   *                                          const MatrixXd& H,
   *                                          const MatrixXd& R,
   *                                          const VectorXd& exMeas)
   *
   * @brief When a bad measurement is recieved, this function manipulates the
   *        matricies used to calculate the Kalman Gain, K, such that the bad
   *        measurement has no effect on the state update.
   *
   * input parameters:
   * @param cov    - System covariance.
   * @param H      - Jacobian of the measurement model, h(x).
   * @param R      - Noise Matrix for the measurement.
   * @param exMeas - Measurements to exclude from the EKF update.
   *
   * @returns - The modified Kalman gain, which ignores the specified
   *            measurements.
   */
  MatrixXd selectiveMeasurementKG(const MatrixXd& cov, const MatrixXd& H,
				  const MatrixXd& R, const VectorXi& exMeas);
  Vector6d stateUpdateDecaWave(Vector6d state, Matrix64 K, Vector4d z,
			       Vector4d h);
  Matrix6d covUpdateDecaWave(Matrix6d cov, Matrix64 K, Matrix46 H);
  void measurementUpdateDecaWave(Vector4d z); // z is d1-d4
  
  // Relative measurement updates (e.g. encoders, IMU)
  // Wheel Encoders
  void initEnc(Matrix2d R, double b, double tpmRight, double tpmLeft);
  Vector2d hEnc(Vector6d state, double b, double tpmRight,
		double tpmLeft, int ticksPreRight, int ticksPreLeft,
		double dt);
  Matrix26 HEnc(Vector6d state, double b, double tpmright,
		double tpmLeft, double dt);
  Matrix62 KEnc(Matrix6d cov, Matrix26 H, Matrix2d R);
  Vector6d stateUpdateEnc(Vector6d state, Matrix62 K, Vector2i z,
			  Vector2d h);
  Matrix6d covUpdateEnc(Matrix6d cov, Matrix62 K, Matrix26 H);
  void measurementUpdateEncoders(Vector2i z, Vector2i zPre, double dt);
  
  // IMU
  void initIMU(double R);
  double hIMU(Vector6d state);
  Matrix16 HIMU(Vector6d state);
  Vector6d KIMU(Matrix6d cov, Matrix16 H, double R);
  Vector6d stateupdateIMU(Vector6d state, Vector6d K, double z,
			  double h);
  Matrix6d covUpdateIMU(Matrix6d cov, Vector6d K, Matrix16 H);
  void measurementUpdateIMU(double z); // z is omega_z

  // Other functions
  // Function that zeros out the covariance between x, y, and theta, and the
  // yaw rate bias. Explanation found in Eric Perko's thesis.
  Matrix6d zeroOutBiasXYThetaCov(Matrix6d cov);
  /* 
   * Helper functions to remove a row or column from an Eigen Matrix. If
   * rowToRemove or columnToRemove is greater than the number of rows or
   * columns, the last column is removed.
   * Found at http://stackoverflow.com/a/21068014/5525775
   */
  void removeRow(MatrixXd& matrix, unsigned int rowToRemove);
  void removeColumn(MatrixXd& matrix, unsigned int colToRemove);
  void addZeroColumn(MatrixXd& matrix, unsigned int colToAdd);
  /*
   * A helper function to sort vector of unknown length and remove duplicates.
   */
  VectorXi uniqueSort(const VectorXi& vec);

  //public:
  Ekf();
  ~Ekf();
};

#endif
