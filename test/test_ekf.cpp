/******************************************************************************
test_ekf.cpp
A set of unit tests for ekf.cpp
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

#include <climits>
#include <cfloat>
#include <cmath>
#include <gtest/gtest.h>
#include <eigen-checks/gtest.h>
#include "snowmower_localization/ekf.h"

/******************************************************************************
 * Testing:                                                                  *
 * Vector6d Ekf::fSystem(Vector6d state, double dt)                          *
 *****************************************************************************
 * zero velocity and zero angular velocity, i.e. standing still.             *
 *****************************************************************************/
TEST(fSystemTest, ZeroVelZeroOmega) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);
  Eigen::MatrixXd stateNew(6,1);
  double dt;

  // Starts at (0, 0) facing in the x directions (heading is 0)
  // with a velocity of 0 m/s for 0.1 s.
  state << 0, 0, 0, 0, 0, 0;
  dt = 0.1;
  // Should end at (1,0) with a speed of 10 m/s
  stateNew << 0, 0, 0, 0, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  // Starts at (0, 0) facing in the x directions (heading is 0)
  // with a velocity of 0 m/s for 0 s.
  state << 0, 0, 0, 0, 0, 0;
  dt = 0;
  // Should end at (1,0) with a speed of 10 m/s
  stateNew << 0, 0, 0, 0, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));
}

/******************************************************************************
 * Testing:                                                                  *
 * Vector6d Ekf::fSystem(Vector6d state, double dt)                          *
 *****************************************************************************
 * Positive velocity and zero angular velocity, i.e. a straight line.        *
 *****************************************************************************/
TEST(fSystemTest, PosVelZeroOmega) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);
  Eigen::MatrixXd stateNew(6,1);
  double dt;

  // Starts at (0, 0) facing in the x directions (heading is 0)
  // with a velocity of 10 m/s for 0.1 s.
  state << 0, 0, 0, 10, 0, 0;
  dt = 0.1;
  // Should end at (1,0) with a speed of 10 m/s
  stateNew << 1, 0, 0, 10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  // Starts at (0, 0) facing in the x directions (heading is 0)
  // with a velocity of 10 m/s for 10 s.
  dt = 10;
  // Should end at (100,0) with a speed of 10 m/s
  stateNew << 100, 0, 0, 10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  // Starts at (0, 0) facing in the -x directions (heading is pi)
  // with a velocity of 10 m/s for 0.1 s.
  state << 0, 0, M_PI, 10, 0, 0;
  dt = 0.1;
  // Should end at (-1,0) with a speed of 10 m/s
  // Heading should change to -pi due to forced angle wrap between [-pi,pi)
  stateNew << -1, 0, -M_PI, 10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  // Starts at (0, 0) facing 45 degrees off of horizontal (heading is pi/4)
  // with a velocity of 10 m/s for 0.1 s.
  state << 0, 0, M_PI_4, 10, 0, 0;
  dt = 0.1;
  // Should end at (sqrt(2)/2,sqrt(2)/2) with a speed of 10 m/s
  // and a heading of pi/4
  stateNew << sqrt(2)/2, sqrt(2)/2, M_PI_4, 10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  // Starts at (0, 0) facing 135 degrees off of horizontal (heading is 3*pi/4)
  // with a velocity of 10 m/s for 0.1 s.
  state << 0, 0, 3*M_PI_4, 10, 0, 0;
  dt = 0.1;
  // Should end at (-sqrt(2)/2,sqrt(2)/2) with a speed of 10 m/s
  // and a heading of 3*pi/4
  stateNew << -sqrt(2)/2, sqrt(2)/2, 3*M_PI_4, 10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  // Starts at (0, 0) facing 215 degrees off of horizontal (heading is 5*pi/4)
  // with a velocity of 10 m/s for 0.1 s.
  state << 0, 0, 5*M_PI_4, 10, 0, 0;
  dt = 0.1;
  // Should end at (-sqrt(2)/2,-sqrt(2)/2) with a speed of 10 m/s
  // and a heading of -3*pi/4 due to forced angle wrap between [-pi,pi)
  stateNew << -sqrt(2)/2, -sqrt(2)/2, -3*M_PI_4, 10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  // Starts at (0, 0) facing 305 degrees off of horizontal (heading is 7*pi/4)
  // with a velocity of 10 m/s for 0.1 s.
  state << 0, 0, 7*M_PI_4, 10, 0, 0;
  dt = 0.1;
  // Should end at (sqrt(2)/2,-sqrt(2)/2) with a speed of 10 m/s
  // and a heading of -pi/4 due to forced angle wrap between [-pi,pi)
  stateNew << sqrt(2)/2, -sqrt(2)/2, -M_PI_4, 10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));
}

/*****************************************************************************
 * Testing:                                                                  *
 * Vector6d Ekf::fSystem(Vector6d state, double dt)                          *
 *****************************************************************************
 * Negative velocity and zero angular velocity, i.e. a straight line.        *
 *****************************************************************************/
TEST(fSystemTest, NegVelZeroOmega) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);
  Eigen::MatrixXd stateNew(6,1);
  double dt;

  // Starts at (0, 0) facing in the x directions (heading is 0)
  // with a velocity of -10 m/s for 0.1 s.
  state << 0, 0, 0, -10, 0, 0;
  dt = 0.1;
  // Should end at (-1,0) with a speed of 10 m/s
  stateNew << -1, 0, 0, -10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  // Starts at (0, 0) facing in the x directions (heading is 0)
  // with a velocity of -10 m/s for 10 s.
  dt = 10;
  // Should end at (-100,0) with a speed of -10 m/s
  stateNew << -100, 0, 0, -10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  state << 0, 0, M_PI, -10, 0, 0;
  dt = 0.1;
  stateNew << 1, 0, -M_PI, -10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, M_PI_4, -10, 0, 0;
  dt = 0.1;
  stateNew << -sqrt(2)/2, -sqrt(2)/2, M_PI_4, -10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, 3*M_PI_4, -10, 0, 0;
  dt = 0.1;
  stateNew << sqrt(2)/2, -sqrt(2)/2, 3*M_PI_4, -10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, 5*M_PI_4, -10, 0, 0;
  dt = 0.1;
  stateNew << sqrt(2)/2, sqrt(2)/2, -3*M_PI_4, -10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, 7*M_PI_4, -10, 0, 0;
  dt = 0.1;
  stateNew << -sqrt(2)/2, sqrt(2)/2, -M_PI_4, -10, 0, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));
}

/*****************************************************************************
 * Testing:                                                                  *
 * Vector6d Ekf::fSystem(Vector6d state, double dt)                          *
 *****************************************************************************
 * Zero velocity and nonzero angular velocity, i.e. Spinning in place.       *
 *****************************************************************************/
TEST(fSystemTest, ZeroVelNonZeroOmega) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);
  Eigen::MatrixXd stateNew(6,1);
  double dt;

  // Starts at (0, 0) facing in the x directions (heading is 0)
  // with an angular velocity of pi rad/s for 1 s.
  state << 0, 0, 0, 0, M_PI,  0;
  dt = 1;
  // Should end at (0,0) facing in the -x direction (heading is -pi)
  // with an angular velocity of pi rad/s.
  stateNew << 0, 0, -M_PI, 0, M_PI, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  // Starts at (0, 0) facing in the x directions (heading is 0)
  // with an angular velocity of pi rad/s for 2 s.
  state << 0, 0, 0, 0, M_PI, 0;
  dt = 2;
  // Should end at (0,0) facing in the x direction (heading is 0)
  // with an angular velocity of pi rad/s.
  stateNew << 0, 0, 0, 0, M_PI, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));
}

/*****************************************************************************
 * Testing:                                                                  *
 * Vector6d Ekf::fSystem(Vector6d state, double dt)                          *
 *****************************************************************************
 * Constant velocity and angular velocity, i.e. Moving along a circular path.*
 *****************************************************************************/
TEST(fSystemTest, drawCircles) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);
  Eigen::MatrixXd stateNew(6,1);

  // Set v and omega as constants
  double omega = 1;
  double v = 2.5;
  // The radius of curvature is defined by:
  double r = v/omega;
  // And the circumference is defined by:
  double c = 2*M_PI*r;
  // The time needed to make a complete circle is then:
  double dt = c/v;

  // Start off at (0,0) and make a complete CCW circle.
  state << 0, 0, 0, v, omega, 0;
  stateNew << 0, 0, 0, v, omega, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  // Start off at (0,0) and make a complete CW circle.
  state << 0, 0, 0, v, -omega, 0;
  stateNew << 0, 0, 0, v, -omega, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  // The time needed to make a half circle.
  dt = c/v/2;

  // Start off at (0,0)
  state << 0, 0, 0, v, omega, 0;
  // and make half a CCW circle
  stateNew << 0, 2*r, -M_PI, v, omega, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  // Start off at (0,0)
  state << 0, 0, 0, v, -omega, 0;
  // and make half a CW circle
  stateNew << 0, -2*r, -M_PI, v, -omega, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));
}

/*****************************************************************************
 * Testing:                                                                  *
 * Vector4d  Ekf::hDecaWave(Vector6d state, Matrix42 DecaWaveBeaconLoc,      *
 *                          Vector2d DecaWaveOffset)                         *
 *****************************************************************************
 * Test on a grid of 3-4-5 triangles.                                        *
 *****************************************************************************/
TEST(hDecaWaveTest, TagInMiddle) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);

  MatrixXd DecaWaveBeaconLoc(4,2);
  Vector2d DecaWaveOffset;

  // Set up a grid of 4 3-4-5 triangles.
  // When beacon is in the middle, all distances will be 5. 
  DecaWaveBeaconLoc << 0.0, 0.0,
                       6.0, 0.0,
                       6.0, 8.0,
                       0.0, 8.0;
  Vector4d h;
  h << 5.0, 5.0, 5.0, 5.0;

  // Start with zero offset and robot in middle
  DecaWaveOffset << 0.0, 0.0;
  state << 3.0, 4.0, 0.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hDecaWave(state, DecaWaveBeaconLoc, DecaWaveOffset)));

  // Offset the decawave and offset the robot
  DecaWaveOffset << -1.0, -2.0;
  state << 4.0, 6.0, 0.0, 0.0, 0.0, 0.0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hDecaWave(state, DecaWaveBeaconLoc, DecaWaveOffset)));

  // Now also offset the angle
  DecaWaveOffset << -1.0, -2.0;
  state << 1.0, 5.0, M_PI_2, 0.0, 0.0, 0.0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hDecaWave(state, DecaWaveBeaconLoc, DecaWaveOffset)));
}

TEST(hDecaWaveTest, TagOnCorner) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);

  MatrixXd DecaWaveBeaconLoc(4,2);
  Vector2d DecaWaveOffset;

  // Set up a grid of 4 3-4-5 triangles.
  // When beacon is in the middle, all distances will be 5. 
  DecaWaveBeaconLoc << 0.0, 0.0,
                       6.0, 0.0,
                       6.0, 8.0,
                       0.0, 8.0;
  Vector4d h;

  // Test tag on beacon 0
  DecaWaveOffset << 0.0, 0.0;
  state << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // Distances should be as such
  h << 0.0, 6.0, 10.0, 8.0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hDecaWave(state, DecaWaveBeaconLoc, DecaWaveOffset)));

  // Test tag on beacon 1
  DecaWaveOffset << 0.0, 0.0;
  state << 6.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // Distances should be as such
  h << 6.0, 0.0, 8.0, 10.0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hDecaWave(state, DecaWaveBeaconLoc, DecaWaveOffset)));

  // Test tag on beacon 2
  DecaWaveOffset << 0.0, 0.0;
  state << 6.0, 8.0, 0.0, 0.0, 0.0, 0.0;
  // Distances should be as such
  h << 10.0, 8.0, 0.0, 6.0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hDecaWave(state, DecaWaveBeaconLoc, DecaWaveOffset)));

  // Test tag on beacon 3
  DecaWaveOffset << 0.0, 0.0;
  state << 0.0, 8.0, 0.0, 0.0, 0.0, 0.0;
  // Distances should be as such
  h << 8.0, 10.0, 6.0, 0.0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hDecaWave(state, DecaWaveBeaconLoc, DecaWaveOffset)));
}

TEST(hEncTest, straightLine) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);
  double b;
  double tpmRight, tpmLeft;
  int ticksPreRight, ticksPreLeft;
  double dt;
  Eigen::MatrixXd h(2,1);

  // Stand still at 0 ticks each.
  state << 0.0, 0.0, 0.0, // x, y, theta
           0.0, 0.0, 0.0; // v, omega, bias
  b = 1.0;
  tpmRight = 10.0;
  tpmLeft = 10.0;
  ticksPreRight = 0;
  ticksPreLeft = 0;
  dt = 1.0;
  h << 0.0, 0.0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hEnc(state, b, tpmRight, tpmLeft,
						   ticksPreRight, ticksPreLeft,
						   dt)));

  // Move forward 10 ticks from -10, 10.
  state << 0.0, 0.0, 0.0, // x, y, theta
           1.0, 0.0, 0.0; // v, omega, bias
  b = 1.0;
  tpmRight = 10.0;
  tpmLeft = 10.0;
  ticksPreRight = -10.0;
  ticksPreLeft = 10.0;
  dt = 1.0;
  h << 0.0, 20.0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hEnc(state, b, tpmRight, tpmLeft,
						   ticksPreRight, ticksPreLeft,
						   dt)));

  // Move forward 10 ticks from -10, 10.
  state << 0.0, 0.0, 0.0, // x, y, theta
           1.0, 0.0, 0.0; // v, omega, bias
  b = 1.0;
  tpmRight = 10.0;
  tpmLeft = 10.0;
  ticksPreRight = 1.0;
  ticksPreLeft = 0.0;
  dt = 1.0;
  h << 11.0, 10.0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hEnc(state, b, tpmRight, tpmLeft,
						   ticksPreRight, ticksPreLeft,
						   dt)));

}

TEST(hEncTest, spinInPlace) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);
  double b;
  double tpmRight, tpmLeft;
  int ticksPreRight, ticksPreLeft;
  double dt;
  Eigen::MatrixXd h(2,1);
  double omega, v, vRight, vLeft, r;

  // Spin in place for a second.
  omega = 1;
  v = 0;
  b = 1.0;
  if (omega != 0) {
    r = v/omega;
    vRight = (r + b/2) * omega;
    vLeft =  (r - b/2) * omega;
  }
  else {
    vRight = v;
    vLeft = v;
  }
  dt = 1.0;

  state << 0.0, 0.0, 0.0, // x, y, theta
           v, omega, 0.0; // v, omega, bias
  tpmRight = 10.0;
  tpmLeft = 10.0;
  ticksPreRight = 0.0;
  ticksPreLeft = 0.0;

  h << ticksPreRight + vRight*dt*tpmRight,
    ticksPreLeft + vLeft*dt*tpmLeft;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hEnc(state, b, tpmRight, tpmLeft,
						   ticksPreRight, ticksPreLeft,
						   dt)));

  // Spin the other way for a second.
  omega = 3;
  v = 0;
  b = 0.6;
  if (omega != 0) {
    r = v/omega;
    vRight = (r + b/2) * omega;
    vLeft =  (r - b/2) * omega;
  }
  else {
    vRight = v;
    vLeft = v;
  }
  dt = 1.0;

  state << 1.0, 0.2, M_PI, // x, y, theta
           v, omega, 0.3; // v, omega, bias
  tpmRight = 25000.0;
  tpmLeft =  24000.0;
  ticksPreRight = 290234;
  ticksPreLeft = -289082;

  h << ticksPreRight + vRight*dt*tpmRight,
    ticksPreLeft + vLeft*dt*tpmLeft;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hEnc(state, b, tpmRight, tpmLeft,
						   ticksPreRight, ticksPreLeft,
						   dt)));

}

TEST(hEncTest, moveAndSpin) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);
  double b;
  double tpmRight, tpmLeft;
  int ticksPreRight, ticksPreLeft;
  double dt;
  Eigen::MatrixXd h(2,1);
  double omega, v, vRight, vLeft, r;

  // Forward and CCW turn.
  omega = 1;
  v = 1;
  b = 1.0;
  if (omega != 0) {
    r = v/omega;
    vRight = (r + b/2) * omega;
    vLeft =  (r - b/2) * omega;
  }
  else {
    vRight = v;
    vLeft = v;
  }
  dt = 1.0;

  state << 0.0, 0.0, 0.0, // x, y, theta
           v, omega, 0.0; // v, omega, bias
  tpmRight = 10.0;
  tpmLeft = 10.0;
  ticksPreRight = 0.0;
  ticksPreLeft = 0.0;

  h << ticksPreRight + vRight*dt*tpmRight,
    ticksPreLeft + vLeft*dt*tpmLeft;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hEnc(state, b, tpmRight, tpmLeft,
						   ticksPreRight, ticksPreLeft,
						   dt)));

  // Forward and CW TURN.
  omega = -3;
  v = 4;
  b = 0.6;
  if (omega != 0) {
    r = v/omega;
    vRight = (r + b/2) * omega;
    vLeft =  (r - b/2) * omega;
  }
  else {
    vRight = v;
    vLeft = v;
  }
  dt = 1.0;

  state << 1.0, 0.2, M_PI, // x, y, theta
           v, omega, 0.3; // v, omega, bias
  tpmRight = 25000.0;
  tpmLeft =  24000.0;
  ticksPreRight = 290234;
  ticksPreLeft = -289082;

  h << ticksPreRight + vRight*dt*tpmRight,
    ticksPreLeft + vLeft*dt*tpmLeft;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hEnc(state, b, tpmRight, tpmLeft,
						   ticksPreRight, ticksPreLeft,
						   dt)));

  // Reverse and CCW turn.
  omega = 1;
  v = -1;
  b = 1.0;
  if (omega != 0) {
    r = v/omega;
    vRight = (r + b/2) * omega;
    vLeft =  (r - b/2) * omega;
  }
  else {
    vRight = v;
    vLeft = v;
  }
  dt = 1.0;

  state << 0.0, 0.0, 0.0, // x, y, theta
           v, omega, 0.0; // v, omega, bias
  tpmRight = 10.0;
  tpmLeft = 10.0;
  ticksPreRight = 0.0;
  ticksPreLeft = 0.0;

  h << ticksPreRight + vRight*dt*tpmRight,
    ticksPreLeft + vLeft*dt*tpmLeft;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hEnc(state, b, tpmRight, tpmLeft,
						   ticksPreRight, ticksPreLeft,
						   dt)));

  // Reverse and CW TURN.
  omega = -3;
  v = -4;
  b = 0.6;
  if (omega != 0) {
    r = v/omega;
    vRight = (r + b/2) * omega;
    vLeft =  (r - b/2) * omega;
  }
  else {
    vRight = v;
    vLeft = v;
  }
  dt = 1.0;

  state << 1.0, 0.2, M_PI, // x, y, theta
           v, omega, 0.3; // v, omega, bias
  tpmRight = 25000.0;
  tpmLeft =  24000.0;
  ticksPreRight = 290234;
  ticksPreLeft = -289082;

  h << ticksPreRight + vRight*dt*tpmRight,
    ticksPreLeft + vLeft*dt*tpmLeft;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(h,ekf.hEnc(state, b, tpmRight, tpmLeft,
						   ticksPreRight, ticksPreLeft,
						   dt)));

}

TEST(FSystemTest, zeroOmega) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);
  double x, y, theta, v, omega, bias;
  double dt;
  typedef Matrix<double, 6, 6, RowMajor> Matrix6d; 
  Matrix6d F;

  x = 0;
  y = 0;
  v = 2;
  theta = M_PI/3;
  omega = 0;
  dt = 0.01;

  state << x, y, theta, v, omega, bias;
  F <<
    1, 0, -0.017320508075689, 0.005, -.00008660254037844386, 0,
    0, 1, 0.01, 0.008660254037844, 0.00005000000000000002, 0,
    0, 0, 1, 0, dt, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0 ,1;

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(F,ekf.FSystem(state, dt), 0.0001));
}

TEST(FSystemTest, smallOmega) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);
  double x, y, theta, v, omega, bias;
  double dt;
  typedef Matrix<double, 6, 6, RowMajor> Matrix6d; 
  Matrix6d F;

  x = 0;
  y = 0;
  v = 2;
  theta = M_PI/3;
  omega = DBL_EPSILON;
  dt = 0.01;

  state << x, y, theta, v, omega, bias;
  F <<
    1, 0, -0.017320508075689, 0.005, -0.00008660254037844386, 0,
    0, 1, 0.01, 0.008660254037844, 0.00005000000000000002, 0,
    0, 0, 1, 0, dt, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0 ,1;

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(F,ekf.FSystem(state, dt), 0.0001));
}

TEST(FSystemTest, maxOmega) {
  Ekf ekf;
  Eigen::MatrixXd state(6,1);
  double x, y, theta, v, omega, bias;
  double dt;
  typedef Matrix<double, 6, 6, RowMajor> Matrix6d; 
  Matrix6d F;

  x = 0;
  y = 0;
  v = 2;
  theta = M_PI/3;
  omega = 2.5;
  dt = 0.01;

  state << x, y, theta, v, omega, bias;
  F <<
    1, 0, -0.017443697402198, 0.004891231645538, -0.00008742229045251353, 0,
    0, 1, 0.009782463291076, 0.008721848701099, 0.00004854890230725648, 0,
    0, 0, 1, 0, dt, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0 ,1;

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(F,ekf.FSystem(state, dt), 0.0001));
}

GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
