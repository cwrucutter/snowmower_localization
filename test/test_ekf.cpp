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

#include <limits.h>
#include <cmath>
#include <gtest/gtest.h>
#include <eigen-checks/gtest.h>
#include "snowmower_localization/ekf.h"

TEST(fSystemTest, PosVelZeroOmega) {
  Ekf ekf;
  Eigen::MatrixXd state(5,1);
  Eigen::MatrixXd stateNew(5,1);
  double dt;

  state << 0, 0, 0, 10, 0;
  dt = 0.1;
  stateNew << 1, 0, 0, 10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  dt = 10;
  stateNew << 100, 0, 0, 10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  state << 0, 0, M_PI, 10, 0;
  dt = 0.1;
  stateNew << -1, 0, M_PI, 10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, M_PI_4, 10, 0;
  dt = 0.1;
  stateNew << sqrt(2)/2, sqrt(2)/2, M_PI_4, 10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, 3*M_PI_4, 10, 0;
  dt = 0.1;
  stateNew << -sqrt(2)/2, sqrt(2)/2, 3*M_PI_4, 10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, 5*M_PI_4, 10, 0;
  dt = 0.1;
  stateNew << -sqrt(2)/2, -sqrt(2)/2, 5*M_PI_4, 10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, 7*M_PI_4, 10, 0;
  dt = 0.1;
  stateNew << sqrt(2)/2, -sqrt(2)/2, 7*M_PI_4, 10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));
}

TEST(fSystemTest, NegVelZeroOmega) {
  Ekf ekf;
  Eigen::MatrixXd state(5,1);
  Eigen::MatrixXd stateNew(5,1);
  double dt;

  state << 0, 0, 0, -10, 0;
  dt = 0.1;
  stateNew << -1, 0, 0, -10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  dt = 10;
  stateNew << -100, 0, 0, -10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));

  state << 0, 0, M_PI, -10, 0;
  dt = 0.1;
  stateNew << 1, 0, M_PI, -10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, M_PI_4, -10, 0;
  dt = 0.1;
  stateNew << -sqrt(2)/2, -sqrt(2)/2, M_PI_4, -10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, 3*M_PI_4, -10, 0;
  dt = 0.1;
  stateNew << sqrt(2)/2, -sqrt(2)/2, 3*M_PI_4, -10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, 5*M_PI_4, -10, 0;
  dt = 0.1;
  stateNew << sqrt(2)/2, sqrt(2)/2, 5*M_PI_4, -10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));

  state << 0, 0, 7*M_PI_4, -10, 0;
  dt = 0.1;
  stateNew << -sqrt(2)/2, sqrt(2)/2, 7*M_PI_4, -10, 0;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew, ekf.fSystem(state,dt)));
}

TEST(fSystemTest, ZeroVelNonZeroOmega) {
  Ekf ekf;
  Eigen::MatrixXd state(5,1);
  Eigen::MatrixXd stateNew(5,1);
  double dt;

  state << 0, 0, 0, 0, M_PI;
  dt = 1;
  stateNew << 0, 0, M_PI, 0, M_PI;
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(stateNew,ekf.fSystem(state,dt)));
}

GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
