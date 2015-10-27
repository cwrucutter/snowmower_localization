#include <ros/ros.h>
#include <iostream>
#include "snowmower_localization/ekf.h"

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "ekf_test");

  typedef Matrix<double, 5, 1> Vector5d;

  // Create an EKF object
  Ekf ekf;
  Vector5d state;
  state << 0, 1, 2, 3, 4;
  // ekf.state_ << 0, 1, 2, 3, 4;
  std::cout << ekf.state_ << std::endl;

  return 0;
}
