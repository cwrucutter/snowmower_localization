#include <ros/ros.h>
#include "snowmower_localization/ekf.h"

/* Constructor */
Ekf::Ekf(): private_nh_("~") {

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
