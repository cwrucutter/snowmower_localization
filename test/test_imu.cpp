#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "test_imu");
  ros::NodeHandle nh;

  ros::NodeHandle private_nh("~");
  // Create variables for yaw rate
  double yawRate;
  if(!private_nh.getParam("rate", yawRate)) {
    yawRate = 0;
    ROS_WARN_STREAM("No rate was recieved. Using 0.0");
  }
  else {
    ROS_WARN_STREAM("Rate is " << yawRate);
  }

  // Create a publisher object
  ros::Publisher imuPub = nh.advertise<sensor_msgs::Imu>("imu/data",1);
  // Create an Imu message object
  sensor_msgs::Imu imuMsg;
  imuMsg.header.frame_id = "base_imu";
  imuMsg.header.stamp = ros::Time::now();
  imuMsg.angular_velocity.z = yawRate;
  imuPub.publish(imuMsg);

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    imuMsg.header.stamp = ros::Time::now();
    imuMsg.angular_velocity.z = yawRate;
    imuPub.publish(imuMsg);
    loop_rate.sleep();
  }
  return 0;
}
