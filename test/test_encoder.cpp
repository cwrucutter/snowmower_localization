#include <ros/ros.h>
#include <snowmower_msgs/EncMsg.h>

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "test_encoder");
  ros::NodeHandle nh;

  // Create a publisher object
  ros::Publisher encPub = nh.advertise<snowmower_msgs::EncMsg>("enc",1);
  // Create an encoder message object
  snowmower_msgs::EncMsg encMsg;
  encMsg.header.frame_id = "base_link";
  // Create variables for left and right encoder ticks
  int left =0;
  int right = 0;

  ros::Rate loop_rate(60);
  while (ros::ok())
  {
	left += 10;
	right += 10;

	encMsg.left = left;
	encMsg.right = right;
	encMsg.header.stamp = ros::Time::now();
	encPub.publish(encMsg);
	loop_rate.sleep();
  }
  return 0;
}
