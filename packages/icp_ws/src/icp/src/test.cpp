// simple_cpp_node.cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

void messageCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Received message: %s", msg->data.c_str());
}

int main(int argc, char** argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "simple_cpp_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  // Subscribe to the "chatter" topic
  ros::Subscriber sub = nh.subscribe("chatter", 10, messageCallback);

  // Spin to process callbacks
  ros::spin();

  return 0;
}
