#include "ros/ros.h"

#include "icp/Lidar.h"


void lidarMessageCallback(const icp::Lidar::ConstPtr& msg)
{
  ROS_INFO("Received custom message: data = %f", msg->distances[0]);
}

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "icp_serial");

  // Create a ROS NodeHandle
  ros::NodeHandle nh;

  // Subscribe to the custom topic with your custom message type
  ros::Subscriber sub = nh.subscribe("lidar_scans", 10, lidarMessageCallback);

  // ROS spin to wait for messages
  ros::spin();

  return 0;
}

