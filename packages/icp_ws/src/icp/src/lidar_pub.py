#!/usr/bin/env python

import rospy
from icp.msg import Lidar  # Import the custom message type

def lidar_msg_publisher():
    # Initialize the ROS node
    rospy.init_node('lidar_publisher', anonymous=True)

    # Create a publisher for the custom topic with your custom message type
    pub = rospy.Publisher('lidar_scans', Lidar, queue_size=10)

    # Create a custom message
    msg = Lidar()
    msg.distances = [0.5,0.1,0.7]

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Publish the custom message
        pub.publish(msg)
        rospy.loginfo("Publishing custom message: data = %f, count = %d", msg.data, msg.count)
        rate.sleep()

if __name__ == '__main__':
    try:
        lidar_msg_publisher()
    except rospy.ROSInterruptException:
        pass
