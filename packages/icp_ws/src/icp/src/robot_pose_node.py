#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Transform, PoseArray


class RobotPose():
    def __init__(self):
        rospy.init_node('robot_pose_node')
        self.pose_pub = rospy.Publisher('robot_pose_array', PoseArray, queue_size=3)
        pose_sub = rospy.Subscriber('transform', Transform, self.update_pose_cb)

        # Initialize Robot Pose to Origin
        self.current_robot_pose = PoseStamped()
        self.current_robot_pose.header.stamp = rospy.Time.now()
        self.current_robot_pose.header.frame_id = 'map'
        self.current_robot_pose.pose.position.x = 0.0
        self.current_robot_pose.pose.position.y = 0.0
        self.current_robot_pose.pose.position.z = 0.0
        self.current_robot_pose.pose.orientation.x = 0.0
        self.current_robot_pose.pose.orientation.y = 0.0
        self.current_robot_pose.pose.orientation.z = 0.0
        self.current_robot_pose.pose.orientation.w = 1.0

        # Array of most recent poses
        self.recent_poses = [self.get_curr_pose()]
        return
    
    def publish_pose_array(self):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = rospy.Time.now()
        pose_array_msg.header.frame_id = "map"  # Replace with your frame ID

        for pose in self.recent_poses:
            pose_array_msg.poses.append(pose.pose)  # Assuming each item in recent_poses is a PoseStamped

        self.pose_pub.publish(pose_array_msg)
        return
    
    def get_curr_pose(self):
        curr_pose = PoseStamped()
        curr_pose.header.stamp = self.current_robot_pose.header.stamp
        curr_pose.header.frame_id = self.current_robot_pose.header.frame_id
        curr_pose.pose.position.x = self.current_robot_pose.pose.position.x
        curr_pose.pose.position.y = self.current_robot_pose.pose.position.y
        curr_pose.pose.position.z = self.current_robot_pose.pose.position.z
        curr_pose.pose.orientation.x = self.current_robot_pose.pose.orientation.x
        curr_pose.pose.orientation.y = self.current_robot_pose.pose.orientation.y
        curr_pose.pose.orientation.z = self.current_robot_pose.pose.orientation.z
        curr_pose.pose.orientation.w = self.current_robot_pose.pose.orientation.w
        return curr_pose
    
    def update_pose_cb(self,msg):
        self.current_robot_pose.header.stamp = rospy.Time.now()

        # Update position
        self.current_robot_pose.pose.position.x += msg.translation.x
        self.current_robot_pose.pose.position.y += msg.translation.y
        self.current_robot_pose.pose.position.z += msg.translation.z
        # Update orientation
        # Assuming both orientations are in quaternion format
        # Quaternion multiplication is needed here to update the orientation
        current_orientation = [self.current_robot_pose.pose.orientation.x,
                            self.current_robot_pose.pose.orientation.y,
                            self.current_robot_pose.pose.orientation.z,
                            self.current_robot_pose.pose.orientation.w]

        transform_orientation = [msg.rotation.x,
                                msg.rotation.y,
                                msg.rotation.z,
                                msg.rotation.w]

        updated_orientation = self.quaternion_multiply(current_orientation, transform_orientation)

        self.current_robot_pose.pose.orientation.x = updated_orientation[0]
        self.current_robot_pose.pose.orientation.y = updated_orientation[1]
        self.current_robot_pose.pose.orientation.z = updated_orientation[2]
        self.current_robot_pose.pose.orientation.w = updated_orientation[3]

        # Append new pose
        self.recent_poses.append(self.get_curr_pose())
        if len(self.recent_poses) > 10:
            self.recent_poses.pop(0)

        # self.pose_pub.publish(self.current_robot_pose)
        self.publish_pose_array()
        return
    
    def quaternion_multiply(self, quaternion1, quaternion2):
        w1, x1, y1, z1 = quaternion1
        w2, x2, y2, z2 = quaternion2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return [x, y, z, w]


if __name__ == '__main__':
    try:
        robot_pose = RobotPose()
        rospy.spin()  # Keep the program from exiting
    except rospy.ROSInterruptException:
        pass