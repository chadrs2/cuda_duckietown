#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
import numpy as np
from geometry_msgs.msg import Transform


class LidarPointCloudSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('icp_gauss_newton_py', anonymous=True)

        # Create a subscriber for the PointCloud topic
        self.pointcloud_sub = rospy.Subscriber('lidar_pointcloud', PointCloud, self.pointcloud_callback)
        self.pointcloud_pub = rospy.Publisher('lidar_aligned', PointCloud, queue_size=1)
        self.transform_pub = rospy.Publisher('transform', Transform, queue_size=10)
        self.prev_points = None

    # P is moved data, q is prev data
    def icp_svd(self, p, q):
        p = np.array(p)
        q = np.array(q)

        # Convert p and q to NumPy arrays without the z coordinate
        p = np.array([[point.x, point.y] for point in p], dtype=float).T
        q = np.array([[point.x, point.y] for point in q], dtype=float).T

        # Reshape to ensure that the input arrays have the shape (dimension, num_points)
        if p.ndim == 1:
            p = p.reshape(-1, 1)
        if q.ndim == 1:
            q = q.reshape(-1, 1)

        # Don't love this implementation, but it'll have to do for now
        # chop off the longer array to match the shorter one
        if p.shape[1] > q.shape[1]:
            p = p[:, :q.shape[1]]
        elif q.shape[1] > p.shape[1]:
            q = q[:, :p.shape[1]]

        icp_itr_default = 5
        # Get the parameter 'icp_itr' from the ROS parameter server
        icp_itr = rospy.get_param("icp_itr", icp_itr_default)

        # Check if the parameter retrieval was successful
        if icp_itr is None:
            rospy.logerror("Failed to get parameter 'icp_itr'. Using default value of %d.", icp_itr_default)
            icp_itr = icp_itr_default
        kernel=lambda diff: 1.0
        
        def get_correspondence_indices(P, Q):
            """For each point in P find closest one in Q."""
            p_size = P.shape[1]
            q_size = Q.shape[1]
            correspondences = []
            for i in range(p_size):
                p_point = P[:, i]
                min_dist = 9999
                chosen_idx = -1
                for j in range(q_size):
                    q_point = Q[:, j]
                    dist = np.linalg.norm(q_point - p_point)
                    if dist < min_dist:
                        min_dist = dist
                        chosen_idx = j
                correspondences.append((i, chosen_idx, min_dist))
            return correspondences

        def compute_cross_covariance(P, Q, correspondences, kernel=lambda diff: 1.0):
            cov = np.zeros((2, 2))
            exclude_indices = []
            for i, j, k in correspondences:
                p_point = P[:, [i]]
                q_point = Q[:, [j]]
                weight = kernel(p_point - q_point)
                if weight < 0.01: exclude_indices.append(i)
                cov += weight * q_point.dot(p_point.T)
            return cov, exclude_indices
        
        def center_data(data, exclude_indices=[]):
            reduced_data = np.delete(data, exclude_indices, axis=1)
            center = np.array([reduced_data.mean(axis=1)]).T
            return center

        """Perform ICP using SVD."""
        center_of_Q = center_data(q)
        norm_values = []
        P_values = [p.copy()]
        P_copy = p.copy()
        corresp_values = []
        exclude_indices = []
        R_tot = np.eye(2)
        t_tot = np.zeros((2, 1))
        for i in range(icp_itr):
            center_of_P = center_data(P_copy, exclude_indices=exclude_indices)
            correspondences = get_correspondence_indices(p, q)
            corresp_values.append(correspondences)
            norm_values.append(np.linalg.norm(p - q))
            cov, exclude_indices = compute_cross_covariance(p, q, correspondences, kernel)
            U, S, V_T = np.linalg.svd(cov)
            R = U.dot(V_T)  
            t = center_of_Q - R.dot(center_of_P)
            R_tot = R @ R_tot
            t_tot = R.dot(t_tot + t) # Could be one of these two methods
            # t_tot = R.dot(t_tot) + t
            P_copy = R.dot(P_copy) + t
            P_values.append(P_copy)
        corresp_values.append(corresp_values[-1])
        return R_tot, t_tot, P_copy

    def icp_gauss_newton(self, points, prev_points):
        pass

    def publish_transform(self, rotation_matrix, translation_vector):
        # Create a Transform message
        transform_msg = Transform()
        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        roll = 0
        pitch = 0
        qx = 0
        qy = 0
        qz = np.sin(yaw/2)
        qw = np.cos(yaw/2)
 
        transform_msg.rotation.x = qx
        transform_msg.rotation.y = qy
        transform_msg.rotation.z = qz
        transform_msg.rotation.w = qw

        # Set the translation vector (3x1 part of the transformation)
        transform_msg.translation.x = translation_vector[0][0]
        transform_msg.translation.y = translation_vector[1][0]
        transform_msg.translation.z = 0

        # Publish the Transform message
        self.transform_pub.publish(transform_msg)


    def pointcloud_callback(self, pointcloud_msg):
        # Extract information from the received PointCloud message
        header = pointcloud_msg.header
        points = pointcloud_msg.points
        channels = pointcloud_msg.channels

        # Process and print the received data (example: print the first 10 points)
        # print(f"Received PointCloud with {len(points)} points:")
        # for i in range(min(10, len(points))):
        #     point = points[i]
        #     intensity = channels[0].values[i] if channels else None
        #     print(f"Point {i + 1}: ({point.x}, {point.y}, {point.z}), Intensity: {intensity}")
        
        if self.prev_points is not None:
            start_time = rospy.Time.now()
            R, t, aligned_points = self.icp_svd(points, self.prev_points)
            end_time = rospy.Time.now()
            time_taken = end_time - start_time
            # convert to seconds
            time_taken = time_taken.secs + time_taken.nsecs * 1e-9
            print(f"Rotation:{R}\nTranslation:{t}\nTime taken:{time_taken}")
            # create pointcloud message
            pointcloud_msg = PointCloud()
            pointcloud_msg.header.stamp = rospy.Time.now()
            pointcloud_msg.header.frame_id = "sonic/lidar_frame"
            print("Aligned Points shape: ", aligned_points.shape)
            points = [Point32(x=float(aligned_points[0][i]), y=float(aligned_points[1][i]), z=0.0) for i in range(aligned_points.shape[1])]
            pointcloud_msg.points = points

            # Add intensity values as an additional channel with default value 1
            intensity_channel = ChannelFloat32()
            intensity_channel.name = "intensity"
            intensity_channel.values = [1.0] * len(points)
            pointcloud_msg.channels.append(intensity_channel)

            self.pointcloud_pub.publish(pointcloud_msg)
            self.publish_transform(R, t)
        self.prev_points = points

if __name__ == '__main__':
    try:
        lidar_subscriber = LidarPointCloudSubscriber()
        rospy.spin()  # Keep the program from exiting
    except rospy.ROSInterruptException:
        pass
