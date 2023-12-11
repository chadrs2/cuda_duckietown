#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
import numpy as np


class LidarPointCloudSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('icp_gauss_newton_py', anonymous=True)

        # Create a subscriber for the PointCloud topic
        self.pointcloud_sub = rospy.Subscriber('lidar_pointcloud', PointCloud, self.pointcloud_callback)
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
        iterations=10
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
                correspondences.append((i, chosen_idx))
            return correspondences
    
        def compute_cross_covariance(P, Q, correspondences, kernel=lambda diff: 1.0):
            cov = np.zeros((2, 2))
            exclude_indices = []
            for i, j in correspondences:
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
        for i in range(iterations):
            center_of_P = center_data(P_copy, exclude_indices=exclude_indices)
            correspondences = get_correspondence_indices(p, q)
            corresp_values.append(correspondences)
            norm_values.append(np.linalg.norm(p - q))
            cov, exclude_indices = compute_cross_covariance(p, q, correspondences, kernel)
            U, S, V_T = np.linalg.svd(cov)
            R = U.dot(V_T)  
            t = center_of_Q - R.dot(center_of_P)  
            P_copy = R.dot(P_copy) + t
            P_values.append(P_copy)
        corresp_values.append(corresp_values[-1])
        return R, t

    def icp_gauss_newton(self, points, prev_points):
        pass

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
            R, t = self.icp_svd(points, self.prev_points)
            print(f"Rotation:{R}\nTranslation:{t}")
        
        self.prev_points = points

if __name__ == '__main__':
    try:
        lidar_subscriber = LidarPointCloudSubscriber()
        rospy.spin()  # Keep the program from exiting
    except rospy.ROSInterruptException:
        pass