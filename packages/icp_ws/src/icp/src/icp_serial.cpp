#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "ros/ros.h"

// #include "icp/Lidar.h"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

sensor_msgs::PointCloud prev_pc;
sensor_msgs::PointCloud curr_pc;

// std::tuple<Eigen::Vector2d, std::vector<Vector2d>> compute_mean(const std::vector<Eigen::Vector2d>& pc) {
Eigen::Vector2d compute_mean(const Eigen::MatrixXd& pc) {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  for (int i = 0; i < pc.cols(); ++i) {
    center += pc.col(i);
  }
  center /= pc.cols();
  return center;
}

Eigen::Matrix2d compute_cross_covariance(const Eigen::MatrixXd& P, 
                                        const Eigen::MatrixXd& Q, 
                                        const std::vector<int>& correspondences) {
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (const int& i : correspondences) {
    cov += Q.col(correspondences[i]) * P.col(i).transpose();
  }
  return cov;
}


Eigen::MatrixXd icp_serial(int n_itr) {
    // std::cout << "Success! PC Sizes: " << prev_pc.points.size() << " : " << curr_pc.points.size() << std::endl;
    // Check that I have 2 point cloud scans
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd P_mat;  
    if (curr_pc.points.size() < prev_pc.points.size()) {
        P = Eigen::MatrixXd(2, curr_pc.points.size());
        Q = Eigen::MatrixXd(2, curr_pc.points.size());
        P_mat = Eigen::MatrixXd(2, curr_pc.points.size());
    } else {
        P = Eigen::MatrixXd(2, prev_pc.points.size());
        Q = Eigen::MatrixXd(2, prev_pc.points.size());
        P_mat = Eigen::MatrixXd(2, prev_pc.points.size());
    }
    /* BEGIN ICP ALGORITHM */  
    bool didFirstItr = false;
    for (int itr = 0; itr < n_itr; itr++) {
        // std::cout << "Made it inside! 1" << std::endl;
        // Find Correspondences
        int tot_dist = 0;
        std::vector<int> correspondences;
        for (int i = 0; i < P.cols(); i++) {
            Eigen::Vector2d p;
            if (!(didFirstItr)) {
                p << curr_pc.points[i].x, curr_pc.points[i].y;
                P.col(i) = p; 
            } else {
                p = P_mat.col(i);
            }
            double min_dist = 1000.0;
            int corr_idx = -1;
            for (int j = 0; j < Q.cols(); j++) {
                Eigen::Vector2d q;
                q << prev_pc.points[j].x, prev_pc.points[j].y;
                if (i == 0) {
                    Q.col(j) = q; 
                }
                double curr_dist = (p - q).norm();
                if (curr_dist < min_dist) {
                    min_dist = curr_dist;
                    corr_idx = j;
                }
            }
            correspondences.push_back(corr_idx);
        }
        // std::cout << "Made it inside! 2" << std::endl;

        // Compute Mean of Point Clouds & Cross-Covariance
        Eigen::Vector2d mu_Q = compute_mean(Q);
        Eigen::Vector2d mu_P;
        Eigen::Matrix2d cov;
        if (!(didFirstItr)) {
            mu_P = compute_mean(P);
            cov = compute_cross_covariance(P,Q,correspondences);
        } else {
            mu_P = compute_mean(P_mat);
            cov = compute_cross_covariance(P_mat,Q,correspondences);
        }
        
        // std::cout << "Made it inside! 4" << std::endl;

        // SVD Decomposition
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd U = svd.matrixU();
        Eigen::MatrixXd S = svd.singularValues().asDiagonal();
        Eigen::MatrixXd V = svd.matrixV();
        // Eigen::MatrixXd reconstructedMatrix = U * S * V.transpose();
        // std::cout << "Made it inside! 5" << std::endl;

        // Shift P's point cloud
        Eigen::MatrixXd R = U * V.transpose();
        Eigen::VectorXd t = mu_Q - R * mu_P;
        if (!(didFirstItr)) {
            for (size_t col = 0; col < P.cols(); col++) {
                P_mat.col(col) = P.col(col);
            } // P_mat = 2 x N
        }
        // std::cout << "Made it inside! R:" << R.rows() << "x" << R.cols() << " P_mat:" << P_mat.rows() << "x" << P_mat.cols() << " t:" << t.size() << std::endl;
        P_mat = R * P_mat + t.replicate(1,P_mat.cols());//t;
        // std::cout << "Made it inside! 8" << std::endl;
        if (!(didFirstItr)) {
            didFirstItr = true;
        }
    }
    /* END ICP ALGORITHM */
    return P_mat;
}

sensor_msgs::PointCloud get_pc_msg(Eigen::MatrixXd P_mat) {
    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "sonic/lidar_frame";
    for (int i = 0; i < P_mat.cols(); i++) {
        geometry_msgs::Point32 point;
        point.x = P_mat.col(i)[0];
        point.y = P_mat.col(i)[1];
        pc.points.push_back(point);
    }
    return pc;
}

// void lidarMessageCallback(const icp::Lidar::ConstPtr& msg)
void lidarMessageCallback(const sensor_msgs::PointCloud::ConstPtr& pc_msg)
{
    //ROS_INFO("Received custom message: data = %f", msg->distances[0]);
    if (curr_pc.points.empty()) {
        curr_pc = *pc_msg;
    } else {
        prev_pc = curr_pc;
        curr_pc = *pc_msg;
    }
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "icp_serial");

    // Create a ROS NodeHandle
    ros::NodeHandle nh;
    ros::Rate rate(1); // Hz

    // Subscribe to the custom topic with your custom message type
    ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud>("lidar_pointcloud", 10, lidarMessageCallback);

    // Publisher of Alligned Point Cloud
    ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud>("icp_pointcloud", 1);

    // ROS spin to wait for messages
    Eigen::MatrixXd P_mat;
    sensor_msgs::PointCloud icp_msg;
    int icp_itr;
    while (ros::ok()) {
        if (!(prev_pc.points.empty()) && !(curr_pc.points.empty())) {
            if (!nh.getParam("icp_itr", icp_itr))
            {
                ROS_ERROR("Failed to get parameter 'icp_itr'. Using default value of 1.");
                icp_itr = 1;  // Default value
            }        
            P_mat = icp_serial(icp_itr);
            icp_msg = get_pc_msg(P_mat);
            pc_pub.publish(icp_msg);
        }
        ros::spinOnce();
        rate.sleep();
    }
    //   ros::spin();

    return 0;
}

