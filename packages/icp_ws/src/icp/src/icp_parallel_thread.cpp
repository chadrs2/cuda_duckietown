#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <thread>

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
    cov += Q.col(i) * P.col(i).transpose();
  }
  return cov;
}


Eigen::MatrixXd icp_parallel_threads(int n_itr, int num_threads) {
    // std::cout << "Success! PC Sizes: " << prev_pc.points.size() << " : " << curr_pc.points.size() << std::endl;
    // Check that I have 2 point cloud scans
    // std::cout << "Made it inside!" << std::endl;
    /* BEGIN ICP ALGORITHM */
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd P_mat;    
    bool didFirstItr = false;
    for (int itr = 0; itr < n_itr; itr++) {
        if (curr_pc.points.size() < prev_pc.points.size()) {
            P = Eigen::MatrixXd(2, curr_pc.points.size());
            Q = Eigen::MatrixXd(2, curr_pc.points.size());
            P_mat = Eigen::MatrixXd(2, curr_pc.points.size());
        } else {
            P = Eigen::MatrixXd(2, prev_pc.points.size());
            Q = Eigen::MatrixXd(2, prev_pc.points.size());
            P_mat = Eigen::MatrixXd(2, prev_pc.points.size());
        }
        // std::cout << "Made it inside! 1" << std::endl;
        // Find Correspondences
        int tot_dist = 0;
        int correspondences[P.cols()];
        
        // ------------ Parallelize here ------------
        int pc_size = std::min(curr_pc.points.size(), prev_pc.points.size());
        int chunk_size = (pc_size + num_threads - 1) / num_threads; // Calculate chunk size
        // ROS_INFO("P size %d, Q size %d", P.cols(), Q.cols());
        // ROS_INFO("PC_Size: %d", pc_size);
        std::vector<std::thread> threads;
        // ROS_INFO("Running %d parallel threads with %d chunk size", num_threads, chunk_size);        
        for (int t = 0; t < num_threads; t++) {
            int start_idx = t * chunk_size;
            int end_idx = std::min((t + 1) * chunk_size, pc_size);

            // ----------------------------------------------------------------------
            sensor_msgs::PointCloud& curr_pc_ref = curr_pc;
            sensor_msgs::PointCloud& prev_pc_ref = prev_pc;
            threads.emplace_back([start_idx, end_idx, &P, &P_mat, &correspondences, curr_pc_ref, prev_pc_ref, &didFirstItr, &Q]() {
            // threads.emplace_back([start_idx, end_idx, &P, &P_mat, &correspondences, curr_pc, prev_pc, &didFirstItr, &Q]() {
                // ROS_INFO("Running icp_parallel_threads with start index %d and end index %d", start_idx, end_idx);
                for (int i = start_idx; i < end_idx; i++) {
                    // ROS_INFO("Running icp_parallel_threads with i %d", i);
                    Eigen::Vector2d p;
                    if (!(didFirstItr)) {
                        // ROS_INFO("First Iteration on icp_parallel_threads with i %d", i);
                        p << curr_pc.points[i].x, curr_pc.points[i].y;
                        P.col(i) = p; 
                        // ROS_INFO("Finished First Iteration on icp_parallel_threads with i %d", i);
                    } else {
                        p = P_mat.col(i);
                    }
                    double min_dist = 1000.0;
                    int corr_idx = -1;
                    for (int j = 0; j < Q.cols(); j++) {
                        Eigen::Vector2d q;
                        // ROS_INFO("Setting q on icp_parallel_threads with i %d and j %d", i, j);
                        q << prev_pc.points[j].x, prev_pc.points[j].y;
                        if (i == 0) {
                            // ROS_INFO("Setting first Q on icp_parallel_threads with i %d and j %d", i, j);
                            Q.col(j) = q; 
                        }
                        // ROS_INFO("Calculating distance on icp_parallel_threads with i %d and j %d", i, j);
                        double curr_dist = (p - q).norm();
                        if (curr_dist < min_dist) {
                            // ROS_INFO("Setting new min_dist on icp_parallel_threads with i %d and j %d", i, j);
                            min_dist = curr_dist;
                            corr_idx = j;
                        }
                    }
                    // ROS_INFO("Setting correspondences on icp_parallel_threads with i %d", i);
                    correspondences[i] = corr_idx;
                }
            });
            // ----------------------------------------------------------------------    
        }
        // ROS_INFO("For loops completed");
        // for (int i = 0; i < P.cols(); i++) {
        //     Eigen::Vector2d p;
        //     if (!(didFirstItr)) {
        //         p << curr_pc.points[i].x, curr_pc.points[i].y;
        //         P.col(i) = p; 
        //     } else {
        //         p = P_mat.col(i);
        //     }
        //     double min_dist = 1000.0;
        //     int corr_idx = -1;
        //     for (int j = 0; j < Q.cols(); j++) {
        //         Eigen::Vector2d q;
        //         q << prev_pc.points[j].x, prev_pc.points[j].y;
        //         if (i == 0) {
        //             Q.col(j) = q; 
        //         }
        //         double curr_dist = (p - q).norm();
        //         if (curr_dist < min_dist) {
        //             min_dist = curr_dist;
        //             corr_idx = j;
        //         }
        //     }
        //     correspondences[i] = corr_idx;
        // }
        // Find Correspondences using a thread pool

        // Wait for all threads to finish
        for (auto& thread : threads) {
            thread.join();
        }
        // ROS_INFO("Threads joined");

        // Convert correspondences to vector
        std::vector<int> correspondences_vec(correspondences, correspondences + pc_size);
        // std::cout << "Made it inside! 2" << std::endl;

        // Compute Mean of Point Clouds & Cross-Covariance
        Eigen::Vector2d mu_Q = compute_mean(Q);
        Eigen::Vector2d mu_P;
        Eigen::Matrix2d cov;
        if (!(didFirstItr)) {
            mu_P = compute_mean(P);
            cov = compute_cross_covariance(P,Q,correspondences_vec);
        } else {
            mu_P = compute_mean(P_mat);
            cov = compute_cross_covariance(P_mat,Q,correspondences_vec);
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
    int num_threads;
    while (ros::ok()) {
        if (!(prev_pc.points.empty()) && !(curr_pc.points.empty())) {
            if (!nh.getParam("icp_itr", icp_itr))
            {
                ROS_WARN("Failed to get parameter 'icp_itr'. Using default value of 5.");
                icp_itr = 5;  // Default value
            }  
            if (!nh.getParam("icp_num_threads", num_threads))
            {
                ROS_WARN("Failed to get parameter 'icp_num_threads'. Using default value of 4.");
                num_threads = 4;  // Default value
            }  

            P_mat = icp_parallel_threads(icp_itr, num_threads);
            icp_msg = get_pc_msg(P_mat);
            pc_pub.publish(icp_msg);
        }
        ros::spinOnce();
        rate.sleep();
    }
    //   ros::spin();

    return 0;
}

