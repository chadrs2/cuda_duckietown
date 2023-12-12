#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include <cuda.h>
#include <cuda_runtime.h>

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
                                        const int correspondences[]) {
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (int i = 0; i < P.cols(); ++i) {
    cov += Q.col(correspondences[i]) * P.col(i).transpose();
  }
  return cov;
}

__global__ void find_correspondences(
  Eigen::MatrixXd* d_P,
  Eigen::MatrixXd* d_Q,
  bool didFirstItr,
  Eigen::MatrixXd* d_P_mat,
  int* d_correspondences,
  geometry_msgs::Point32* d_prev_pc,
  geometry_msgs::Point32* d_curr_pc) {
  
  int i = blockIdx.x * blockDim.x + threadIdx.x;

  if (i < d_P->cols()) {
    Eigen::Vector2d p;
    if (!didFirstItr) {
      p << d_curr_pc[i].x, d_curr_pc[i].y;
      d_P->col(i) = p;
    } else {
      p = d_P_mat->col(i);
    }

    double min_dist = 1000.0;
    int corr_idx = -1;

    // Parallel loop over Q
    for (int j = threadIdx.y * blockDim.y; j < d_Q->cols(); j += blockDim.y) {
      Eigen::Vector2d q;
      q << d_prev_pc[j].x, d_prev_pc[j].y;
      double curr_dist = (p - q).norm();
      if (i == 0) {
        d_Q->col(j) = q; 
      }
      if (curr_dist < min_dist) {
        min_dist = curr_dist;
        corr_idx = j;
      }
    }

    d_correspondences[i] = corr_idx;
  }
}

Eigen::MatrixXd icp_parallel(int n_itr) {
    // std::cout << "Success! PC Sizes: " << prev_pc.points.size() << " : " << curr_pc.points.size() << std::endl;
    // Check that I have 2 point cloud scans
    // std::cout << "Made it inside!" << std::endl;
    /* BEGIN ICP ALGORITHM */
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
    int correspondences[P.cols()];
    // Convert point cloud data to arrays of Point32
    geometry_msgs::Point32* prev_pc_array = new geometry_msgs::Point32[prev_pc.points.size()];
    geometry_msgs::Point32* curr_pc_array = new geometry_msgs::Point32[curr_pc.points.size()];
    for (int i = 0; i < prev_pc.points.size(); i++) {
        prev_pc_array[i] = prev_pc.points[i];
    }
    for (int i = 0; i < curr_pc.points.size(); i++) {
        curr_pc_array[i] = curr_pc.points[i];
    }

    // CUDA Defintions
    // Set grid and block dimensions
    dim3 gridDim(P.cols() / blockDim.x + (P.cols() % blockDim.x != 0), Q.cols() / blockDim.y + (Q.cols() % blockDim.y != 0));
    dim3 blockDim(16, 1);

    // Allocate device memory for P, Q, P_mat, and correspondences
    Eigen::MatrixXd* d_P;
    Eigen::MatrixXd* d_Q;
    Eigen::MatrixXd* d_P_mat;
    int* d_correspondences;
    geometry_msgs::Point32* d_prev_pc;
    geometry_msgs::Point32* d_curr_pc;
    cudaMalloc(&d_prev_pc, sizeof(geometry_msgs::Point32) * P.cols());
    cudaMalloc(&d_curr_pc, sizeof(geometry_msgs::Point32) * P.cols());
    cudaMalloc(&d_correspondences, sizeof(int) * P.cols());
    cudaMalloc(&d_P, sizeof(Eigen::MatrixXd) * P.rows() * P.cols());
    cudaMalloc(&d_Q, sizeof(Eigen::MatrixXd) * Q.rows() * Q.cols());
    cudaMalloc(&d_P_mat, sizeof(Eigen::MatrixXd) * P_mat.rows() * P_mat.cols());
    
    // Copy P, Q, and P_mat to device memory
    cudaMemcpy(d_prev_pc, prev_pc_array, sizeof(geometry_msgs::Point32) * P.cols(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_curr_pc, curr_pc_array, sizeof(geometry_msgs::Point32) * P.cols(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_correspondences, correspondences, sizeof(int) * P.cols(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_P, P.data(), sizeof(Eigen::MatrixXd) * P.rows() * P.cols(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_Q, Q.data(), sizeof(Eigen::MatrixXd) * Q.rows() * Q.cols(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_P_mat, P_mat.data(), sizeof(Eigen::MatrixXd) * P_mat.rows() * P_mat.cols(), cudaMemcpyHostToDevice);


    bool didFirstItr = false;
    for (int itr = 0; itr < n_itr; itr++) {
        // Find Correspondences
        int tot_dist = 0;

        //******* DO KERNEL HERE *******//
        // Launch the kernel
        find_correspondences<<<gridDim, blockDim>>>(d_P, d_Q, didFirstItr, d_P_mat, d_correspondences, d_prev_pc, d_curr_pc);
        // Copy correspondences back to host memory
        cudaMemcpy(correspondences, d_correspondences, P.cols() * sizeof(int), cudaMemcpyDeviceToHost);
        if (itr == 0) {
          cudaMemcpy(P.data(), d_P, sizeof(Eigen::MatrixXd) * P.rows() * P.cols(), cudaMemcpyDeviceToHost);
          cudaMemcpy(Q.data(), d_Q, sizeof(Eigen::MatrixXd) * Q.rows() * Q.cols(), cudaMemcpyDeviceToHost);
        }
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
        //     correspondences.push_back(corr_idx);
        // }
        //******* STOP KERNEL *******//

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

        // SVD Decomposition
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd U = svd.matrixU();
        Eigen::MatrixXd S = svd.singularValues().asDiagonal();
        Eigen::MatrixXd V = svd.matrixV();

        // Shift P's point cloud
        Eigen::MatrixXd R = U * V.transpose();
        Eigen::VectorXd t = mu_Q - R * mu_P;
        if (!(didFirstItr)) {
            for (size_t col = 0; col < P.cols(); col++) {
                P_mat.col(col) = P.col(col);
            } // P_mat = 2 x N
        }
        P_mat = R * P_mat + t.replicate(1,P_mat.cols());
        if (!(didFirstItr)) {
            didFirstItr = true;
        }
    }
    // Free host memory
    delete[] prev_pc_array;
    delete[] curr_pc_array;
    // Free device memory
    cudaFree(d_correspondences);
    cudaFree(d_prev_pc);
    cudaFree(d_curr_pc);
    cudaFree(d_P);
    cudaFree(d_Q);
    cudaFree(d_P_mat);
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
            P_mat = icp_parallel(icp_itr);
            icp_msg = get_pc_msg(P_mat);
            pc_pub.publish(icp_msg);
        }
        ros::spinOnce();
        rate.sleep();
    }
    //   ros::spin();

    return 0;
}

