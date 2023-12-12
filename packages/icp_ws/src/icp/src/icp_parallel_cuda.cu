#include <iostream>
#include <vector>
#include <time.h>

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

// __global__ void find_correspondences(
//   Eigen::MatrixXd* d_P,
//   Eigen::MatrixXd* d_Q,
//   bool didFirstItr,
//   Eigen::MatrixXd* d_P_mat,
//   int* d_correspondences,
//   geometry_msgs::Point32* d_prev_pc,
//   geometry_msgs::Point32* d_curr_pc) {
  
//   int i = blockIdx.x * blockDim.x + threadIdx.x;

//   if (i < d_P->cols()) {
//     Eigen::Vector2d p;
//     if (!didFirstItr) {
//       p << d_curr_pc[i].x, d_curr_pc[i].y;
//       d_P->col(i) = p;
//     } else {
//       p = d_P_mat->col(i);
//     }

//     double min_dist = 1000.0;
//     int corr_idx = -1;

//     // Parallel loop over Q
//     for (int j = threadIdx.y * blockDim.y; j < d_Q->cols(); j += blockDim.y) {
//       Eigen::Vector2d q;
//       q << d_prev_pc[j].x, d_prev_pc[j].y;
//       double curr_dist = (p - q).norm();
//       if (i == 0) {
//         d_Q->col(j) = q; 
//       }
//       if (curr_dist < min_dist) {
//         min_dist = curr_dist;
//         corr_idx = j;
//       }
//     }

//     d_correspondences[i] = corr_idx;
//   }
// }
__global__ void find_correspondences(
  double* d_P,
  double* d_Q,
  int* d_correspondences,
  const geometry_msgs::Point32* d_prev_pc,
  const geometry_msgs::Point32* d_curr_pc,
  const int num_pts, const bool didFirstItr) {
  
  int i = blockIdx.x * blockDim.x + threadIdx.x;

  if (i < num_pts) {
    double px, py;
    if (!didFirstItr) {
      px = d_curr_pc[i].x;
      py = d_curr_pc[i].y;
      d_P[i * 2] = px;
      d_P[i * 2 + 1] = py;
    } else {
      px = d_P[i * 2];
      py = d_P[i * 2 + 1];
    }

    double min_dist = 1000.0;
    int corr_idx = -1;

    // Parallel loop over Q
    for (int j = threadIdx.y * blockDim.y; j < num_pts; j += blockDim.y) {
      double qx, qy;
      qx = d_prev_pc[j].x;
      qy = d_prev_pc[j].y;
      if (i == 0) {
        d_Q[j * 2] = qx;
        d_Q[j * 2 + 1] = qy;
      }
      double curr_dist = std::sqrt((px - qx) * (px - qx) + (py - qy) * (py - qy));
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
    if (curr_pc.points.size() < prev_pc.points.size()) {
        P = Eigen::MatrixXd::Zero(2, curr_pc.points.size());
        Q = Eigen::MatrixXd::Zero(2, curr_pc.points.size());
    } else {
        P = Eigen::MatrixXd::Zero(2, prev_pc.points.size());
        Q = Eigen::MatrixXd::Zero(2, prev_pc.points.size());
    }
    int num_pts = P.cols();
    int correspondences[num_pts];
    // Convert point cloud data to arrays of Point32
    geometry_msgs::Point32* prev_pc_array = new geometry_msgs::Point32[num_pts];
    geometry_msgs::Point32* curr_pc_array = new geometry_msgs::Point32[num_pts];
    for (int i = 0; i < num_pts; i++) {
        prev_pc_array[i] = prev_pc.points[i];
        curr_pc_array[i] = curr_pc.points[i];
    }

    // CUDA Defintions
    // Set grid and block dimensions
    dim3 blockDim(16, 1);
    dim3 gridDim(P.cols() / blockDim.x + (P.cols() % blockDim.x != 0), 1);

    // Allocate device memory for P, Q, P_mat, and correspondences
    double* d_P;
    double* d_Q;
    int* d_correspondences;
    geometry_msgs::Point32* d_prev_pc;
    geometry_msgs::Point32* d_curr_pc;
    cudaMalloc(&d_P, sizeof(double) * P.rows() * P.cols());
    cudaMalloc(&d_Q, sizeof(double) * Q.rows() * Q.cols());
    cudaMalloc(&d_correspondences, sizeof(int) * num_pts);
    cudaMalloc(&d_prev_pc, sizeof(geometry_msgs::Point32) * num_pts);
    cudaMalloc(&d_curr_pc, sizeof(geometry_msgs::Point32) * num_pts);
    
    // Copy kernel variables to device memory
    cudaMemcpy(d_P, P.data(), sizeof(double) * P.rows() * P.cols(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_Q, Q.data(), sizeof(double) * Q.rows() * Q.cols(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_correspondences, correspondences, sizeof(int) * num_pts, cudaMemcpyHostToDevice);
    cudaMemcpy(d_prev_pc, prev_pc_array, sizeof(geometry_msgs::Point32) * num_pts, cudaMemcpyHostToDevice);
    cudaMemcpy(d_curr_pc, curr_pc_array, sizeof(geometry_msgs::Point32) * num_pts, cudaMemcpyHostToDevice);
    
    bool didFirstItr = false;
    for (int itr = 0; itr < n_itr; itr++) {
        // Find Correspondences
        //******* DO KERNEL HERE *******//
        // Launch the kernel
        find_correspondences<<<gridDim, blockDim>>>(d_P, d_Q, d_correspondences, d_prev_pc, d_curr_pc, num_pts, didFirstItr);
        cudaDeviceSynchronize(); // wait for all blocks in the launch to finish processing
        // Copy correspondences back to host memory
        cudaMemcpy(correspondences, d_correspondences, sizeof(int) * num_pts, cudaMemcpyDeviceToHost);
        if (itr == 0) {
          cudaMemcpy(P.data(), d_P, sizeof(double) * P.rows() * P.cols(), cudaMemcpyDeviceToHost);
          cudaMemcpy(Q.data(), d_Q, sizeof(double) * Q.rows() * Q.cols(), cudaMemcpyDeviceToHost);
        }
        //******* STOP KERNEL *******//

        // Compute Mean of Point Clouds & Cross-Covariance
        Eigen::Vector2d mu_Q = compute_mean(Q);
        Eigen::Vector2d mu_P = compute_mean(P);
        Eigen::Matrix2d cov = compute_cross_covariance(P,Q,correspondences);

        // SVD Decomposition
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd U = svd.matrixU();
        Eigen::MatrixXd S = svd.singularValues().asDiagonal();
        Eigen::MatrixXd V = svd.matrixV();

        // Shift P's point cloud
        Eigen::MatrixXd R = U * V.transpose();
        Eigen::VectorXd t = mu_Q - R * mu_P;
        P = R * P + t.replicate(1,P.cols());
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
    /* END ICP ALGORITHM */
    return P;
}

sensor_msgs::PointCloud get_pc_msg(Eigen::MatrixXd P) {
    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "sonic/lidar_frame";
    for (int i = 0; i < P.cols(); i++) {
        geometry_msgs::Point32 point;
        point.x = P.col(i)[0];
        point.y = P.col(i)[1];
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
    struct timespec start_time, end_time;
    Eigen::MatrixXd P;
    sensor_msgs::PointCloud icp_msg;
    int icp_itr;
    while (ros::ok()) {
        if (!(prev_pc.points.empty()) && !(curr_pc.points.empty())) {
            if (!nh.getParam("icp_itr", icp_itr))
            {
                ROS_ERROR("Failed to get parameter 'icp_itr'. Using default value of 1.");
                icp_itr = 1;  // Default value
            }
            clock_gettime(CLOCK_MONOTONIC, &start_time);
            P = icp_parallel(icp_itr);
            clock_gettime(CLOCK_MONOTONIC, &end_time);
            long double elapsed_time = (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_nsec - start_time.tv_nsec) / 1e9L;
            std::cout << "Execution time: " << elapsed_time << " seconds" << std::endl;
            icp_msg = get_pc_msg(P);
            pc_pub.publish(icp_msg);
        }
        ros::spinOnce();
        rate.sleep();
    }
    //   ros::spin();

    return 0;
}

