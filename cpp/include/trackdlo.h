#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>

#include <ctime>
#include <chrono>
#include <thread>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef TRACKDLO_H
#define TRACKDLO_H

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using cv::Mat;

bool ecpd_lle (MatrixXf X_orig,
               MatrixXf& Y,
               double& sigma2,
               double beta,
               double alpha,
               double gamma,
               double mu,
               int max_iter = 30,
               double tol = 0.00001,
               bool include_lle = true,
               bool use_geodesic = false,
               bool use_prev_sigma2 = false,
               bool use_ecpd = false,
               std::vector<MatrixXf> correspondence_priors = {},
               double omega = 0,
               std::string kernel = "Gaussian",
               std::vector<int> occluded_nodes = {},
               double k_vis = 0,
               Mat bmask_transformed_normalized = Mat::zeros(cv::Size(0, 0), CV_64F),
               double mat_max = 0);

void tracking_step (MatrixXf X_orig,
                    MatrixXf& Y,
                    double& sigma2,
                    MatrixXf& gn_result,
                    std::vector<MatrixXf>& priors_result,
                    std::vector<double> geodesic_coord,
                    Mat bmask_transformed_normalized,
                    double mask_dist_threshold,
                    double mat_max);

class trackdlo
{
    public:
        // default constructor
        trackdlo(int num_of_nodes);
        // fancy constructor
        trackdlo(int num_of_nodes,
                 MatrixXf Y,
                 double sigma2,
                 double beta,
                 double lambda,
                 double alpha,
                 double gamma,
                 double k_vis,
                 double mu,
                 int max_iter,
                 double tol,
                 bool include_lle,
                 bool use_geodesic,
                 bool use_prev_sigma2,
                 std::string kernel);

        double get_sigma2();
        MatrixXf get_tracking_result();
        MatrixXf get_guide_nodes();
        std::vector<MatrixXf> get_correspondence_pairs();
        void set_geodesic_coord (std::vector<double> geodesic_coord);

        bool ecpd_lle (MatrixXf X_orig,
                        MatrixXf& Y,
                        double& sigma2,
                        double beta,
                        double lambda,
                        double gamma,
                        double mu,
                        int max_iter = 30,
                        double tol = 0.00001,
                        bool include_lle = true,
                        bool use_geodesic = false,
                        bool use_prev_sigma2 = false,
                        bool use_ecpd = false,
                        std::vector<MatrixXf> correspondence_priors = {},
                        double alpha = 0,
                        std::string kernel = "Gaussian",
                        std::vector<int> occluded_nodes = {},
                        double k_vis = 0,
                        Mat bmask_transformed_normalized = Mat::zeros(cv::Size(0, 0), CV_64F),
                        double mat_max = 0);
        void tracking_step (MatrixXf X_orig, Mat bmask_transformed_normalized, double mask_dist_threshold, double mat_max);

    private:
        MatrixXf Y_;
        double sigma2_;
        double beta_;
        double lambda_;
        double alpha_;
        double gamma_;
        double k_vis_;
        double mu_;
        int max_iter_;
        double tol_;
        bool include_lle_;
        bool use_geodesic_;
        bool use_prev_sigma2_;
        std::string kernel_;
        std::vector<double> geodesic_coord_;

        std::vector<int> get_nearest_indices (int k, int M, int idx);
        MatrixXf calc_LLE_weights (int k, MatrixXf X);
        std::vector<MatrixXf> traverse_geodesic (std::vector<double> geodesic_coord, const MatrixXf guide_nodes, 
                                                 const std::vector<int> visible_nodes, int alignment);
        std::vector<MatrixXf> traverse_euclidean (std::vector<double> geodesic_coord, const MatrixXf guide_nodes, 
                                                  const std::vector<int> visible_nodes, int alignment, int alignment_node_idx = -1);

};

#endif