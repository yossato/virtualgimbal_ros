/*
 * Copyright (c) 2020, Yoshiaki Sato
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __VIRTUALGIMAL_ROS_VG_CALIBRATOR_H__
#define __VIRTUALGIMAL_ROS_VG_CALIBRATOR_H__

#include "ros/ros.h"
#include <ros/package.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "rotation.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <Eigen/Dense>
#include "param.h"
#include "camera_information.h"
#include "cl_manager.h"
#include "SO3Filters.h"
#include "aruco_board.h"
#include <opencv2/core/eigen.hpp>
#include "aruco_board.h"
#include <random>
namespace virtualgimbal
{
    using MatrixPtr = std::shared_ptr<std::vector<float>>;

    using Rotation = StampedDeque<Eigen::Quaterniond>;
    using ImageDeque = StampedDeque<cv::UMat>;
    using UMatPtr = std::unique_ptr<cv::UMat>;
    class calibrator
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        calibrator();
        ~calibrator();
        void run();

    private:
        void initializeDetection();
        void detectMarkers(const cv::Mat &image, const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs);
        int estimatePoseBoard(const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, cv::Vec3d &rvec, cv::Vec3d &tvec);
        void estimatePoseSingleMarkers(const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs);
        void estimatePoseSingleMarkersWithInitPose(const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs, cv::Vec3d &init_rvec, cv::Vec3d &init_tvec);
        cv::Mat drawResults(const cv::Mat &image, const int markers_of_board_detected, const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, cv::Vec3d &rvec, cv::Vec3d &tvec);
        cv::Mat drawSingleMarkersResults(const cv::Mat &image, const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs);
        void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &ros_camera_info);
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        std::vector<double> estimateRelativeZAxisAngles(cv::Vec3d &old_rvec, cv::Vec3d &current_rvec, std::vector<cv::Vec3d> &rvecs);
        cv::Mat drawPhase(const cv::Mat &image, std::vector<double> relative_z_axis_angles, cv::Scalar color = cv::Scalar(0, 255, 255));
        cv::Mat createMarkersImage2(const ArucoRos &ar);
        Eigen::Vector3d getDiffAngleVector(cv::Vec3d &old_rvec, cv::Vec3d &current_rvec);
        cv::Point2f getCenter(int i);
        Eigen::VectorXd calculateLinearEquationCoefficients(double dt, std::vector<double> relative_z_axis_angles);
        Eigen::VectorXd calculateLinearEquationCoefficientsRansac(std::vector<double> x, std::vector<double> y);
        Eigen::VectorXd drawPhaseLSM(double dt, Eigen::VectorXd coeffs, cv::Mat &image, cv::Scalar color = cv::Scalar(0, 255, 0));

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        image_transport::ImageTransport image_transport_;
        image_transport::CameraSubscriber camera_subscriber_;

        Parameters param;
        ArucoRos arr_;

        cv::Ptr<cv::aruco::Dictionary> dictionary_;
        cv::Ptr<cv::aruco::GridBoard> gridboard_;
        cv::Ptr<cv::aruco::Board> board_;
        std::vector<int> ids_;
        std::vector<std::vector<cv::Point2f>> corners_, rejected_;

        double min_angle_thres_;

        double maximum_relative_delay_ransac_;
        int maximum_iteration_ransac_;
        std::vector<double> vec_delay_, vec_v_;
        int minimum_number_of_data_ransac_;
        bool generate_aruco_board_;
        bool show_gui_;
    };

}

#endif //__VIRTUALGIMAL_ROS_VG_CALIBRATOR_H__