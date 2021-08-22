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


namespace virtualgimbal
{
using MatrixPtr = std::shared_ptr<std::vector<float>>;

using Rotation = StampedDeque<Eigen::Quaterniond>;
using Image = StampedDeque<cv::UMat>;
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
    int  estimatePoseBoard(const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, cv::Vec3d &rvec, cv::Vec3d &tvec);
    void drawResults();
    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& ros_camera_info);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport image_transport_;
    image_transport::CameraSubscriber camera_subscriber_;
    image_transport::CameraPublisher camera_publisher_;
    ros::Subscriber imu_subscriber_;
    Eigen::Quaterniond q,q_filtered;
    sensor_msgs::Imu::ConstPtr imu_previous = nullptr;
    sensor_msgs::ImageConstPtr image_previous = nullptr;
    ros::Publisher raw_quaternion_queue_size_pub,filtered_quaternion_queue_size_pub;  
    Eigen::Vector3d last_vector;
    MatrixPtr getR(ros::Time time, double ratio=1.0);
    MatrixPtr getR_LMS(ros::Time time, const ros::Time begin, const ros::Time end, int order, double ratio=1.0);
    ros::Time get_begin_time(ros::Time time);
    ros::Time get_end_time(ros::Time time);

    Rotation raw_angle_quaternion;
    Rotation filtered_angle_quaternion;

    Image src_image;

    Parameters param; 
    CameraInformationPtr camera_info_;
    CameraInformationPtr dst_camera_info_;
    sensor_msgs::CameraInfoConstPtr ros_camera_info_;

    // Image processing parameters
    float zoom_;
    float cutoff_frequency_;
    float a1_;
    bool enable_trimming_;
    ros::Duration offset_time_;

    // Prepare OpenCL
    cv::ocl::Context context;
    cv::String build_opt;

    // Debug
    bool verbose;
    bool allow_blue_space;

    // LMS
    double lms_period_;
    int lms_order_;

    ArucoRos arr_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::GridBoard> gridboard_;
    cv::Ptr<cv::aruco::Board> board_;
};

}

#endif  //__VIRTUALGIMAL_ROS_VG_CALIBRATOR_H__