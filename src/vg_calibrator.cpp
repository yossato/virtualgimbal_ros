/* 
 * Software License Agreement (BSD 3-Clause License)
 * 
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

/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.
                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)
Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.
This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#include "vg_calibrator.h"
#include <opencv2/aruco/charuco.hpp>
#include <ros/package.h>
namespace virtualgimbal
{



calibrator::calibrator() : pnh_("~"), image_transport_(nh_), q(1.0, 0, 0, 0), q_filtered(1.0, 0, 0, 0),
 last_vector(0, 0, 0), param(pnh_),
 zoom_(1.3f),cutoff_frequency_(0.5),enable_trimming_(true),
 offset_time_(ros::Duration(0.0)), verbose(false), allow_blue_space(false), lms_period_(1.5), lms_order_(1),
 arr_(pnh_)
{
    std::string image = "/image_rect";
    std::string imu_data = "/imu_data";
    pnh_.param("image", image, image);
    pnh_.param("imu_data", imu_data, imu_data);
    ROS_INFO("image topic is %s", image.c_str());
    ROS_INFO("imu_data topic is %s", imu_data.c_str());

    pnh_.param("zoom_factor",zoom_,zoom_);
    if(zoom_ < 1.0)
    {
        ROS_ERROR("zoom_factor must be larger than 1.0.");
        throw;
    }
    pnh_.param("cutoff_frequency",cutoff_frequency_,cutoff_frequency_);
    pnh_.param("enable_trimming",enable_trimming_,enable_trimming_);
    
    double offset_time_double = offset_time_.toSec();
    pnh_.param("offset_time",offset_time_double,offset_time_double);
    offset_time_ = ros::Duration(offset_time_double);

    pnh_.param("verbose",verbose,verbose);
    pnh_.param("allow_blue_space",allow_blue_space,allow_blue_space);

    pnh_.param("lsm_period",lms_period_,lms_period_);
    pnh_.param("lsm_order",lms_order_,lms_order_);

    initializeDetection();

    camera_subscriber_ = image_transport_.subscribeCamera(image, 100, &calibrator::callback, this);
    imu_subscriber_ = pnh_.subscribe(imu_data, 10000, &calibrator::imuCallback, this);

    camera_publisher_ = image_transport_.advertiseCamera("stabilized/image_rect",1);

    if(verbose)
    {
        raw_quaternion_queue_size_pub = pnh_.advertise<std_msgs::Float64>("raw_quaternion_queue_size", 10);
        filtered_quaternion_queue_size_pub = pnh_.advertise<std_msgs::Float64>("filtered_quaternion_queue_size", 10);
    }

    raw_angle_quaternion = Rotation(verbose);

    // OpenCL
    // initializeCL(context);
}

calibrator::~calibrator()
{
    cv::destroyAllWindows();
}

MatrixPtr calibrator::getR(ros::Time time, double ratio){
    Eigen::Quaterniond raw, filtered;
    MatrixPtr R(new std::vector<float>(camera_info_->height_ * 9));
    
    assert(ratio >= 0.0);
    assert((ratio - 1.0) < std::numeric_limits<double>::epsilon());

    filtered = filtered_angle_quaternion.get_interpolate(time);

    for (int row = 0, e = camera_info_->height_; row < e; ++row)
    {
        raw = raw_angle_quaternion.get_interpolate(time + ros::Duration(camera_info_->line_delay_ * (row - camera_info_->height_ * 0.5)));

        Eigen::Quaterniond q = filtered.conjugate() * raw;
        Eigen::Vector3d vec = Quaternion2Vector(q) * ratio;
        Eigen::Quaterniond q2 = Vector2Quaternion<double>(vec );

        q2 = raw.conjugate() * q2 * raw;

        Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(&(*R)[row * 9], 3, 3) =
            q2.matrix().cast<float>();
    }
    return R;
}

MatrixPtr calibrator::getR_LMS(ros::Time time, const ros::Time begin, const ros::Time end, int order, double ratio)
{
    Eigen::Quaterniond raw, filtered;
    MatrixPtr R(new std::vector<float>(camera_info_->height_ * 9));
    
    assert(ratio >= 0.0);
    assert((ratio - 1.0) < std::numeric_limits<double>::epsilon());


    Eigen::Quaterniond correction_quaternion = raw_angle_quaternion.get_correction_quaternion_using_least_squares_method(begin,end,time,order);
    

    if(0)
    {
        printf("rw,rx,ry,rz,cw,cx,cy,cz\r\n");
        raw = raw_angle_quaternion.get_interpolate(time);
        printf("%f,%f,%f,%f,%f,%f,%f,%f\r\n",raw.w(),raw.x(),raw.y(),raw.z(),
        correction_quaternion.w(),correction_quaternion.x(),correction_quaternion.y(),correction_quaternion.z());
    }

    for (int row = 0, e = camera_info_->height_; row < e; ++row)
    {
        raw = raw_angle_quaternion.get_interpolate(time + ros::Duration(camera_info_->line_delay_ * (row - camera_info_->height_ * 0.5)));

        Eigen::Quaterniond q = correction_quaternion.conjugate() * raw;
        Eigen::Vector3d vec = Quaternion2Vector(q) * ratio;
        Eigen::Quaterniond q2 = Vector2Quaternion<double>(vec).normalized();

        q2 = (raw.conjugate() * q2 * raw).normalized();

        Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(&(*R)[row * 9], 3, 3) =
            q2.matrix().cast<float>();
    }
    return R;
}

void calibrator::initializeDetection()
{
    arr_.detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers
    // detectorParams->
    
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(arr_.dictionary_id_));


    // create board object
    gridboard_ =
        cv::aruco::GridBoard::create(arr_.markers_X_, arr_.markers_Y_,arr_. marker_length_, arr_.marker_separation_, dictionary_);
    board_ = gridboard_.staticCast<cv::aruco::Board>();
}

void calibrator::detectMarkers(const cv::Mat &image, const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs)
{
    ids_.clear();
    rejected_.clear();
    corners_.clear();
        

    // detect markers
    cv::aruco::detectMarkers(image, dictionary_, corners_, ids_, arr_.detector_params_, rejected_);

    // refind strategy to detect more markers
    if(arr_.refind_strategy_)
        cv::aruco::refineDetectedMarkers(image, board_, corners_, ids_, rejected_, cam_matrix,
                                        dist_coeffs);
}

int calibrator::estimatePoseBoard(const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, cv::Vec3d &rvec, cv::Vec3d &tvec)
{
    // estimate board pose
    int markers_of_board_detected = 0;
    if(ids_.size() > 0)
    {
        markers_of_board_detected = cv::aruco::estimatePoseBoard(corners_, ids_, board_, cam_matrix, dist_coeffs, rvec, tvec);
    }
    return markers_of_board_detected;
}

void calibrator::estimatePoseSingleMarkers(const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs)
{
    cv::aruco::estimatePoseSingleMarkers(corners_, 0.05, cam_matrix, dist_coeffs, rvecs, tvecs);
}

cv::Mat calibrator::drawSingleMarkersResults(const cv::Mat &image, const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs)
{
    cv::Mat image_copy;
    // draw results
    image.copyTo(image_copy);
    for (int i = 0; i < rvecs.size(); ++i) {
        auto rvec = rvecs[i];
        auto tvec = tvecs[i];
        cv::aruco::drawAxis(image_copy, cam_matrix, dist_coeffs, rvec, tvec, 0.1);
    }
        
    return image_copy;
    // cv::imshow("out", imageCopy);
}

cv::Mat calibrator::drawResults(const cv::Mat &image, const int markers_of_board_detected,const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, cv::Vec3d &rvec, cv::Vec3d &tvec)
{
    cv::Mat image_copy;
    // draw results
    image.copyTo(image_copy);
    if(ids_.size() > 0) {
        cv::aruco::drawDetectedMarkers(image_copy, corners_, ids_);
    }

    if(arr_.show_rejected_ && rejected_.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(image_copy, rejected_, cv::noArray(), cv::Scalar(100, 0, 255));
    }
        
    if(markers_of_board_detected > 0)
    {
            float axisLength = 0.5f * ((float)std::min(arr_.markers_X_, arr_.markers_Y_) * (arr_.marker_length_ + arr_.marker_separation_) +
                               arr_.marker_separation_);
        cv::aruco::drawAxis(image_copy, cam_matrix, dist_coeffs, rvec, tvec, axisLength);
    }
    return image_copy;
    // cv::imshow("out", imageCopy);
}

void calibrator::callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &ros_camera_info)
{

    {
        // Write aruco marker code here.
        cv::Vec3d rvec, tvec;
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat cam_matrix = (cv::Mat_<double>(3,3) << ros_camera_info->P[0], ros_camera_info->P[1], ros_camera_info->P[2], ros_camera_info->P[4], ros_camera_info->P[5], ros_camera_info->P[6],
        ros_camera_info->P[8], ros_camera_info->P[9], ros_camera_info->P[10]);

        cv::Mat dist_coeffs;
        if(ros_camera_info->D.size()){
            dist_coeffs = cv::Mat(cv::Size(1,ros_camera_info->D.size()),CV_64FC1);
            memcpy(dist_coeffs.data,ros_camera_info->D.data(),ros_camera_info->D.size()*sizeof(double));
        }

        // std::cout << "cam_matrix:\r\n" << cam_matrix << std::endl;

        detectMarkers(cv_ptr->image,cam_matrix,dist_coeffs);

        int markers_of_board_detected = estimatePoseBoard(cam_matrix,dist_coeffs,rvec,tvec);

        cv::Mat result_board_image = drawResults(cv_ptr->image,markers_of_board_detected,cam_matrix,dist_coeffs,rvec,tvec);
        cv::imshow("Board",result_board_image);

        std::vector<cv::Vec3d> rvecs, tvecs;
        estimatePoseSingleMarkers(cam_matrix,dist_coeffs,rvecs,tvecs);
        cv::Mat result_single_markers_image = drawSingleMarkersResults(cv_ptr->image,cam_matrix,dist_coeffs,rvecs,tvecs);
        cv::imshow("Single markers",result_single_markers_image);

        // char key = (char)cv::waitKey(1);
        // if(key == 'q') std::exit(EXIT_SUCCESS);
    }

    // if (!camera_info_)
    // {
    //     camera_info_ = std::make_shared<CameraInformation>(std::string("ros_camera"), ros_camera_info->distortion_model, Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
    //                                                        ros_camera_info->width, ros_camera_info->height, ros_camera_info->P[0],
    //                                                        ros_camera_info->P[5], ros_camera_info->P[2], ros_camera_info->P[6],
    //                                                        ros_camera_info->D[0], ros_camera_info->D[1], ros_camera_info->D[2],
    //                                                        ros_camera_info->D[3], param.line_delay);
    //     ros_camera_info_ = ros_camera_info;
    // }

    // if (image_previous)
    // {
    //     // Jamp to back
    //     if ((image->header.stamp - image_previous->header.stamp).toSec() < 0)
    //     {
    //         ROS_INFO("image time stamp jamp is detected.");
    //         src_image.clear();
    //         image_previous = nullptr;
    //     }
    // }

    // cv_bridge::CvImageConstPtr cv_ptr;
    // try
    // {
    //     cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGRA8);
    // }
    // catch (cv_bridge::Exception &e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }

    // cv::UMat umat_src = cv_ptr->image.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

    // if (umat_src.empty())
    // {
    //     ROS_WARN("Input image is empty.");
    //     return;
    // }
    // // TODO: check channel

    // // Push back umat
    // src_image.push_back(image->header.stamp, umat_src);

    // // TODO: Limit queue size
    // image_previous = image;
}

void calibrator::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // if (!std::isfinite(msg->angular_velocity.x + msg->angular_velocity.y + msg->angular_velocity.z + msg->header.stamp.toSec()))
    // {
    //     ROS_WARN("Input angular velocity and time stamp contains NaN. Skipped.");
    //     return;
    // }
    // Eigen::Quaterniond w_g(0.0, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // if (imu_previous)
    // { // Previous data is exist.
    //     ros::Duration diff = (msg->header.stamp - imu_previous->header.stamp);
    //     if (diff.toSec() < 0)
    //     {
    //         ROS_INFO("Jump");
    //         imu_previous = nullptr;
    //         raw_angle_quaternion.clear();
    //         filtered_angle_quaternion.clear();
    //         return;
    //     }

    //     Eigen::Quaterniond w_o = q * w_g * q.conjugate();
    //     Eigen::Vector3d dq = Eigen::Vector3d(w_o.x(),w_o.y(),w_o.z()) * diff.toSec();

        
    //     q = q * Vector2Quaternion<double>(dq);
    //     q.normalize();
    //     // Bilinear transform, prewarping
    //     float w_c = 2*CV_PI*cutoff_frequency_;
    //     float T = diff.toSec();
    //     float w_a = tan(w_c*T/2.f);
    //     float a1 = (1.f - w_a) / (1.f + w_a); 


    //     Eigen::Vector3d vec = a1 * Quaternion2Vector<double>((q_filtered * q.conjugate()).normalized(), last_vector);
    //     q_filtered =  Vector2Quaternion<double>(vec) * q;
    //     q_filtered.normalize();

    //     last_vector = vec;

    //     raw_angle_quaternion.push_back(msg->header.stamp, q);
    //     filtered_angle_quaternion.push_back(msg->header.stamp, q_filtered);

    // }
    // imu_previous = msg;
}

cv::Mat create_markers_image()
{
    bool showImage = true;
    int squaresX = 6;
    int squaresY = 4;
    int squareLength = 150;
    int markerLength = 100;
    int dictionaryId = cv::aruco::DICT_6X6_250;
    int margins = squareLength - markerLength;
    int borderBits = 1;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    cv::Size imageSize;
    imageSize.width = squaresX * squareLength + 2 * margins;
    imageSize.height = squaresY * squareLength + 2 * margins;

    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, (float)squareLength,
                                                            (float)markerLength, dictionary);

    // show created board
    cv::Mat boardImage;
    board->draw(imageSize, boardImage, margins, borderBits);

    return boardImage;
}

cv::Mat create_markers_image2(const ArucoRos &ar)
{
    bool showImage = true;
    // int markersX = 6;
    // int markersY = 4;
    // int markerSeparation = 150;
    // int markerLength = 100;
    // int dictionaryId = cv::aruco::DICT_6X6_250;
    // int margins = markerSeparation;
    // int borderBits = 1;


    cv::Size imageSize;
    imageSize.width = ar.markers_X_ * (ar.marker_length_ + ar.marker_separation_) - ar.marker_separation_ + 2 * ar.marker_separation_;
    imageSize.height =
        ar.markers_Y_ * (ar.marker_length_ + ar.marker_separation_) - ar.marker_separation_ + 2 * ar.marker_separation_;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(ar.dictionary_id_));

    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(ar.markers_X_, ar.markers_Y_, float(ar.marker_length_),
                                                      float(ar.marker_separation_), dictionary);

    // show created board
    cv::Mat boardImage;
    board->draw(imageSize, boardImage, ar.marker_separation_, ar.detector_params_->markerBorderBits);

    return boardImage;
}



void calibrator::run()
{
    cv::Mat aruco_board = create_markers_image2(arr_);
    
        ros::Rate rate(120);
    while (ros::ok())
    {
        ros::spinOnce();
        cv::imshow("aruco_board",aruco_board);
        char key = (char)cv::waitKey(1);
        if(key == 'q')
        {
            break;
        }
        rate.sleep();

    


    }
}

} // namespace virtualgimbal