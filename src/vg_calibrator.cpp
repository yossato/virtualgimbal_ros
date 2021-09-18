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
 arr_(pnh_),min_angle_thres_(0.05), maximum_relative_delay_ransac_(0.01), maximum_iteration_ransac_(10000), minimum_number_of_data_ransac_(10000), generate_aruco_board_(false), show_gui_(true)
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

    pnh_.param("minimum_angle_thresh",min_angle_thres_,min_angle_thres_);

    pnh_.param("maximum_angle_distance_ransac",maximum_relative_delay_ransac_,maximum_relative_delay_ransac_);
    pnh_.param("maximum_iteration_ransac",maximum_iteration_ransac_,maximum_iteration_ransac_);
    pnh_.param("minimum_number_of_data_ransac",minimum_number_of_data_ransac_,minimum_number_of_data_ransac_);

    pnh_.param("generate_aruco_board",generate_aruco_board_,generate_aruco_board_);
    pnh_.param("show_gui",show_gui_,show_gui_);

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

void calibrator::estimatePoseSingleMarkersWithInitPose(const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs, cv::Vec3d &init_rvec, cv::Vec3d &init_tvec)
{
    for(int i=0;i<corners_.size();++i)
    {
        rvecs.push_back(init_rvec);
        tvecs.push_back(init_tvec);
    }
    estimatePoseSingleMarkersWithInitialPose(corners_, 0.05, cam_matrix, dist_coeffs, rvecs, tvecs);
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
        cv::aruco::drawAxis(image_copy, cam_matrix, dist_coeffs, rvec, tvec, 0.05);
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

        // Debug 
        // cv::Mat result_board_image = drawResults(cv_ptr->image,markers_of_board_detected,cam_matrix,dist_coeffs,rvec,tvec);
        // cv::imshow("Board",result_board_image);

        std::vector<cv::Vec3d> rvecs, tvecs;
        estimatePoseSingleMarkersWithInitPose(cam_matrix,dist_coeffs,rvecs,tvecs,rvec,tvec);

        cv::Mat result_single_markers_image;
        if(show_gui_)
        {
            result_single_markers_image = drawSingleMarkersResults(cv_ptr->image,cam_matrix,dist_coeffs,rvecs,tvecs);
            // cv::imshow("Single markers",result_single_markers_image);
        }

        static cv::Vec3d old_rvec = rvec;
        std::vector<double> relative_z_axis_angles = estimateRelativeZAxisAngles(old_rvec,rvec,rvecs);

        // Get frame rate
        // static double fps = 0;
        static ros::Time old_time = ros_camera_info->header.stamp;
        ros::Duration dt = ros_camera_info->header.stamp - old_time;
        
        old_time = ros_camera_info->header.stamp;

        if (dt.isZero())
        {
            return;
        }

        Eigen::VectorXd linear_equation_coeffs(2);
        linear_equation_coeffs << std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN();

        cv::Mat result_phases;

        bool angle_diff_is_large = std::fabs(getDiffAngleVector(old_rvec, rvec).z()) > min_angle_thres_;
        if(angle_diff_is_large)
        {
            if(show_gui_)
            {
                result_phases = drawPhase(result_single_markers_image,relative_z_axis_angles);
            }
            
            linear_equation_coeffs = calculateLinearEquationCoefficients(dt.toSec(),relative_z_axis_angles);
            drawPhaseLSM(dt.toSec(),linear_equation_coeffs,result_phases);
        }
        else
        {
            if(show_gui_)
            {
                result_phases = drawPhase(result_single_markers_image,relative_z_axis_angles,cv::Scalar(127,127,127));
            }
        }
        old_rvec = rvec;

        // size_t minimum_number_of_data_ransac_ = 5000;
        // static std::vector<double> vec_delay_,vec_v_;
        
        if(angle_diff_is_large)
        {
            // std::copy(relative_z_axis_angles.begin(),relative_z_axis_angles.end(),std::back_inserter(line_delay_buff));
            for(const auto &el:relative_z_axis_angles)
            {
                vec_delay_.push_back(el * dt.toSec()); // Convert a unit from relative angle to second.
            }
            for(int i =0;i<relative_z_axis_angles.size();++i)
            {
                vec_v_.push_back(getCenter(i).y);
            }
        }
        
        ROS_INFO_THROTTLE(5, "Capturing delay data: %lu / %d", vec_delay_.size(),minimum_number_of_data_ransac_);

        // if((int)vec_delay_.size() >= minimum_number_of_data_ransac_)
        // {
        //     Eigen::VectorXd linear_equation_coeffs = calculateLinearEquationCoefficientsRansac(vec_v_,vec_delay_);
        //     Eigen::VectorXd coeff_to_show;
        //     coeff_to_show = linear_equation_coeffs;
        //     coeff_to_show = coeff_to_show * 1./dt.toSec(); // Scaling to shows
        //     drawPhaseLSM(dt.toSec(),coeff_to_show,result_phases,cv::Scalar(255,255,0));
        // }

        if(!result_phases.empty() && show_gui_)
        {
            cv::imshow("phase",result_phases);
        }     

    }

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

cv::Mat createMarkersImage()
{
    bool show_image = true;
    int squares_x = 6;
    int squares_y = 4;
    int square_length = 150;
    int marker_length = 100;
    int dictionary_id = cv::aruco::DICT_6X6_250;
    int margins = square_length - marker_length;
    int border_bits = 1;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

    cv::Size imageSize;
    imageSize.width = squares_x * square_length + 2 * margins;
    imageSize.height = squares_y * square_length + 2 * margins;

    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squares_x, squares_y, (float)square_length,
                                                            (float)marker_length, dictionary);

    // show created board
    cv::Mat boardImage;
    board->draw(imageSize, boardImage, margins, border_bits);

    return boardImage;
}

cv::Mat calibrator::createMarkersImage2(const ArucoRos &ar)
{
    bool show_image = true;
    // int markersX = 6;
    // int markersY = 4;
    // int markerSeparation = 150;
    // int markerLength = 100;
    // int dictionaryId = cv::aruco::DICT_6X6_250;
    // int margins = markerSeparation;
    // int borderBits = 1;


    cv::Size image_size;
    image_size.width = ar.markers_X_ * (ar.marker_length_ + ar.marker_separation_) - ar.marker_separation_ + 2 * ar.marker_separation_;
    image_size.height =
        ar.markers_Y_ * (ar.marker_length_ + ar.marker_separation_) - ar.marker_separation_ + 2 * ar.marker_separation_;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(ar.dictionary_id_));

    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(ar.markers_X_, ar.markers_Y_, float(ar.marker_length_),
                                                      float(ar.marker_separation_), dictionary);

    // show created board
    cv::Mat boardImage;
    board->draw(image_size, boardImage, ar.marker_separation_, ar.detector_params_->markerBorderBits);

    return boardImage;
}

Eigen::Vector3d calibrator::getDiffAngleVector(cv::Vec3d &old_rvec, cv::Vec3d &current_rvec)
{
    Eigen::Vector3d old_eigen_vec,current_eigen_vec;

    cv::cv2eigen(old_rvec,old_eigen_vec);
    Eigen::Quaterniond old_q = Vector2Quaternion<double>(old_eigen_vec);

    cv::cv2eigen(current_rvec,current_eigen_vec);
    Eigen::Quaterniond current_q = Vector2Quaternion<double>(current_eigen_vec);
    
    Eigen::Quaterniond diff_old_new_q = (old_q * current_q.conjugate()).normalized();
    Eigen::Vector3d diff_old_new_vec = Quaternion2Vector(diff_old_new_q);


    return diff_old_new_vec;
}

std::vector<double> calibrator::estimateRelativeZAxisAngles(cv::Vec3d &old_rvec, cv::Vec3d &current_rvec, std::vector<cv::Vec3d> &rvecs)
{
    // Eigen::Vector3d old_eigen_vec,current_eigen_vec;

    // cv::cv2eigen(old_rvec,old_eigen_vec);
    // Eigen::Quaterniond old_q = Vector2Quaternion<double>(old_eigen_vec);

    // cv::cv2eigen(current_rvec,current_eigen_vec);
    // Eigen::Quaterniond current_q = Vector2Quaternion<double>(current_eigen_vec);
    
    // Eigen::Quaterniond diff_old_new_q = old_q * current_q.conjugate();
    // Eigen::Vector3d diff_old_new_vec = Quaternion2Vector(diff_old_new_q);
    Eigen::Vector3d diff_old_new_vec = getDiffAngleVector(old_rvec, current_rvec);

    Eigen::Vector3d current_eigen_vec;
    cv::cv2eigen(current_rvec,current_eigen_vec);
    Eigen::Quaterniond current_q = Vector2Quaternion<double>(current_eigen_vec);

    std::vector<double> relative_z_axis_angles;
    int i=0;
    for(auto &r:rvecs)
    {
        // std::cout << i++ << ":" << r[2] << std::endl;
        Eigen::Vector3d r_eigen_vec;
        cv::cv2eigen(r,r_eigen_vec);
        Eigen::Quaterniond q = Vector2Quaternion<double>(r_eigen_vec);

        Eigen::Quaterniond diff_q = q * current_q.conjugate();
        Eigen::Vector3d diff_vec = Quaternion2Vector(diff_q);
        relative_z_axis_angles.push_back(diff_vec.z()/diff_old_new_vec.z());
        // relative_z_axis_angles.push_back(diff_vec.z());
    }
    return relative_z_axis_angles;
}

cv::Mat calibrator::drawPhase(const cv::Mat &image, std::vector<double> relative_z_axis_angles, cv::Scalar color)
{
    cv::Mat image_copy;
    image.copyTo(image_copy);

    int width = image.cols * 0.1;

    for(int i=0;i<relative_z_axis_angles.size();++i)
    {
        cv::Point2f center = (corners_[i][0] + corners_[i][1] + corners_[i][2] + corners_[i][3]) * 0.25;
        
        // Calculate destination u coordinate. To prevent slowdonw, limit it's range.
        double dest_u = relative_z_axis_angles[i] * width + image.cols/2;
        dest_u = std::min(double(image.cols-1), dest_u);
        dest_u = std::max(0.0, dest_u);

        cv::line(image_copy,cv::Point(image.cols/2,center.y),cv::Point(dest_u, center.y),color);
    }
    cv::line(image_copy,cv::Point(image.cols/2,0),cv::Point(image.cols/2,image_copy.rows-1),color);
    
    return image_copy;
}

cv::Point2f calibrator::getCenter(int i)
{
    cv::Point2f center = (corners_[i][0] + corners_[i][1] + corners_[i][2] + corners_[i][3]) * 0.25;
    return center;
}

Eigen::VectorXd calibrator::calculateLinearEquationCoefficients(double dt, std::vector<double> relative_z_axis_angles)
{
    Eigen::VectorXd x(relative_z_axis_angles.size()),y(relative_z_axis_angles.size());
    for(int i=0;i<relative_z_axis_angles.size();++i)
    {
        cv::Point2f center = getCenter(i);
        x[i] = center.y;
        y[i] = relative_z_axis_angles[i];
    }
    Eigen::VectorXd coeffs = least_squares_method(x,y,1);
    // ROS_INFO("dt: %f LSM: %f %f",dt, coeffs[0],coeffs[1]);
    return coeffs;
}

Eigen::VectorXd calibrator::calculateLinearEquationCoefficientsRansac(std::vector<double> x, std::vector<double> y)
{
    // double maximum_angle_distance_ransac_ = 0.5;
    // int maximum_iteration_ransac_ = 1000;
    int maximum_inlier = 0;
    double a_best,b_best;

    // Eigen::MatrixXd pm(vec_v.size(),2);
    // Eigen::MatrixXd mat_vec_v = Eigen::Map<Eigen::MatrixXd>(vec_v.data(),1,vec_v.size());
    // Eigen::MatrixXd rz = Eigen::Map<Eigen::MatrixXd>(relative_z_axis_angles.data(),1,relative_z_axis_angles.size());
    // pm.block(0,0,pm.rows(),1) = mat_vec_v;
    // pm.block(0,1,pm.rows(),1) = rz;
    // std::cout << "pm:\r\n" << pm << std::endl;

    std::random_device seed_gen;
    std::mt19937 engine(seed_gen());
    std::uniform_int_distribution<uint64_t> get_rand_uni_int( 0, y.size()-1 );
    for(int n=0; n<maximum_iteration_ransac_; ++n)
    {
        int i0 = get_rand_uni_int(engine);
        int i1 = get_rand_uni_int(engine);
        // u = a*v + b
        // a = (u(i1) - u(i0)) / (v(i1) - v(i0))
        double a = (y[i1] - y[i0]) / (x[i1] - x[i0]);
        double b = y[i1] - a * x[i1];
        
        // Count a number of inlier
        int inlier = 0;
        for(int i = 0;i<y.size();++i)
        {
            double diff = std::fabs( a * x[i] + b - y[i]);
            if(diff <= maximum_relative_delay_ransac_)
            {
                ++inlier;
            } 
        }
        if(inlier > maximum_inlier)
        {
            maximum_inlier = inlier;
            a_best = a;
            b_best = b;
         }
    }

    if(maximum_inlier == 0)
    {
        ROS_ERROR("Maximum inlier is zero. @ %s %d",__FILE__,__LINE__);    
        Eigen::VectorXd retval(2);
        retval << std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN();
        return retval;
    }

    // std::cout << "inlier:\r\n" << maximum_inlier << "/" << relative_z_axis_angles.size() << std::endl;
    // std::cout << "a_best:\r\n" << a_best << std::endl;
    // std::cout << "b_best:\r\n" << b_best << std::endl;
    

    // Extract inlers
    Eigen::VectorXd x_in(maximum_inlier),y_in(maximum_inlier);
    size_t inlier_index = 0;
    for(int i=0;i<y.size();++i)
    {
        double diff = std::fabs( a_best * x[i] + b_best - y[i]);
        if(diff <= maximum_relative_delay_ransac_)
        {
            x_in[inlier_index] = (x[i]);
            y_in[inlier_index] = (y[i]);
            ++inlier_index;
        } 
    }

    // Get best coeffs
    Eigen::VectorXd coeffs =  least_squares_method(x_in,y_in,1);
    ROS_INFO("Inlier:%d / %lu Line_delay:%.8f [second]",maximum_inlier,y.size(),-coeffs[1]);
    return coeffs;
}

Eigen::VectorXd calibrator::drawPhaseLSM(double dt, Eigen::VectorXd coeffs, cv::Mat &image, cv::Scalar color)
{
    int width = image.cols * 0.1;
    //  = calculateLinearEquationCoefficients(dt, relative_z_axis_angles);

    cv::line(image, cv::Point(image.cols/2 + width * (coeffs[1]*0                + coeffs[0]),     0), 
                    cv::Point(image.cols/2 + width * (coeffs[1]*(image.rows-1)   + coeffs[0]),  (image.rows-1)),
                    color,3);

    return coeffs;
}

void calibrator::run()
{
    cv::Mat aruco_board = createMarkersImage2(arr_);

    if(generate_aruco_board_)
    {
        std::string output_png_path = ros::package::getPath("virtualgimbal_ros") + "/output/aruco_board.png";
        cv::imwrite(output_png_path.c_str(),aruco_board);
    }

    ros::Rate rate(120);
    while (ros::ok())
    {
        ros::spinOnce();
        if(show_gui_)
        {
            cv::imshow("aruco_board",aruco_board);
        }
        char key = (char)cv::waitKey(1);
        if(key == 'q')
        {
            break;
        }

        if((int)vec_delay_.size() >= minimum_number_of_data_ransac_)
        {
            Eigen::VectorXd linear_equation_coeffs = calculateLinearEquationCoefficientsRansac(vec_v_,vec_delay_);
            // Eigen::VectorXd coeff_to_show;
            // coeff_to_show = linear_equation_coeffs;
            // coeff_to_show = coeff_to_show * 1./dt.toSec(); // Scaling to shows
            // drawPhaseLSM(dt.toSec(),coeff_to_show,result_phases,cv::Scalar(255,255,0));
            camera_subscriber_.shutdown();
            imu_subscriber_.shutdown();
            ros::shutdown();
            ros::spinOnce();
        }

        rate.sleep();

    


    }
}

} // namespace virtualgimbal