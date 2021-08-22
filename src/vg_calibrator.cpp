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
    initializeCL(context);
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

void calibrator::drawResults(const cv::Mat &image, const int markers_of_board_detected,const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, cv::Vec3d &rvec, cv::Vec3d &tvec)
{
    cv::Mat imageCopy;
    // draw results
    image.copyTo(imageCopy);
    if(ids_.size() > 0) {
        cv::aruco::drawDetectedMarkers(imageCopy, corners_, ids_);
    }

    if(arr_.show_rejected_ && rejected_.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(imageCopy, rejected_, cv::noArray(), cv::Scalar(100, 0, 255));
    }
        
    if(markers_of_board_detected > 0)
    {
            float axisLength = 0.5f * ((float)std::min(arr_.markers_X_, arr_.markers_Y_) * (arr_.marker_length_ + arr_.marker_separation_) +
                               arr_.marker_separation_);
        cv::aruco::drawAxis(imageCopy, cam_matrix, dist_coeffs, rvec, tvec, axisLength);
    }
        
    cv::imshow("out", imageCopy);
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

        detectMarkers(cv_ptr->image,cam_matrix,dist_coeffs);

        int markers_of_board_detected = estimatePoseBoard(cam_matrix,dist_coeffs,rvec,tvec);

        drawResults(cv_ptr->image,markers_of_board_detected,cam_matrix,dist_coeffs,rvec,tvec);

        char key = (char)cv::waitKey(1);
        if(key == 'q') std::exit(EXIT_SUCCESS);
    }

    if (!camera_info_)
    {
        camera_info_ = std::make_shared<CameraInformation>(std::string("ros_camera"), ros_camera_info->distortion_model, Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                                                           ros_camera_info->width, ros_camera_info->height, ros_camera_info->P[0],
                                                           ros_camera_info->P[5], ros_camera_info->P[2], ros_camera_info->P[6],
                                                           ros_camera_info->D[0], ros_camera_info->D[1], ros_camera_info->D[2],
                                                           ros_camera_info->D[3], param.line_delay);
        ros_camera_info_ = ros_camera_info;
    }

    if (image_previous)
    {
        // Jamp to back
        if ((image->header.stamp - image_previous->header.stamp).toSec() < 0)
        {
            ROS_INFO("image time stamp jamp is detected.");
            src_image.clear();
            image_previous = nullptr;
        }
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGRA8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::UMat umat_src = cv_ptr->image.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

    if (umat_src.empty())
    {
        ROS_WARN("Input image is empty.");
        return;
    }
    // TODO: check channel

    // Push back umat
    src_image.push_back(image->header.stamp, umat_src);

    // TODO: Limit queue size
    image_previous = image;
}

void calibrator::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (!std::isfinite(msg->angular_velocity.x + msg->angular_velocity.y + msg->angular_velocity.z + msg->header.stamp.toSec()))
    {
        ROS_WARN("Input angular velocity and time stamp contains NaN. Skipped.");
        return;
    }
    Eigen::Quaterniond w_g(0.0, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    if (imu_previous)
    { // Previous data is exist.
        ros::Duration diff = (msg->header.stamp - imu_previous->header.stamp);
        if (diff.toSec() < 0)
        {
            ROS_INFO("Jump");
            imu_previous = nullptr;
            raw_angle_quaternion.clear();
            filtered_angle_quaternion.clear();
            return;
        }

        Eigen::Quaterniond w_o = q * w_g * q.conjugate();
        Eigen::Vector3d dq = Eigen::Vector3d(w_o.x(),w_o.y(),w_o.z()) * diff.toSec();

        
        q = q * Vector2Quaternion<double>(dq);
        q.normalize();
        // Bilinear transform, prewarping
        float w_c = 2*CV_PI*cutoff_frequency_;
        float T = diff.toSec();
        float w_a = tan(w_c*T/2.f);
        float a1 = (1.f - w_a) / (1.f + w_a); 


        Eigen::Vector3d vec = a1 * Quaternion2Vector<double>((q_filtered * q.conjugate()).normalized(), last_vector);
        q_filtered =  Vector2Quaternion<double>(vec) * q;
        q_filtered.normalize();

        last_vector = vec;

        raw_angle_quaternion.push_back(msg->header.stamp, q);
        filtered_angle_quaternion.push_back(msg->header.stamp, q_filtered);

    }
    imu_previous = msg;
}

ros::Time calibrator::get_begin_time(ros::Time time)
{
    ros::Time begin_time;
    if (camera_info_->line_delay_ >= 0.0)
    {
        begin_time = time + ros::Duration(camera_info_->line_delay_ * (0 - camera_info_->height_ * 0.5));
    }
    else
    {
        begin_time = time + ros::Duration(camera_info_->line_delay_ * ((camera_info_->height_ - 1) - camera_info_->height_ * 0.5));
    }
    return begin_time;
}

ros::Time calibrator::get_end_time(ros::Time time)
{
    ros::Time end_time;
    if (camera_info_->line_delay_ >= 0.0)
    {
        end_time = time + ros::Duration(camera_info_->line_delay_ * ((camera_info_->height_ - 1) - camera_info_->height_ * 0.5));
    }
    else
    {
        end_time = time + ros::Duration(camera_info_->line_delay_ * (0 - camera_info_->height_ * 0.5));
    }
    return end_time;
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

    if(showImage) {
        cv::imshow("board", boardImage);
        cv::waitKey(0);
    }

    return boardImage;
}

cv::Mat create_markers_image2()
{
    bool showImage = true;
    int markersX = 6;
    int markersY = 4;
    int markerSeparation = 150;
    int markerLength = 100;
    int dictionaryId = cv::aruco::DICT_6X6_250;
    int margins = markerSeparation;
    int borderBits = 1;


    cv::Size imageSize;
    imageSize.width = markersX * (markerLength + markerSeparation) - markerSeparation + 2 * margins;
    imageSize.height =
        markersY * (markerLength + markerSeparation) - markerSeparation + 2 * margins;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markersX, markersY, float(markerLength),
                                                      float(markerSeparation), dictionary);

    // show created board
    cv::Mat boardImage;
    board->draw(imageSize, boardImage, margins, borderBits);

    if(showImage) {
        cv::imshow("board", boardImage);
        cv::waitKey(0);
    }

    return boardImage;
}



void calibrator::run()
{
    create_markers_image();
    create_markers_image2();
    return ;
    
    std::string kernel_path = ros::package::getPath("virtualgimbal_ros")+"/cl/stabilizer_kernel.cl";
    ROS_INFO("kernel path:%s",  kernel_path.c_str());
    const char *kernel_function = "stabilizer_function";

    ros::Rate rate(120);
    while (ros::ok())
    {
        ros::spinOnce();
        cv::waitKey(1);
        rate.sleep();

        if(!camera_info_) continue;

        // Show debug information
        if (verbose)
        {
            std_msgs::Float64 msg;
            msg.data = (double)raw_angle_quaternion.size();
            raw_quaternion_queue_size_pub.publish(msg);
            std_msgs::Float64 msg2;
            msg2.data = (double)filtered_angle_quaternion.size();
            filtered_quaternion_queue_size_pub.publish(msg2);
        }

        // Is gyro availavle?
        if (raw_angle_quaternion.size())
        {
            // Get first angle of gyro sensor
            auto time_gyro_front = raw_angle_quaternion.front().first;
            
            ros::Duration half_height_delay = ros::Duration(fabs(camera_info_->line_delay_) * camera_info_->height_ * 0.5);
            
            // Time stamp of image. The image should be later than this time.
            auto time_image_request = time_gyro_front + offset_time_ + half_height_delay;
            if(!src_image.is_available_after(time_image_request))
            {
                continue;
            }

            // Get time stamp of center row of the image
            ros::Time time_image_center_line;
            auto image = src_image.get(time_image_request,time_image_center_line);
            
            
            // Check availability of gyro angle data at the time stamp of the last row of the image
            auto time_gyro_last_line = time_image_center_line + half_height_delay - offset_time_;
            auto time_gyro_center_line = time_image_center_line - offset_time_;
            auto time_gyro_first_line = time_image_center_line - half_height_delay - offset_time_;
            if(!raw_angle_quaternion.is_available_after(time_gyro_last_line)) continue;

            MatrixPtr R2 = getR_LMS(time_gyro_center_line,time_gyro_last_line-ros::Duration(lms_period_),time_gyro_last_line,lms_order_ );
            if(!allow_blue_space)
            {
                double ratio = bisectionMethod(zoom_,R2,camera_info_,0.0,1.0,1000,0.001);//TODO:zoomをなくす
                if(ratio < (1.0 - std::numeric_limits<double>::epsilon()))
                {
                    if(verbose){
                        std::cout << "ratio:" << ratio << std::endl;
                    }
                    R2 = getR_LMS(time_gyro_center_line,time_gyro_last_line-ros::Duration(lms_period_),time_gyro_last_line,lms_order_ ,ratio);
                }
            }

            

            float ik1 = camera_info_->inverse_k1_;
            float ik2 = camera_info_->inverse_k2_;
            float ip1 = camera_info_->inverse_p1_;
            float ip2 = camera_info_->inverse_p2_;
            float fx = camera_info_->fx_;
            float fy = camera_info_->fy_;
            float cx = camera_info_->cx_;
            float cy = camera_info_->cy_;


            // Define destinatino image 
            int image_width_dst,image_height_dst;
            float fx_dst,fy_dst,cx_dst,cy_dst,line_delay_dst;
            if(enable_trimming_)
            {
                image_width_dst = image.cols / zoom_;
                image_height_dst = image.rows / zoom_;
                fx_dst = fx;
                fy_dst = fy;
                cx_dst = cx - (image.cols - image_width_dst)*0.5;
                cy_dst = cy - (image.rows - image_height_dst)*0.5;
                line_delay_dst = camera_info_->line_delay_;
            }
            else
            {
                image_width_dst = image.cols;
                image_height_dst = image.rows;
                fx_dst = fx*zoom_;
                fy_dst = fy*zoom_;
                cx_dst = cx;
                cy_dst = cy;
                line_delay_dst = camera_info_->line_delay_/zoom_;

            }
            if(!dst_camera_info_)
            {
                dst_camera_info_ = std::make_shared<CameraInformation>("dst_camera","dst_lens",Eigen::Quaterniond(1.0,0.,0.,0.),image_width_dst,image_height_dst,
                fx_dst,fy_dst,cx_dst ,cy_dst,0.,0.,0.,0.,line_delay_dst);
            }
            UMatPtr umat_dst_ptr(new cv::UMat(cv::Size(image_width_dst,image_height_dst), CV_8UC4, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY));
           
            // Send arguments to kernel
            cv::ocl::Image2D image_src(image);

            cv::Mat mat_R = cv::Mat(R2->size(), 1, CV_32F, R2->data());
            cv::UMat umat_R = mat_R.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
            cv::ocl::Kernel kernel;
            getKernel(kernel_path.c_str(), kernel_function, kernel, context, build_opt);
            kernel.args(image_src, cv::ocl::KernelArg::WriteOnly(*umat_dst_ptr), cv::ocl::KernelArg::ReadOnlyNoSize(umat_R),ik1,ik2,ip1,ip2,fx,fy,cx,cy,fx_dst,fy_dst,cx_dst,cy_dst);
            size_t globalThreads[3] = {(size_t)image.cols, (size_t)image.rows, 1};
            bool success = kernel.run(3, globalThreads, NULL, true);
            if (!success)
            {
                std::cout << "Failed running the kernel..." << std::endl
                          << std::flush;
                throw "Failed running the kernel...";
            }

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(ros_camera_info_->header,"bgra8",umat_dst_ptr->getMat(cv::ACCESS_READ)).toImageMsg();
            sensor_msgs::CameraInfo info = *ros_camera_info_;
            info.header.stamp = time_image_center_line;
            if(enable_trimming_)
            {

            }
            else
            {
                info.K[0] *= zoom_; // fx
                info.K[4] *= zoom_; // fy
                info.P[0] *= zoom_; // fx
                info.P[5] *= zoom_; // fy
                info.width = image_width_dst;
                info.height = image_height_dst;
                //TODO : Modify cx and cy.
            }
            camera_publisher_.publish(*msg,info);

            raw_angle_quaternion.pop_old(time_gyro_first_line-ros::Duration(3.0));    // TODO:ジャイロと画像のオフセットを考慮
            filtered_angle_quaternion.pop_old(time_gyro_first_line-ros::Duration(3.0));
            src_image.pop_old_close(time_image_center_line);

        }
        else
        {
            continue;
        }


    }
}

} // namespace virtualgimbal