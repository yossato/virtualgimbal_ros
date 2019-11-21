#include "ros/ros.h"
#include "vg_manager.h"

namespace virtualgimbal
{

manager::manager() : pnh_("~"), image_transport_(nh_), q(1.0, 0, 0, 0), q_filtered(1.0, 0, 0, 0),
 last_vector(0, 0, 0), param(pnh_), publish_statistics(true),
 zoom_(1.3f),enable_black_space_removal_(true),cutoff_frequency_(0.5),enable_trimming_(true),offset_time_(0.0)
{
    std::string image = "/image";
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
    pnh_.param("enable_black_space_removal",enable_black_space_removal_,enable_black_space_removal_);
    pnh_.param("cutoff_frequency",cutoff_frequency_,cutoff_frequency_);
    pnh_.param("enable_trimming",enable_trimming_,enable_trimming_);
    pnh_.param("offset_time",offset_time_,offset_time_);

    camera_subscriber_ = image_transport_.subscribeCamera(image, 100, &manager::callback, this);
    imu_subscriber_ = pnh_.subscribe(imu_data, 10000, &manager::imu_callback, this);
    pub_ = image_transport_.advertise("camera/image", 1);

    camera_publisher_ = image_transport_.advertiseCamera("camera/stabilized/image_rect",1);

    raw_quaternion_pub              = pnh_.advertise<sensor_msgs::Imu>("angle/raw", 1000);
    filtered_quaternion_pub         = pnh_.advertise<sensor_msgs::Imu>("angle/filtered", 1000);
    estimated_angular_velocity_pub  = pnh_.advertise<sensor_msgs::Imu>("angular_velocity/estimate",1000);
    measured_augular_velocity_pub   = pnh_.advertise<sensor_msgs::Imu>("angular_velocity/measured",1000);

    raw_quaternion_queue_size_pub = pnh_.advertise<std_msgs::Float64>("raw_quaternion_queue_size", 10);
    filtered_quaternion_queue_size_pub = pnh_.advertise<std_msgs::Float64>("filtered_quaternion_queue_size", 10);

    // OpenCL
    initializeCL(context);
}

manager::~manager()
{
    cv::destroyAllWindows();
}

MatrixPtr manager::getR(double ratio){
    Eigen::Quaterniond raw, filtered;
    MatrixPtr R(new std::vector<float>(camera_info_->height_ * 9));
    
    assert(ratio >= 0.0);
    assert((ratio - 1.0) < std::numeric_limits<double>::epsilon());

    // Calculate Rotation matrix for each line
    if(fabs(ratio - 1.0) < std::numeric_limits<double>::epsilon())
    {
        for (int row = 0, e = camera_info_->height_; row < e; ++row)
        {
            int status = raw_angle_quaternion.get(src_image.front().first + ros::Duration(camera_info_->line_delay_ * (row - camera_info_->height_ * 0.5)+offset_time_), raw);

            if (DequeStatus::GOOD != status)
            {
                ROS_ERROR("Logic error at %s:%d",__FUNCTION__,__LINE__);
                throw;
            }
            status = filtered_angle_quaternion.get(src_image.front().first + ros::Duration(camera_info_->line_delay_ * (row - camera_info_->height_ * 0.5)+offset_time_), filtered);

            if (DequeStatus::GOOD != status)
            {
                ROS_ERROR("Logic error at %s:%d",__FUNCTION__,__LINE__);
                throw;
            }
            Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(&(*R)[row * 9], 3, 3) =
                (raw * filtered.conjugate()).matrix().cast<float>(); //順序合ってる？
        }
    }
    else
    {
        for (int row = 0, e = camera_info_->height_; row < e; ++row)
        {
            int status = raw_angle_quaternion.get(src_image.front().first + ros::Duration(camera_info_->line_delay_ * (row - camera_info_->height_ * 0.5)+offset_time_), raw);

            if (DequeStatus::GOOD != status)
            {
                ROS_ERROR("Logic error at %s:%d",__FUNCTION__,__LINE__);
                throw;
            }
            status = filtered_angle_quaternion.get(src_image.front().first + ros::Duration(camera_info_->line_delay_ * (row - camera_info_->height_ * 0.5)+offset_time_), filtered);

            if (DequeStatus::GOOD != status)
            {
                ROS_ERROR("Logic error at %s:%d",__FUNCTION__,__LINE__);
                throw;
            }

            Eigen::Quaterniond q = raw * filtered.conjugate();
            Eigen::Vector3d vec = Quaternion2Vector(q) * ratio;
            Eigen::Quaterniond q2 = Vector2Quaternion<double>(vec );

            Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(&(*R)[row * 9], 3, 3) =
                q2.matrix().cast<float>();
        }
    }


    return R;
}

void manager::callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &ros_camera_info)
{
    if (!camera_info_)
    {
        double line_delay;
        pnh_.param("line_delay",line_delay,0.0);
        camera_info_ = std::make_shared<CameraInformation>(std::string("ros_camera"), ros_camera_info->distortion_model, Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
                                                           ros_camera_info->width, ros_camera_info->height, ros_camera_info->P[0],
                                                           ros_camera_info->P[5], ros_camera_info->P[2], ros_camera_info->P[6],
                                                           ros_camera_info->D[0], ros_camera_info->D[1], ros_camera_info->D[2],
                                                           ros_camera_info->D[3], line_delay);
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
    // src_image.limit_data_length(10);
    // ROS_INFO("src_image.size():%lu",src_image.size());

    //publish image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(image->header, "bgr8", umat_src.getMat(cv::ACCESS_READ)).toImageMsg();
    pub_.publish(msg);
    image_previous = image;
}

void manager::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (!std::isfinite(msg->angular_velocity.x + msg->angular_velocity.y + msg->angular_velocity.z + msg->header.stamp.toSec()))
    {
        ROS_WARN("Input angular velocity and time stamp contains NaN. Skipped.");
        return;
    }
    Eigen::Vector3d w(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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
        Eigen::Vector3d dq = w * diff.toSec();
        q = q * Vector2Quaternion<double>(dq);
        q.normalize();
        // Bilinear transform, prewarping
        float w_c = 2*CV_PI*cutoff_frequency_;
        float T = diff.toSec();
        float w_a = tan(w_c*T/2.f);
        float a1 = (1.f - w_a) / (1.f + w_a); 


        Eigen::Vector3d vec = a1 * Quaternion2Vector<double>((q.conjugate() * q_filtered).normalized(), last_vector);
        q_filtered = q * Vector2Quaternion<double>(vec);
        q_filtered.normalize();

        last_vector = vec;

        sensor_msgs::Imu angle_raw, angle_filtered;
        angle_raw.header = msg->header;
        angle_raw.orientation.w = q.w();
        angle_raw.orientation.x = q.x();
        angle_raw.orientation.y = q.y();
        angle_raw.orientation.z = q.z();
        raw_quaternion_pub.publish(angle_raw);

        angle_filtered.header = msg->header;
        angle_filtered.orientation.w = q_filtered.w();
        angle_filtered.orientation.x = q_filtered.x();
        angle_filtered.orientation.y = q_filtered.y();
        angle_filtered.orientation.z = q_filtered.z();
        filtered_quaternion_pub.publish(angle_filtered);

        raw_angle_quaternion.push_back(msg->header.stamp, q);
        filtered_angle_quaternion.push_back(msg->header.stamp, q_filtered);

    }
    imu_previous = msg;
}

ros::Time manager::get_begin_time(ros::Time time)
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

ros::Time manager::get_end_time(ros::Time time)
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

void manager::run()
{

    const char *kernel_name = "cl/stabilizer_kernel.cl";
    const char *kernel_function = "stabilizer_function";

    ros::Rate rate(120);
    while (ros::ok())
    {

        // Show debug information
        if (publish_statistics)
        {
            std_msgs::Float64 msg;
            msg.data = (double)raw_angle_quaternion.size();
            raw_quaternion_queue_size_pub.publish(msg);
            std_msgs::Float64 msg2;
            msg2.data = (double)filtered_angle_quaternion.size();
            filtered_quaternion_queue_size_pub.publish(msg2);
        }

        // If an images are available.
        if (src_image.size())
        {
            MatrixPtr R;//(new std::vector<float>(camera_info_->height_ * 9));

            Eigen::Quaterniond raw, filtered;

            // Wait for IMU data ariving.
            if ((0 == raw_angle_quaternion.size()) || (0 == filtered_angle_quaternion.size()))
            {
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            // If an image is older than IMU angle time stamp, delete the image since the image is not be able to be synchronized with IMU.
            while ((DequeStatus::TIME_STAMP_IS_EARLIER_THAN_FRONT == raw_angle_quaternion.get(get_begin_time(src_image.front().first), raw)) ||
                   (DequeStatus::TIME_STAMP_IS_EARLIER_THAN_FRONT == filtered_angle_quaternion.get(get_begin_time(src_image.front().first), filtered)))
            {
                src_image.pop_front();
                ROS_WARN("Image is discarded, since Image has old time stamp.");
                if (!src_image.size())
                {
                    break;
                }
            }
            if (!src_image.size())
            {
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            // Try to get IMU angle.
            if ((DequeStatus::TIME_STAMP_IS_LATER_THAN_BACK == raw_angle_quaternion.get(get_end_time(src_image.front().first), raw)) ||
                (DequeStatus::TIME_STAMP_IS_LATER_THAN_BACK == filtered_angle_quaternion.get(get_end_time(src_image.front().first), filtered)))
            {
                // If angles are not available, wait for it.
                ROS_WARN("Waiting for IMU data.");
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            // Calculate Rotation matrix for each line
            R = getR();
            double ratio = bisectionMethod(zoom_,R,camera_info_,0.0,1.0,1000,0.001);//TODO:zoomをなくす
            ROS_INFO("ratio:%f",ratio);
            R = getR(ratio);

            if (0)
            {
                ROS_INFO("Matrix:");
                std::cout << Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(&(*R)[0 * 9], 3, 3) << std::endl
                          << std::flush;
                ROS_INFO("Received image's time stamp: %d.%d, Quarternion:\r\n", src_image.front().first.sec, src_image.front().first.nsec);
                if (DequeStatus::GOOD != raw_angle_quaternion.get(src_image.front().first, raw))
                {
                    ROS_WARN("TIMING ERROR");
                }
                else
                {
                    std::cout << raw.coeffs() << std::endl
                              << std::flush;
                }
            }

            // Pop old angle quaternions
            if (camera_info_->line_delay_ >= 0)
            {
                raw_angle_quaternion.pop_old(src_image.front().first + ros::Duration(camera_info_->line_delay_ * ((camera_info_->height_ - 1) - camera_info_->height_ * 0.5)+offset_time_));
                filtered_angle_quaternion.pop_old(src_image.front().first + ros::Duration(camera_info_->line_delay_ * ((camera_info_->height_ - 1) - camera_info_->height_ * 0.5)+offset_time_));
            }
            else
            {
                raw_angle_quaternion.pop_old(src_image.front().first + ros::Duration(camera_info_->line_delay_ * (0 - camera_info_->height_ * 0.5)+offset_time_));
                filtered_angle_quaternion.pop_old(src_image.front().first + ros::Duration(camera_info_->line_delay_ * (0 - camera_info_->height_ * 0.5)+offset_time_));
            }

            float ik1 = camera_info_->inverse_k1_;
            float ik2 = camera_info_->inverse_k2_;
            float ip1 = camera_info_->inverse_p1_;
            float ip2 = camera_info_->inverse_p2_;
            float fx = camera_info_->fx_;
            float fy = camera_info_->fy_;
            float cx = camera_info_->cx_;
            float cy = camera_info_->cy_;
            // float zoom = 1.f;

            auto time = src_image.front().first;
            auto &image = src_image.front().second;

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
            cv::ocl::Image2D image_dst(*umat_dst_ptr, false, true);
           
            // Send arguments to kernel
            cv::ocl::Image2D image_src(image);

            cv::Mat mat_R = cv::Mat(R->size(), 1, CV_32F, R->data());
            cv::UMat umat_R = mat_R.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
            cv::ocl::Kernel kernel;
            getKernel(kernel_name, kernel_function, kernel, context, build_opt);
            kernel.args(image_src, image_dst, cv::ocl::KernelArg::ReadOnlyNoSize(umat_R),ik1,ik2,ip1,ip2,fx,fy,cx,cy,fx_dst,fy_dst,cx_dst,cy_dst);
            size_t globalThreads[3] = {(size_t)image.cols, (size_t)image.rows, 1};
            //size_t localThreads[3] = { 16, 16, 1 };
            bool success = kernel.run(3, globalThreads, NULL, true);
            if (!success)
            {
                std::cout << "Failed running the kernel..." << std::endl
                          << std::flush;
                throw "Failed running the kernel...";
            }

            // cv::imshow("received image", src_image.front().second);
            src_image.pop_front();
            // cv::imshow("Stabilized image", *umat_dst_ptr);


            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(ros_camera_info_->header,"bgra8",umat_dst_ptr->getMat(cv::ACCESS_READ)).toImageMsg();
            sensor_msgs::CameraInfo info = *ros_camera_info_;
            info.header.stamp = time;
            if(enable_trimming_)
            {

            }
            else
            {
                info.K[0] *= zoom_; // fx
                info.K[4] *= zoom_; // fy
                info.P[0] *= zoom_; // fx
                info.P[5] *= zoom_; // fy
            }
            camera_publisher_.publish(*msg,info);


            // Show debug information
            // if (publish_statistics)
            // {
            //     // 画像のタイムスタンプを保存->次の周期に活かす
            //     // 画像も保存
            //     // IMUから新旧のタイムスタンプの中間の角速度を取得
            //     // 新旧画像から角速度を推定

            //     sensor_msgs::Imu measured, estimated;

            //     measured.header = ros_camera_info_->header;
            //     measured.header.stamp = imu_previous->header.stamp + (msg->header.stamp - imu_previous->header.stamp)*0.5;
            //     measured_augular_velocity_pub.publish(measured);

            //     estimated_angular_velocity_pub.publish(estimated);
            // }
        }

        char key = cv::waitKey(1);

        if ('q' == key)
        {
            cv::destroyAllWindows();
            ros::shutdown();
        }

        ros::spinOnce();
        rate.sleep();
    }
}

} // namespace virtualgimbal