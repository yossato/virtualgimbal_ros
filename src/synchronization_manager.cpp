#include "synchronization_manager.h"
namespace virtualgimbal
{



synchronization_manager::synchronization_manager() : pnh_("~"), image_transport_(nh_)
{
    std::string image = "/image";
    std::string imu_data = "/imu_data";
    pnh_.param("image", image, image);
    pnh_.param("imu_data", imu_data, imu_data);
    ROS_INFO("image topic is %s", image.c_str());
    ROS_INFO("imu_data topic is %s", imu_data.c_str());

    camera_subscriber_ = image_transport_.subscribeCamera(image, 100, &synchronization_manager::callback, this);
    imu_subscriber_ = pnh_.subscribe(imu_data, 10000, &synchronization_manager::imu_callback, this);

    estimated_angular_velocity_pub_ = pnh_.advertise<geometry_msgs::Vector3>("estimated_angular_velocity",1000);
    measured_angular_velocity_pub_  = pnh_.advertise<geometry_msgs::Vector3>("measured_angular_velocity",1000);
    
}

void synchronization_manager::callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &ros_camera_info)
{
    if(previous_image_)
    {
        Eigen::Vector3d optical_flow;
        // Get optical flow
        if(0 == estimate_angular_velocity(cv_bridge::toCvShare(image)->image,cv_bridge::toCvShare(previous_image_)->image,optical_flow))
        {
            // Convert optical flow to angular velocity
            Eigen::Vector3d angular_velocity;
            const double &fx = ros_camera_info->K[0];
            const double &fy = ros_camera_info->K[4];
            double period = (image->header.stamp - previous_image_->header.stamp).toSec();
            angular_velocity << atan(optical_flow[0] / fx)/period, -atan(optical_flow[1] / fy)/period, - optical_flow[2]/period;
            // Save angular velocity with average time stamp between two images.
            estimated_angular_velocity_.push_back(
                previous_image_->header.stamp + (image->header.stamp - previous_image_->header.stamp) * 0.5,
                angular_velocity
            );

            geometry_msgs::Vector3 msg;
            msg.x = angular_velocity[0];
            msg.y = angular_velocity[1];
            msg.z = angular_velocity[2];
            estimated_angular_velocity_pub_.publish(msg);
        }
        // cv_bridge::toCvShare(image)->image
    }
    previous_image_ = image;
}

void synchronization_manager::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // とにかくIMUのdequeに保存
    Eigen::Vector3d angular_velocity;
    angular_velocity << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    measured_angular_velocity_.push_back(msg->header.stamp,angular_velocity);
}

double synchronization_manager::estimate_offset_time()
{
    // ROSのspinを続ける
    // 推定角速度と、IMUデータが十分な数貯まるまで待つ
    // データが十分に溜まったら、
    return 0.0;
}

} // namespace virtualgimbal
