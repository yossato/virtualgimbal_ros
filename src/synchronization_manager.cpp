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
    
}

void synchronization_manager::callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &ros_camera_info)
{
    if(previous_image_)
    {
        
        Eigen::Vector3d optical_flow;
        // Get optical flow
        if(estimate_angular_velocity(cv_bridge::toCvShare(image)->image,cv_bridge::toCvShare(previous_image_)->image,optical_flow))
        {
            // Convert optical flow to angular velocity
            estimated_angular_velocity_.push_back(
                previous_image_->header.stamp + (image->header.stamp - previous_image_->header.stamp) * 0.5,
                optical_flow
            );
        }
        // cv_bridge::toCvShare(image)->image
    }
    else
    {
        previous_image_ = image;
    }
    
    
    // 画像を受け取り、前回の画像との差分から角速度を計算
    // 画像から推定した角速度を、time stampとともにdequeに保存
    // time stampは2枚の画像のタイムスタンプの平均値にする
    // 今回の画像を保存
}

void synchronization_manager::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // とにかくIMUのdequeに保存
}

double synchronization_manager::estimate_offset_time()
{
    // ROSのspinを続ける
    // 推定角速度と、IMUデータが十分な数貯まるまで待つ
    // データが十分に溜まったら、
    return 0.0;
}

} // namespace virtualgimbal
