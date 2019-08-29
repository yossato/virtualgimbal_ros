#include "ros/ros.h"
#include "vg_manager.h"

namespace virtualgimbal
{

manager::manager() : pnh_("~"), image_transport_(pnh_)
{
    camera_subscriber_ = image_transport_.subscribeCamera("image", 10, &manager::callback, this);
    imu_subscriber_ = pnh_.subscribe("imu_data",1000,&manager::imu_callback, this);
}

void manager::callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &camera_info)
{
}

void manager::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
}

} // namespace virtualgimbal