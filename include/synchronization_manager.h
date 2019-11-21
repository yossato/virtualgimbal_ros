#ifndef __SYNCHRONIZATION_MANAGER_H__
#define __SYNCHRONIZATION_MANAGER_H__

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

namespace virtualgimbal
{
class synchronization_manager
{
public:
    synchronization_manager();
    double estimate_offset_time();
private:
    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& ros_camera_info);
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport image_transport_;
    image_transport::CameraSubscriber camera_subscriber_;
    ros::Subscriber imu_subscriber_;
};
} // namespace virtualgimbal

#endif //__SYNCHRONIZATION_MANAGER_H__