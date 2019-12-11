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
#include "angular_velocity_estimator.h"
#include "rotation.h"
namespace virtualgimbal
{

using AngularVelocity = StampedDeque<Eigen::Vector3d>;

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
    sensor_msgs::ImageConstPtr previous_image_;

    AngularVelocity estimated_angular_velocity_;
    AngularVelocity measured_angular_velocity_;

    ros::Publisher estimated_angular_velocity_pub_, measured_angular_velocity_pub_;

    // Synchronize parameter
    double offset_time; // How long change time between measured and estimated angular velocity.[sec]
    double sad_time_length; // How long time calculate SAD [sec]
    
};
} // namespace virtualgimbal

#endif //__SYNCHRONIZATION_MANAGER_H__