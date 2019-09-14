#ifndef __VIRTUALGIMAL_ROS_VG_MANAGER_H__
#define __VIRTUALGIMAL_ROS_VG_MANAGER_H__

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include "rotation.h"
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "param.h"
namespace virtualgimbal
{
class manager
{
public:
    manager();
    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info);
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport image_transport_;
    image_transport::CameraSubscriber camera_subscriber_;
    ros::Subscriber imu_subscriber_;
    image_transport::Publisher pub_ ;
    Eigen::Quaterniond q,q_filtered;
    sensor_msgs::Imu::ConstPtr imu_previous = nullptr;
    ros::Publisher raw_quaternion_pub,filtered_quaternion_pub;  
    Eigen::Vector3d last_vector;

    

    rotation raw_angle_quaternion;
    rotation filtered_angle_quaternion;

    Parameters param; 
};

} // namespace virtualgimbal

#endif //__VIRTUALGIMAL_ROS_VG_MANAGER_H__