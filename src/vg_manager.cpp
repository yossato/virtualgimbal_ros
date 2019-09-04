#include "ros/ros.h"
#include "vg_manager.h"

namespace virtualgimbal
{

manager::manager() : pnh_("~"), image_transport_(pnh_)
{
    camera_subscriber_ = image_transport_.subscribeCamera("image", 10, &manager::callback, this);
    imu_subscriber_ = pnh_.subscribe("imu_data", 1000, &manager::imu_callback, this);
     pub_ = image_transport_.advertise("camera/image", 1);
}

void manager::callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &camera_info)
{
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
    cv::UMat umat_src = cv_ptr->image.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

    cv::imshow("received image", umat_src);
    cv::waitKey(1);

    //publish image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(image->header, "bgr8", umat_src.getMat(cv::ACCESS_READ)).toImageMsg();
    pub_.publish(msg);
}

void manager::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
}

} // namespace virtualgimbal