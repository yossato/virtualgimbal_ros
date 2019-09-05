#include "ros/ros.h"
#include "vg_manager.h"

namespace virtualgimbal
{

manager::manager() : pnh_("~"), image_transport_(pnh_), q(1.0, 0, 0, 0)
{
    std::string image = "/image";
    std::string imu_data = "/imu_data";
    pnh_.param("image",image,image);
    pnh_.param("imu_data",imu_data,imu_data);
    ROS_INFO("image topic is %s",image.c_str());
    ROS_INFO("imu_data topic is %s",imu_data.c_str());
    camera_subscriber_ = image_transport_.subscribeCamera(image, 10, &manager::callback, this);
    imu_subscriber_ = pnh_.subscribe(imu_data, 1000, &manager::imu_callback, this);
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

#define EPS_Q 1E-4

    template <typename T_num>
    Eigen::Quaternion<T_num> Vector2Quaternion(Eigen::Vector3d w)
    {
        double theta = w.norm(); //sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);//回転角度を計算、normと等しい
        //0割を回避するためにマクローリン展開
        if (theta > EPS_Q)
        {
            Eigen::Vector3d n = w.normalized(); //w * (1.0/theta);//単位ベクトルに変換
            //            double sin_theta_2 = sin(theta*0.5);
            //            return Eigen::Quaternion<T_num>(cos(theta*0.5),n[0]*sin_theta_2,n[1]*sin_theta_2,n[2]*sin_theta_2);
            Eigen::VectorXd n_sin_theta_2 = n * sin(theta * 0.5);
            return Eigen::Quaternion<T_num>(cos(theta * 0.5), n_sin_theta_2[0], n_sin_theta_2[1], n_sin_theta_2[2]);
        }
        else
        {
            return Eigen::Quaternion<T_num>(1.0, 0.5 * w[0], 0.5 * w[1], 0.5 * w[2]).normalized();
        }
    }

void manager::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    Eigen::Vector3d w(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    
    if(imu_previous){   // Previous data is exist.
        ros::Duration diff = (msg->header.stamp - imu_previous->header.stamp);
        Eigen::Vector3d dq = w * diff.toSec();
        q = q*Vector2Quaternion<double>(dq);
        ROS_INFO("%f %f %f %f",q.w(),q.x(),q.y(),q.z());
    }
    imu_previous = msg;
    
}

} // namespace virtualgimbal