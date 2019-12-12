#include "synchronization_manager.h"
namespace virtualgimbal
{



synchronization_manager::synchronization_manager() : pnh_("~"), image_transport_(nh_), auto_sync_(false)
{
    std::string image = "/image";
    std::string imu_data = "/imu_data";
    pnh_.param("image", image, image);
    pnh_.param("imu_data", imu_data, imu_data);
    
    double offset_time_double = 0.5;
    pnh_.param("sad_time_diff", offset_time_double, offset_time_double);
    offset_time = ros::Duration(offset_time_double);

    pnh_.param("auto_sync",auto_sync_,auto_sync_);

    double sad_time_length_double = 5.0;
    pnh_.param("sad_period",sad_time_length_double,sad_time_length_double);
    sad_time_length =  ros::Duration(sad_time_length_double);

    ROS_INFO("image topic is %s", image.c_str());
    ROS_INFO("imu_data topic is %s", imu_data.c_str());

    camera_subscriber_ = image_transport_.subscribeCamera(image, 100, &synchronization_manager::callback, this);
    imu_subscriber_ = pnh_.subscribe(imu_data, 100, &synchronization_manager::imu_callback, this);

    estimated_angular_velocity_pub_ = pnh_.advertise<geometry_msgs::Vector3>("estimated_angular_velocity",100);
    measured_angular_velocity_pub_  = pnh_.advertise<geometry_msgs::Vector3>("measured_angular_velocity",100);
    
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
            angular_velocity << atan(optical_flow[1] / fy)/period, -atan(optical_flow[0] / fx)/period, - optical_flow[2]/period;
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
    geometry_msgs::Vector3 dst_msg;
    dst_msg.x = angular_velocity[0];
    dst_msg.y = angular_velocity[1];
    dst_msg.z = angular_velocity[2];
    measured_angular_velocity_pub_.publish(dst_msg);
}

double synchronization_manager::estimate_offset_time()
{
    if(!auto_sync_)
    {
        ROS_INFO("Auto sync is disabled");
        return 0.0;
    }
    else
    {
        ROS_INFO("Auto sync is enabled.");
    }
    

    // 推定角速度と、IMUデータが十分な数貯まるまで待つ
    // データが十分に溜まったら、計算
    ros::Duration min_offset_time;
    min_offset_time.nsec = std::numeric_limits<int32_t>::max();
    min_offset_time.sec = std::numeric_limits<int32_t>::max();

    // ROSのspinを続ける
    ros::Rate rate(120);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

        // 1.SADで計算する長さと、時間差について計算可能か調べる
        // Gyro(Measured)の角速度の先頭のタイムスタンプを取得
        if(!measured_angular_velocity_.size()) continue;
        auto measured_front_time = measured_angular_velocity_.front().first;
        // 画像から取得した角速度をの先頭のタイムスタンプを取得
        if(!estimated_angular_velocity_.size()) continue;
        auto estimated_front_time = estimated_angular_velocity_.front().first;
        
        // SAD計算区間の始点を求める
        ros::Time front_time;
        if((measured_front_time-estimated_front_time).toSec()>=0)
        {  
            front_time = measured_front_time;
        }
        else
        {
            front_time = estimated_front_time;
        }

        // SAD計算区間の終点を求める
        ros::Time back_time;
        auto measured_back_time = measured_angular_velocity_.back().first;
        auto estimated_back_time = estimated_angular_velocity_.back().first;
        if((measured_back_time-estimated_back_time).toSec()>=0)
        {
            back_time = estimated_back_time;
        }
        else
        {
            back_time = measured_back_time;
        }
        
        // 十分な長さがあるか調べる
        if(back_time-front_time < (offset_time * 2.0 +sad_time_length)) continue;

        // すべてのオフセットについて
        double min_sad = std::numeric_limits<double>::max();
        ros::Duration period(0.033); //とりあえず33msに指定
        for(ros::Duration dt = -offset_time; dt < offset_time; dt+= ros::Duration(0.001))
        {
            // すべてのフレームについて
            
            double sum = 0.0;
            int num = 0;
            for(ros::Time time = front_time+offset_time, e = front_time + offset_time + sad_time_length; time<e; time += period)
            {
                sum +=(estimated_angular_velocity_.get(time) - measured_angular_velocity_.get(time+dt)).squaredNorm();
                num++;
            }
            if(num == 0)
            {
                ROS_WARN("Logic error @ %s:%d",__FILE__,__LINE__);
                continue;
            }

            if((sum / num) < min_sad)
            {
                min_sad = sum/num;
                min_offset_time = dt;
            }

            ROS_INFO("dt:%f sad:%f num:%d",dt.toSec(),sum / num,num);
            
            
        }
        return min_offset_time.toSec();
    }
    
    
}

} // namespace virtualgimbal
