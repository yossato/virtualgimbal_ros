/* 
 * Software License Agreement (BSD 3-Clause License)
 * 
 * Copyright (c) 2020, Yoshiaki Sato
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
    ros::Duration offset_time; // How long change time between measured and estimated angular velocity.[sec]
    ros::Duration sad_time_length; // How long time calculate SAD [sec]
};
} // namespace virtualgimbal

#endif //__SYNCHRONIZATION_MANAGER_H__