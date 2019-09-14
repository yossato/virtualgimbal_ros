#ifndef __VIRTUALGIMAL_ROS_ROTATION_H_
#define __VIRTUALGIMAL_ROS_ROTATION_H_
#include <Eigen/Dense>
#include <deque>
#include "ros/ros.h"

namespace virtualgimbal
{

class rotation
{
public:
    rotation();
    void push_back(ros::Time time, Eigen::Quaterniond &q);
    void pop_front(ros::Time time);
    size_t size();
    int get(ros::Time time, Eigen::Quaterniond &q);
private:
    std::deque<std::pair<ros::Time,Eigen::Quaterniond>> data;

};

}

#endif //__VIRTUALGIMAL_ROS_ROTATION_H_