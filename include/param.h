#ifndef __VIRTUALGIMBAL_ROS_PARAM_H__
#define __VIRTUALGIMBAL_ROS_PARAM_H__

#include "ros/ros.h"
namespace virtualgimbal
{
class Parameters
{
public:
    Parameters(ros::NodeHandle &pnh);
    double line_delay;
private:

    
};
} // namespace virtualgimbal

#endif //__VIRTUALGIMBAL_ROS_PARAM_H__