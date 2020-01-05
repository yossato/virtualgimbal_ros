#ifndef __VIRTUALGIMAL_ROS_LEAST_SQUARES_METHOD_H__
#define __VIRTUALGIMAL_ROS_LEAST_SQUARES_METHOD_H__

#include <Eigen/Dense>
namespace virtualgimbal
{
    
Eigen::VectorXd least_squares_method(Eigen::VectorXd &x, Eigen::VectorXd &y, int degree);


}

#endif //__VIRTUALGIMAL_ROS_LEAST_SQUARES_METHOD_H__