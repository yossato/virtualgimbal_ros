#include "ros/ros.h"
#include "vg_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "virtualgimbal_ros");
    std::cout << "hello" << std::endl;
    virtualgimbal::manager mgr;
    ros::spin();
    return 0;
}