#include "ros/ros.h"
#include "vg_manager.h"
#include "synchronization_manager.h"
int main(int argc, char **argv)
{
    ros::init(argc,argv, "virtualgimbal_ros");
    std::cout << "hello" << std::endl;
    virtualgimbal::synchronization_manager sync_mgr;
    double offset_time = sync_mgr.estimate_offset_time();
    ROS_INFO("offset_time:%f",offset_time);
    return 0;
    virtualgimbal::manager mgr;
    mgr.run();
    return 0;
}