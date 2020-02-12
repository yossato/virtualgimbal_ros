#include "ros/ros.h"
#include "vg_manager.h"
#include "synchronization_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "virtualgimbal_ros");
    // std::cout << "hello" << std::endl;
    // double offset_time = 0.0;
    // {
    //     virtualgimbal::synchronization_manager sync_mgr;
    //     offset_time = sync_mgr.estimate_offset_time();
    // }
    // ROS_INFO("offset_time:%f",offset_time);

    virtualgimbal::manager mgr;
    mgr.run();
    return 0;
}