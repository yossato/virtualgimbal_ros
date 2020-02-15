#include "ros/ros.h"
#include "vg_manager.h"
#include "synchronization_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "virtualgimbal_ros");
    virtualgimbal::manager mgr;
    mgr.run();
    return 0;
}