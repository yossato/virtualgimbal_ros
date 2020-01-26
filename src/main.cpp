#include "ros/ros.h"
#include "vg_manager.h"
#include "synchronization_manager.h"

// #include <opencv2/highgui.hpp>
// #include <opencv2/plot.hpp>

// using namespace cv;

// int main(int argc, char** argv)
// {
//    // Plot data must be a 1xN or Nx1 matrix.
//    // Plot data type must be double (CV_64F) 
//     Mat data( 30, 1, CV_64F );
//     randu( data, 0, 500 ); // random values

//     Mat plot_result;

//     Ptr<plot::Plot2d> plot = plot::Plot2d::create(data);
//     plot->setPlotBackgroundColor( Scalar( 50, 50, 50 ) );
//     plot->setPlotLineColor( Scalar( 50, 50, 255 ) );
//     plot->render( plot_result );

//     imshow( "plot", plot_result );
//     waitKey();

//     plot->setPlotLineColor( Scalar( 50, 255, 255 ) );
//     data = data / 3;
//     plot->render( plot_result );

//     imshow( "plot", plot_result );
//     waitKey();

//     plot->setPlotGridColor( Scalar( 255, 0, 255 ) );
//     data = data * 4;
//     plot->render( plot_result );

//     imshow( "plot", plot_result );
//     waitKey();

//     plot->setPlotTextColor( Scalar( 255, 0, 0 ) );
//     randu( data, 100, 400 );
//     plot->render( plot_result );

//     imshow( "plot", plot_result );
//     waitKey();

//     return 0;
// }

int main(int argc, char **argv)
{
    ros::init(argc,argv, "virtualgimbal_ros");
    std::cout << "hello" << std::endl;
    double offset_time = 0.0;
    {
        virtualgimbal::synchronization_manager sync_mgr;
        offset_time = sync_mgr.estimate_offset_time();

    }
    ROS_INFO("offset_time:%f",offset_time);

    virtualgimbal::manager mgr;
    mgr.run();
    return 0;
}