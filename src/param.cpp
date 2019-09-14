#include "param.h"

namespace virtualgimbal
{
    Parameters::Parameters(ros::NodeHandle &pnh) : line_delay(0.0){
        pnh.param("line_delay",line_delay,line_delay);
        
    }
}