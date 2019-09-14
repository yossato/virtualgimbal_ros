#include "rotation.h"

namespace virtualgimbal
{

    rotation::rotation(){
        
    }

    void rotation::push_back(ros::Time time, Eigen::Quaterniond &q){
        data.emplace_back(time,q);
    }
    
    void rotation::pop_front(ros::Time time){
        auto it = std::find_if(data.begin(),data.end(),[&](std::pair<ros::Time,Eigen::Quaterniond > x){return time < x.first;});
        if(data.begin() != it){
            data.erase(data.begin(),it);
        }
    }

    size_t rotation::size(){
        return data.size();
    }

    int rotation::get(ros::Time time, Eigen::Quaterniond &q){
        auto it = std::find_if(data.begin(),data.end(),[&](std::pair<ros::Time,Eigen::Quaterniond > x){return time < x.first;});
        if(data.begin() == it){
            q = data.front().second;
            return -1;
        }else if(data.end() == it){
            q = data.back().second;
            return 1;
        }else{
            // Slerp, Interpolation of quaternion.
            auto pit = it-1;
            double a = (time - pit->first).toSec()/(it->first - pit->first).toSec();
            q = pit->second.slerp(a,it->second);
            return 0;
        }
    }
}