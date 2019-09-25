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

    void rotation::limit_data_length(int length){
        if(data.size() > length)
        {
            for(int i=0,e=data.size()-length;i<e;++i){
                data.pop_front();
            }
        }
    }

    size_t rotation::size(){
        return data.size();
    }

    int rotation::get(ros::Time time, Eigen::Quaterniond &q){
        auto it = std::find_if(data.begin(),data.end(),[&](std::pair<ros::Time,Eigen::Quaterniond > x){return time < x.first;});
        if(data.begin() == it){
            q = data.front().second;

            if(1)
            {
                ROS_INFO("Input time : %d.%d",time.sec,time.nsec);
                ROS_INFO("Size of data:%lu",data.size());
                
                ROS_INFO("Data time begin:  %d.%d",data.front().first.sec,data.front().first.nsec);
                ROS_INFO("Data time end: %d.%d",data.back().first.sec,data.back().first.nsec);
                for(int i=0;i<data.size();++i){
                    printf("%d:%d.%d\r\n",i,data[i].first.sec,data[i].first.nsec);
                }
            }

            return -1;
        }else if(data.end() == it){
            q = data.back().second;

            if(1)
            {
                ROS_INFO("Input time : %d.%d",time.sec,time.nsec);
                ROS_INFO("Size of data:%lu",data.size());
                
                ROS_INFO("Data time begin:  %d.%d",data.front().first.sec,data.back().first.nsec);
                ROS_INFO("Data time end: %d.%d",data.front().first.sec,data.back().first.nsec);
                for(int i=0;i<data.size();++i){
                    printf("%d:%d.%d\r\n",i,data[i].first.sec,data[i].first.nsec);
                }
            }

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