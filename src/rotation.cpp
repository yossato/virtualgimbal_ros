#include "rotation.h"

namespace virtualgimbal
{

// Template specialization for Eigen::Quaterniond
template <>
int StampedDeque<Eigen::Quaterniond>::get(ros::Time time, Eigen::Quaterniond &q)
{
    std::cout << "Specialized" << std::endl;
    auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, Eigen::Quaterniond> x) { return time < x.first; });
    if (data.begin() == it)
    {
        q = data.front().second;

        if (1)
        {
            ROS_INFO("Input time : %d.%d", time.sec, time.nsec);
            ROS_INFO("Size of data:%lu", data.size());

            ROS_INFO("Data time begin:  %d.%d", data.front().first.sec, data.front().first.nsec);
            ROS_INFO("Data time end: %d.%d", data.back().first.sec, data.back().first.nsec);
            for (int i = 0; i < data.size(); ++i)
            {
                printf("%d:%d.%d\r\n", i, data[i].first.sec, data[i].first.nsec);
            }
        }

        return -1;
    }
    else if (data.end() == it)
    {
        q = data.back().second;

        if (1)
        {
            ROS_INFO("Input time : %d.%d", time.sec, time.nsec);
            ROS_INFO("Size of data:%lu", data.size());

            ROS_INFO("Data time begin:  %d.%d", data.front().first.sec, data.back().first.nsec);
            ROS_INFO("Data time end: %d.%d", data.front().first.sec, data.back().first.nsec);
            for (int i = 0; i < data.size(); ++i)
            {
                printf("%d:%d.%d\r\n", i, data[i].first.sec, data[i].first.nsec);
            }
        }

        return 1;
    }
    else
    {
        // Slerp, Interpolation of quaternion.
        auto pit = it - 1;
        double a = (time - pit->first).toSec() / (it->first - pit->first).toSec();
        q = pit->second.slerp(a, it->second);
        return 0;
    }
};

//    template<class T> StampedDeque<T>::StampedDeque()

// template<class T> void StampedDeque<T>::push_back(ros::Time time, T &q){
//     data.emplace_back(time,q);
// }

// template<class T> void StampedDeque<T>::pop_front(ros::Time time){
//     auto it = std::find_if(data.begin(),data.end(),[&](std::pair<ros::Time,T > x){return time < x.first;});
//     if(data.begin() != it){
//         data.erase(data.begin(),it);
//     }
// }

// template<class T> void StampedDeque<T>::limit_data_length(int length){
//     if(data.size() > length)
//     {
//         for(int i=0,e=data.size()-length;i<e;++i){
//             data.pop_front();
//         }
//     }
// }

// template<class T> size_t StampedDeque<T>::size(){
//     return data.size();
// }

// template<class T> int StampedDeque<T>::get(ros::Time time, T &q){
//     auto it = std::find_if(data.begin(),data.end(),[&](std::pair<ros::Time,T > x){return time < x.first;});
//     if(data.begin() == it){
//         q = data.front().second;

//         if(1)
//         {
//             ROS_INFO("Input time : %d.%d",time.sec,time.nsec);
//             ROS_INFO("Size of data:%lu",data.size());

//             ROS_INFO("Data time begin:  %d.%d",data.front().first.sec,data.front().first.nsec);
//             ROS_INFO("Data time end: %d.%d",data.back().first.sec,data.back().first.nsec);
//             for(int i=0;i<data.size();++i){
//                 printf("%d:%d.%d\r\n",i,data[i].first.sec,data[i].first.nsec);
//             }
//         }

//         return -1;
//     }else if(data.end() == it){
//         q = data.back().second;

//         if(1)
//         {
//             ROS_INFO("Input time : %d.%d",time.sec,time.nsec);
//             ROS_INFO("Size of data:%lu",data.size());

//             ROS_INFO("Data time begin:  %d.%d",data.front().first.sec,data.back().first.nsec);
//             ROS_INFO("Data time end: %d.%d",data.front().first.sec,data.back().first.nsec);
//             for(int i=0;i<data.size();++i){
//                 printf("%d:%d.%d\r\n",i,data[i].first.sec,data[i].first.nsec);
//             }
//         }

//         return 1;
//     }else{
//         // Slerp, Interpolation of quaternion.
//         auto pit = it-1;
//         double a = (time - pit->first).toSec()/(it->first - pit->first).toSec();
//         q = pit->second.slerp(a,it->second);
//         return 0;
//     }
// }
} // namespace virtualgimbal