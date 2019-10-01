#ifndef __VIRTUALGIMAL_ROS_ROTATION_H_
#define __VIRTUALGIMAL_ROS_ROTATION_H_

#include <deque>
#include "ros/ros.h"
#include <Eigen/Dense>
namespace virtualgimbal
{

template <class T>
class StampedDeque
{
public:
   
    StampedDeque(){};

    void push_back(ros::Time time, T &q)
    {
        data.emplace_back(time, q);
    };

    void pop_front(ros::Time time)
    {
        auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, T> x) { return time < x.first; });
        if (data.begin() != it)
        {
            data.erase(data.begin(), it);
        }
    };

    size_t size()
    {
        return data.size();
    };

    void limit_data_length(int length)
    {
        if (data.size() > length)
        {
            for (int i = 0, e = data.size() - length; i < e; ++i)
            {
                data.pop_front();
            }
        }
    };
    int get(ros::Time time, T &q)
    {
        int retval = 0;

        auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, T> x) { return time == x.first; });
        if (data.end() == it)   // Not found
        {
            q = T();
            std::cerr << "Failed to get()." << std::endl;
            return 1;
        }
        else
        {
            q = it->second;
            return 0;
        }
    };

private:
    std::deque<std::pair<ros::Time, T>> data;
};





} // namespace virtualgimbal

#endif //__VIRTUALGIMAL_ROS_ROTATION_H_