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

    void pop_old(ros::Time time)
    {
        // std::cout << "Input time:" << time << std::endl;
        // std::cout << "Before" << std::endl;
        // for(auto el:data)
        // {
        //     std::cout << el.first << std::endl;
        // }
        auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, T> x) { return time < x.first; });

        if(data.end() == it)
        {
            std::cerr << "Warning: Failed to pop_old()" << std::endl;
        }

        // To linear interpolate data, a previous data is required, so decrease iterator.
        --it;

        if (data.begin() != it)
        {
            data.erase(data.begin(), it);
        }

        // std::cout << "After" << std::endl;
        // for(auto el:data)
        // {
        //     std::cout << el.first << std::endl;
        // }
    };

    void pop_front()
    {
       
        data.pop_front();
    };

    std::pair<ros::Time, T> &front()
    {
        return data.front();
    }

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