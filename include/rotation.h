#ifndef __VIRTUALGIMAL_ROS_ROTATION_H_
#define __VIRTUALGIMAL_ROS_ROTATION_H_

#include <deque>
#include "ros/ros.h"
#include <Eigen/Dense>
#include "least_squares_method.h"
namespace virtualgimbal
{

     enum DequeStatus
    {
        GOOD = 0,
        TIME_STAMP_IS_EARLIER_THAN_FRONT,
        TIME_STAMP_IS_LATER_THAN_BACK,
        EMPTY

    };


template <class T>
class StampedDeque
{
public:


    StampedDeque(){};

    void clear()
    {
        data.clear();
    }

    void push_back(ros::Time time, T &q)
    {
        data.emplace_back(time, q);
    };

    void pop_old_close(ros::Time time)
    {
        auto it = std::find_if(data.begin(),data.end(),[&](std::pair<ros::Time,T> x){return time == x.first;});
        ++it;
        data.erase(it);
    }

    void pop_old(ros::Time time)
    {
        // std::cout << "Input time:" << time << std::endl;
        // std::cout << "Before" << std::endl;
        // for(auto el:data)
        // {
        //     std::cout << el.first << std::endl;
        // }
        auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, T> x) { return time <= x.first; });

        if(data.end() == it) return;
        // if(data.begin() == it) return;

        // To linear interpolate data, a previous data is required, so decrease iterator.
        
        --it;
        // TODO:先頭を消せるようにする。ここ変。
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

    std::pair<ros::Time, T> &back()
    {
        return data.back();
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

    bool is_available_after(ros::Time time)
    {
        if(data.empty())
        {
            return false;
        }
        auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, T> x) { return time <= x.first; });
        // if (data.begin() == it)
        // {
        //     return false;
        // }
        // else 
        if (data.end() == it)   // Not found
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    size_t count(ros::Time time)
    {
        return data.count(time);
    }

    T get(ros::Time time)
    {
        
        if(data.empty())
        {
            throw;
        }

        auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, T> x) { return time <= x.first; });
        if (data.end() == it)   // Not found
        {
            throw;
        }
        else
        {
            return it->second;
        }
    }

    T get(ros::Time request_time, ros::Time &actual_time)
    {
        
        if(data.empty())
        {
            throw;
        }

        auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, T> x) { return request_time <= x.first; });
        if (data.end() == it)   // Not found
        {
            throw;
        }
        else
        {
            actual_time = it->first;
            return it->second;
        }
    }

    int get(ros::Time time, T &q)
    {
        int retval = 0;

        if(data.empty())
        {
            return DequeStatus::EMPTY;
        }

        auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, T> x) { return time <= x.first; });
        /*if (data.begin() == it)
        {
            // TODO: 最初と次の間の時間を指定された時に変な動作する？
            q = data.front().second;

            return DequeStatus::TIME_STAMP_IS_EARLIER_THAN_FRONT;
        }
        else */if (data.end() == it)   // Not found
        {
            q = T();
            // std::cerr << "Failed to get()." << std::endl;
            return DequeStatus::TIME_STAMP_IS_LATER_THAN_BACK;
        }
        else
        {
            q = it->second;
            return DequeStatus::GOOD;
        }
    };

    void print_least_squares_method();
    // {
    //             std::cout << "Error" << std::endl;
    // }

    void print_all()
    {
        for(auto &el:data){
            std::cout << el.first << std::endl;//<< ":" << el.second.coeffs().transpose() << std::endl;
        }
    }

private:
    std::deque<std::pair<ros::Time, T>> data;
};





} // namespace virtualgimbal

#endif //__VIRTUALGIMAL_ROS_ROTATION_H_