/* 
 * Software License Agreement (BSD 3-Clause License)
 * 
 * Copyright (c) 2020, Yoshiaki Sato
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StampedDeque(bool verbose = false) : verbose(verbose) 
    {};

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
        auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, T> x) { return time <= x.first; });

        if(data.end() == it) return;

        if (data.begin() != it)
        {
            data.erase(data.begin(), it);
        }
        else
        {
            data.pop_front();
        }

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
        if (data.end() == it)
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
        if (data.end() == it)
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
        if (data.end() == it)
        {
            throw;
        }
        else
        {
            actual_time = it->first;
            return it->second;
        }
    }

    T get_interpolate(ros::Time time);


    void print_least_squares_method(const ros::Time &begin, const ros::Time &end);
    Eigen::Quaterniond get_correction_quaternion_using_least_squares_method(const ros::Time &begin, const ros::Time &end, ros::Time &times, int order);
    
    void print_all()
    {
        for(auto &el:data){
            std::cout << el.first << std::endl;
        }
    }

private:
    std::deque<std::pair<ros::Time, T>> data;
    bool verbose;
};





} // namespace virtualgimbal

#endif //__VIRTUALGIMAL_ROS_ROTATION_H_