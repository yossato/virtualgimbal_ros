#include "rotation.h"

namespace virtualgimbal
{

// Template specialization for Eigen::Quaterniond
template <>
int StampedDeque<Eigen::Quaterniond>::get(ros::Time time, Eigen::Quaterniond &q)
{
    constexpr bool verbose = false; // FOR DEBUG
    // std::cout << "Specialized" << std::endl;
    if(data.empty())
    {
        return DequeStatus::EMPTY;
    }
    auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, Eigen::Quaterniond> x) { return time < x.first; });
    if (data.begin() == it)
    {
        q = data.front().second;

        if (verbose)
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

        return DequeStatus::TIME_STAMP_IS_EARLIER_THAN_FRONT;
    }
    else if (data.end() == it)
    {
        q = data.back().second;

        if (verbose)
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

        return DequeStatus::TIME_STAMP_IS_LATER_THAN_BACK;
    }
    else
    {
        // Slerp, Interpolation of quaternion.
        auto pit = it - 1;
        double a = (time - pit->first).toSec() / (it->first - pit->first).toSec();
        q = pit->second.slerp(a, it->second);
        return DequeStatus::GOOD;
    }
};
template <>
Eigen::Vector3d StampedDeque<Eigen::Vector3d>::get(ros::Time time)
{
    Eigen::Vector3d v;
    if(data.empty())
    {
        throw;
    }
    auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, Eigen::Vector3d> x) { return time < x.first; });
    if (data.end() == it)
    {
        throw;
    }
    else
    {
        // Slerp, Interpolation of quaternion.
        auto pit = it - 1;
        double a = (time - pit->first).toSec() / (it->first - pit->first).toSec();
        return it->second * a + pit->second * (1.0-a);
    }
};
} // namespace virtualgimbal