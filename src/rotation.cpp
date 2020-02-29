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

#include "rotation.h"
#include "SO3Filters.h"

// For debug
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <opencv2/imgproc.hpp>

namespace virtualgimbal
{

// Template specialization for Eigen::Quaterniond
template <>
Eigen::Quaterniond StampedDeque<Eigen::Quaterniond>::get_interpolate(ros::Time time)
{
    if(data.empty())
    {
        throw;
    }
    auto it = std::find_if(data.begin(), data.end(), [&](std::pair<ros::Time, Eigen::Quaterniond> x) { return time < x.first; });
    if (data.begin() == it)
    {
        throw;
    }
    else if (data.end() == it)
    {
        throw;
    }
    else
    {
        // Slerp, Interpolation of quaternion.
        auto pit = it - 1;
        double a = (time - pit->first).toSec() / (it->first - pit->first).toSec();
        return pit->second.slerp(a, it->second);
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

    template <>
    void StampedDeque<Eigen::Quaternion<double>>::print_least_squares_method(const ros::Time &begin, const ros::Time &end)
    {
        // beginを探す
        // endを探す
        // beginとendの間で最小二乗法
        // beginとendの間にデータが存在することは他の関数で保証させる
        auto begin_el   = std::find_if(data.begin(),data.end(),[&begin](std::pair<ros::Time, Eigen::Quaterniond> x) { return begin < x.first; });
        auto end_el     = std::find_if(data.begin(),data.end(),[&end](std::pair<ros::Time, Eigen::Quaterniond> x) { return end < x.first; });
        int num = std::distance(begin_el,end_el);

        Eigen::VectorXd time(num);
        Eigen::VectorXd angle(num);
        for(int i=0;i < num;++i)
        {
            time(i)  = ((begin_el+i)->first - (end_el)->first).toSec();
            angle(i) = Quaternion2Vector((begin_el+i)->second * end_el->second.conjugate())[0];
        }
        Eigen::VectorXd coeff = least_squares_method(time,angle,2);
        printf("Coeff:%f %f %f\r\n",coeff(0),coeff(1),coeff(2));
        for(int i=0;i<num;++i)
        {
            std::cout << time(i) << "," << angle(i) << "," << coeff(0) + time(i) * coeff(1) + pow(time(i),2.0) * coeff(2) << std::endl;
        }
        std::cout << std::flush;
    }

    template <>
    Eigen::Quaterniond StampedDeque<Eigen::Quaternion<double>>::get_correction_quaternion_using_least_squares_method(const ros::Time &begin, const ros::Time &end, ros::Time &target,int order)
    {
        auto begin_el   = std::find_if(data.begin(),data.end(),[&begin](std::pair<ros::Time, Eigen::Quaterniond> x) { return begin < x.first; });
        Eigen::Quaterniond target_angle = get_interpolate(target);
        auto end_el     = std::find_if(data.begin(),data.end(),[&end](std::pair<ros::Time, Eigen::Quaterniond> x) { return end < x.first; });
        
        ros::Time standard_time = target;
        int num = std::distance(begin_el,end_el);

        if(1 >= num)
        {
            return Eigen::Quaterniond(1.0,0,0,0);
        }

        Eigen::Quaterniond origin = target_angle;

        

        Eigen::VectorXd time(num);
        Eigen::VectorXd angle_x(num);
        Eigen::VectorXd angle_y(num);
        Eigen::VectorXd angle_z(num);

        

        for(int i=0;i < num;++i)
        {
            time(i)  = ((begin_el+i)->first - standard_time).toSec();
            Eigen::VectorXd vec = Quaternion2Vector(((begin_el+i)->second * origin.conjugate()).normalized());
            angle_x(i) = vec[0];
            angle_y(i) = vec[1];
            angle_z(i) = vec[2];
        }
        Eigen::VectorXd coeff_x = least_squares_method(time,angle_x,order);
        Eigen::VectorXd coeff_y = least_squares_method(time,angle_y,order);
        Eigen::VectorXd coeff_z = least_squares_method(time,angle_z,order);
        
        Eigen::Quaterniond lsm_value;
        double diff_t = (target - standard_time).toSec();
        Eigen::Vector3d lsm_vec = Eigen::Vector3d(coeff_x(order),coeff_y(order),coeff_z(order));
        for(int n= order - 1; n>=0; --n)//★ここ
        {
            lsm_vec = lsm_vec * diff_t + Eigen::Vector3d(coeff_x(n),coeff_y(n),coeff_z(n));
        }
        lsm_value = Vector2Quaternion<double>(lsm_vec) * origin;

        if(verbose){
            int width = 320;
            int height = 240;
            cv::Mat plot_result_x = cv::Mat::zeros(height,width,CV_8UC3);//,plot_result_y,plot_result_z;
            double gain = 250;
            for(int i=0;i<width;++i)
            {
                
                int index = (double)i/width*(angle_x.size()-1);
                double value = angle_x(index) *gain + height/2;
                int r = std::min(height-1,std::max(0,(int)value));
                plot_result_x.at<cv::Vec3b>(r,i) = cv::Vec3b(0,0,255);


                value = angle_y(index) *gain + height/2;
                r = std::min(height-1,std::max(0,(int)value));
                plot_result_x.at<cv::Vec3b>(r,i) = cv::Vec3b(0,255,0);

                value = angle_z(index) *gain + height/2;
                r = std::min(height-1,std::max(0,(int)value));
                plot_result_x.at<cv::Vec3b>(r,i) = cv::Vec3b(255,0,0);

                Eigen::Vector3d vec = Eigen::Vector3d(coeff_x(order),coeff_y(order),coeff_z(coeff_z.size()-1));
                for(int n= order - 1; n>=0; --n)
                {
                    vec = vec * time(index) + Eigen::Vector3d(coeff_x(n),coeff_y(n),coeff_z(n));
                }

                value = vec[0] *gain + height/2;
                r = std::min(height-1,std::max(0,(int)value));
                plot_result_x.at<cv::Vec3b>(r,i) = cv::Vec3b(127,127,255);

                value = vec[1] *gain + height/2;
                r = std::min(height-1,std::max(0,(int)value));
                plot_result_x.at<cv::Vec3b>(r,i) = cv::Vec3b(127,255,127);

                value = vec[2] *gain + height/2;
                r = std::min(height-1,std::max(0,(int)value));
                plot_result_x.at<cv::Vec3b>(r,i) = cv::Vec3b(255,127,127);


                

            }

            int index = (target - begin).toSec()/(end - begin ).toSec()*(angle_x.size()-1);
            int i = (target - begin).toSec()/(end - begin ).toSec()*(width-1);
            

            Eigen::Vector3d vec = Eigen::Vector3d(  coeff_x(0) + time(index) * coeff_x(1) + pow(time(index),2.0) * coeff_x(2),
                                                                    coeff_y(0) + time(index) * coeff_y(1) + pow(time(index),2.0) * coeff_y(2),
                                                                    coeff_z(0) + time(index) * coeff_z(1) + pow(time(index),2.0) * coeff_z(2));
            double value = vec[0] *gain + height/2;
            int r = std::min(height-1,std::max(0,(int)value));
            value = angle_x(index) *gain + height/2;
            int r2 = std::min(height-1,std::max(0,(int)value));
            for(int row = std::min(r,r2);row<std::max(r,r2);++row)
            {
                plot_result_x.at<cv::Vec3b>(row,i)[2] = std::min(plot_result_x.at<cv::Vec3b>(row,i)[2]+255,255);
            }

            value = vec[1] *gain + height/2;
            r = std::min(height-1,std::max(0,(int)value));
            value = angle_y(index) *gain + height/2;
            r2 = std::min(height-1,std::max(0,(int)value));
            for(int row = std::min(r,r2);row<std::max(r,r2);++row)
            {
                plot_result_x.at<cv::Vec3b>(row,i)[1] = std::min(plot_result_x.at<cv::Vec3b>(row,i)[1]+255,255);
            }

            value = vec[2] *gain + height/2;
            r = std::min(height-1,std::max(0,(int)value));
            value = angle_z(index) *gain + height/2;
            r2 = std::min(height-1,std::max(0,(int)value));
            for(int row = std::min(r,r2);row<std::max(r,r2);++row)
            {
                plot_result_x.at<cv::Vec3b>(row,i)[0] = std::min(plot_result_x.at<cv::Vec3b>(row,i)[0]+255,255);
            }

            cv::imshow("plot",plot_result_x);
            

        }


        return lsm_value.normalized();
    }

} // namespace virtualgimbal