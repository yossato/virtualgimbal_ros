/*************************************************************************
*  Software License Agreement (BSD 3-Clause License)
*  
*  Copyright (c) 2019, Yoshiaki Sato
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
*  
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*  
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/
#include "angular_velocity_estimator.h"
int estimate_angular_velocity(cv::Mat &cur, cv::Mat &prev, Eigen::MatrixXd &optical_flow)
{
    optical_flow = Eigen::MatrixXd::Zero(1, 3);

    cv::Mat cur_grey;
    cv::Mat prev_grey;

    int width = prev.cols;  //cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = prev.rows; //cap.get(cv::CAP_PROP_FRAME_HEIGHT);


    // Color conversion
    if (prev.type() == CV_8UC3)
    {
        cv::cvtColor(prev, prev_grey, cv::COLOR_BGR2GRAY);
    }
    else
    {
        prev_grey = prev;
    }
    
    // Trimming
    if ((width >= 640) && (height >= 480))
    {
        prev_grey = prev_grey(cv::Rect((width - 640) / 2, (height - 480) / 2, 640, 480));
    }

    // Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames

    if (prev.type() == CV_8UC3) // TODO:もしもモノクロ画像やデプス画像だった時の対応が必要！！！！！！！！！
    {
        cv::cvtColor(cur, cur_grey, cv::COLOR_BGR2GRAY);
    }else
    {
        cur_grey = cur;
    }

    // Trimming
    if ((width >= 640) && (height >= 480))
    {
            cur_grey = cur_grey(cv::Rect((cur_grey.cols-640)/2,(cur_grey.rows-480)/2,640,480));
    }

    // vector from prev to cur
    std::vector<cv::Point2f> prev_corner, cur_corner;
    std::vector<cv::Point2f> prev_corner2, cur_corner2;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
    cv::calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);

    // weed out bad matches
    for (size_t i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            prev_corner2.push_back(prev_corner[i]);
            cur_corner2.push_back(cur_corner[i]);
        }
    }

    // translation + rotation only
    try
    {
        cv::Mat T = cv::estimateAffinePartial2D(prev_corner2, cur_corner2);

        // in rare cases no transform is found. We'll just use the last known good transform.
        if (T.data == NULL)
        {
            return 1;
        }
        else
        {
            double dx = T.at<double>(0, 2);
            double dy = T.at<double>(1, 2);
            double da = atan2(T.at<double>(1, 0), T.at<double>(0, 0));
            optical_flow << dx, dy, da;
            return 0;
        }
    }
    catch (...)
    {
        return 1;
    }
}