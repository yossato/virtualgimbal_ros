/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.
                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)
Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.
This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/


#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <aruco_board.h>
using namespace std;
using namespace cv;

namespace {
const char* about = "Pose estimation using a ArUco Planar Grid board";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{l        |       | Marker side length (in pixels) }"
        "{s        |       | Separation between two consecutive markers in the grid (in pixels)}"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        "{r        |       | show rejected candidates too }";
}

ArucoRos::ArucoRos(ros::NodeHandle &pnh) : pnh_(pnh), detector_params_(aruco::DetectorParameters::create())
{
    bool success = 
    readDetectorParams() && 
    readMarkerParams();

    if(!success)
    {
        ROS_ERROR("Failed to get params");
        std::exit(EXIT_FAILURE);
    }
}

bool ArucoRos::readDetectorParams()
{ 
    bool retval = 
    pnh_.getParam("adaptiveThreshWinSizeMin",detector_params_->adaptiveThreshWinSizeMin) && 
    pnh_.getParam("adaptiveThreshWinSizeMax",detector_params_->adaptiveThreshWinSizeMax) && 
    pnh_.getParam("adaptiveThreshWinSizeStep",detector_params_->adaptiveThreshWinSizeStep) && 
    pnh_.getParam("adaptiveThreshConstant",detector_params_->adaptiveThreshConstant) && 
    pnh_.getParam("minMarkerPerimeterRate",detector_params_->minMarkerPerimeterRate) && 
    pnh_.getParam("maxMarkerPerimeterRate",detector_params_->maxMarkerPerimeterRate) &&
    pnh_.getParam("minCornerDistanceRate",detector_params_->minCornerDistanceRate) && 
    pnh_.getParam("minDistanceToBorder",detector_params_->minDistanceToBorder) && 
    pnh_.getParam("minMarkerDistanceRate",detector_params_->minMarkerDistanceRate) && 
    pnh_.getParam("cornerRefinementMethod",detector_params_->cornerRefinementMethod) && 
    pnh_.getParam("cornerRefinementWinSize",detector_params_->cornerRefinementWinSize) && 
    pnh_.getParam("cornerRefinementMaxIterations",detector_params_->cornerRefinementMaxIterations) && 
    pnh_.getParam("cornerRefinementMinAccuracy",detector_params_->cornerRefinementMinAccuracy) && 
    pnh_.getParam("markerBorderBits",detector_params_->markerBorderBits) && 
    pnh_.getParam("perspectiveRemovePixelPerCell",detector_params_->perspectiveRemovePixelPerCell) && 
    pnh_.getParam("perspectiveRemoveIgnoredMarginPerCell",detector_params_->perspectiveRemoveIgnoredMarginPerCell) && 
    pnh_.getParam("maxErroneousBitsInBorderRate",detector_params_->maxErroneousBitsInBorderRate) && 
    pnh_.getParam("minOtsuStdDev",detector_params_->minOtsuStdDev) && 
    pnh_.getParam("errorCorrectionRate",detector_params_->errorCorrectionRate);
    return retval;
}

bool ArucoRos::readMarkerParams()
{
    
    bool retval = 
    pnh_.getParam("markersX",markers_X_) && 
    pnh_.getParam("markersY",markers_Y_) && 
    pnh_.getParam("markerLength",marker_length_) && 
    pnh_.getParam("markerSeparation",marker_separation_) && 
    pnh_.getParam("dictionaryId",dictionary_id_) && 
    pnh_.getParam("showRejected",show_rejected_) && 
    pnh_.getParam("refindStrategy",refind_strategy_);
    return retval;
}



/**
 */
bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}



/**
 */
bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
static void _getSingleMarkerObjectPoints(float markerLength, OutputArray _objPoints) {

    CV_Assert(markerLength > 0);

    _objPoints.create(4, 1, CV_32FC3);
    Mat objPoints = _objPoints.getMat();
    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.ptr< Vec3f >(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr< Vec3f >(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
}


/**
  * ParallelLoopBody class for the parallelization of the single markers pose estimation
  * Called from function estimatePoseSingleMarkers()
  */
class SinglePoseEstimationParallel : public ParallelLoopBody {
    public:
    SinglePoseEstimationParallel(cv::Mat& _markerObjPoints, cv::InputArrayOfArrays _corners,
                                 cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                                 cv::Mat& _rvecs, cv::Mat& _tvecs)
        : markerObjPoints(_markerObjPoints), corners(_corners), cameraMatrix(_cameraMatrix),
          distCoeffs(_distCoeffs), rvecs(_rvecs), tvecs(_tvecs), use_extrinsic_guess(true) {}

    void operator()(const cv::Range &range) const CV_OVERRIDE {
        const int begin = range.start;
        const int end = range.end;
        cv::Vec3d dummy_rvec;
        for(int i = begin; i < end; i++) {
            cv::solvePnP(markerObjPoints, corners.getMat(i), cameraMatrix, distCoeffs,
                    dummy_rvec, tvecs.at<cv::Vec3d>(i));
            cv::solvePnP(markerObjPoints, corners.getMat(i), cameraMatrix, distCoeffs,
                    rvecs.at<cv::Vec3d>(i), tvecs.at<cv::Vec3d>(i), use_extrinsic_guess);
        }
    }

    private:
    SinglePoseEstimationParallel &operator=(const SinglePoseEstimationParallel &); // to quiet MSVC

    cv::Mat& markerObjPoints;
    cv::InputArrayOfArrays corners;
    cv::InputArray cameraMatrix, distCoeffs;
    cv::Mat& rvecs, tvecs;
    bool use_extrinsic_guess;
};




/**
  */
void estimatePoseSingleMarkersWithInitialPose(cv::InputArrayOfArrays _corners, float markerLength,
                               cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                               cv::OutputArray _rvecs, cv::OutputArray _tvecs, cv::OutputArray _objPoints) {

    CV_Assert(markerLength > 0);

    cv::Mat markerObjPoints;
    _getSingleMarkerObjectPoints(markerLength, markerObjPoints);
    int nMarkers = (int)_corners.total();
    _rvecs.create(nMarkers, 1, CV_64FC3);
    _tvecs.create(nMarkers, 1, CV_64FC3);

    cv::Mat rvecs = _rvecs.getMat(), tvecs = _tvecs.getMat();

    //// for each marker, calculate its pose
    // for (int i = 0; i < nMarkers; i++) {
    //    solvePnP(markerObjPoints, _corners.getMat(i), _cameraMatrix, _distCoeffs,
    //             _rvecs.getMat(i), _tvecs.getMat(i));
    //}

    // this is the parallel call for the previous commented loop (result is equivalent)
    parallel_for_(cv::Range(0, nMarkers),
                  SinglePoseEstimationParallel(markerObjPoints, _corners, _cameraMatrix,
                                               _distCoeffs, rvecs, tvecs));
    if(_objPoints.needed()){
        markerObjPoints.convertTo(_objPoints, -1);
    }
}

