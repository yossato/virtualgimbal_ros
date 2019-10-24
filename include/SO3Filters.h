#ifndef __SO3FILTERS__H__
#define __SO3FILTERS__H__

#include <stdio.h>
#include <Eigen/Dense>
#include <vector>
// #include <rotation_param.h>
#include <camera_information.h>
#include <memory>
// Eigen::MatrixXd getFilterCoefficients

// void gradientLimit(Eigen::VectorXd &input, double maximum_gradient_);

using MatrixPtr = std::shared_ptr<std::vector<float>>;

bool isGoodWarp(std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> &contour, CameraInformationPtr camera_info);
std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> getSparseContour(CameraInformationPtr camera_info, int n);
void getUndistortUnrollingContour(
    // int frame,
    MatrixPtr R,
    std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> &contour,
    double zoom,
    CameraInformationPtr camera_info);
// Eigen::VectorXd getKaiserWindow(uint32_t tap_length, uint32_t alpha, bool swap);

bool hasBlackSpace(double zoom,
                   MatrixPtr R,
                   CameraInformationPtr camera_info);
uint32_t bisectionMethod(int frame,
                         double zoom,
                         MatrixPtr R,
                         CameraInformationPtr camera_info,
                         int32_t minimum_filter_strength,
                         int32_t maximum_filter_strength,
                         int max_iteration = 1000, uint32_t eps = 1);
// bool isGoodWarp(std::vector<Eigen::Array2d, Eigen::aligned_allocator<Eigen::Array2d>> &contour);
#endif //__SO3FILTERS__H__