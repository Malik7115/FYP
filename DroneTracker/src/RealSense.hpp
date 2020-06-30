/*
 * RealSenseTest.hpp
 *
 *  Created on: Sep 28, 2018
 *      Author: sdc01
 */

#ifndef REALSENSE_HPP_
#define REALSENSE_HPP_

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

// ********************************************************
//                  Real Sense Functions
// ********************************************************
// Convert rs2::frame to cv::Mat
cv::Mat frame_to_mat(const rs2::frame& f);
// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f);
// Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera
void rs2_deproject_pixel_to_point(float point[3], const struct rs2_intrinsics * intrin, const float pixel[2], float depth);

extern rs2_intrinsics intrinsics;

#endif /* REALSENSE_HPP_ */
