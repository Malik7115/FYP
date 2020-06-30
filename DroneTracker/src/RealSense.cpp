/*
 * RealSense.cpp
 *
 *  Created on: Sep 30, 2018
 *      Author: sdc02
 */

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "RealSense.hpp"

rs2_intrinsics intrinsics;


// https://github.com/IntelRealSense/librealsense/blob/master/wrappers/opencv/cv-helpers.hpp
using namespace cv;
using namespace rs2;

// Convert rs2::frame to cv::Mat
Mat frame_to_mat(const rs2::frame& f)
{

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        cvtColor(r, r, COLOR_RGB2BGR);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f)
{
    Mat dm = frame_to_mat(f);
    dm.convertTo(dm, CV_64F);
    auto depth_scale = pipe.get_active_profile()
        .get_device()
        .first<depth_sensor>()
        .get_depth_scale();
    dm = dm * depth_scale;
    return dm;
}


// https://github.com/IntelRealSense/librealsense/blob/master/include/librealsense2/rsutil.h

/* Given a point in 3D space, compute the corresponding pixel coordinates in an image with no distortion or forward distortion coefficients produced by the same camera */
void rs2_project_point_to_pixel(float pixel[2], const struct rs2_intrinsics * intrin, const float point[3])
{
    //assert(intrin->model != RS2_DISTORTION_INVERSE_BROWN_CONRADY); // Cannot project to an inverse-distorted image

    float x = point[0] / point[2], y = point[1] / point[2];

    if(intrin->model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY)
    {

        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        x *= f;
        y *= f;
        float dx = x + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float dy = y + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = dx;
        y = dy;
    }
    if (intrin->model == RS2_DISTORTION_FTHETA)
    {
        float r = sqrtf(x*x + y*y);
        float rd = (float)(1.0f / intrin->coeffs[0] * atan(2 * r* tan(intrin->coeffs[0] / 2.0f)));
        x *= rd / r;
        y *= rd / r;
    }

    pixel[0] = x * intrin->fx + intrin->ppx;
    pixel[1] = y * intrin->fy + intrin->ppy;
}

/* Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera */
void rs2_deproject_pixel_to_point(float point[3], const struct rs2_intrinsics * intrin, const float pixel[2], float depth)
{
    assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
    assert(intrin->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
    //assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

    float x = (pixel[0] - intrin->ppx) / intrin->fx;
    float y = (pixel[1] - intrin->ppy) / intrin->fy;
    if(intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
    {
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        float ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }
    point[0] = depth * x;
    point[1] = depth * y;
    point[2] = depth;
}

/* Transform 3D coordinates relative to one sensor to 3D coordinates relative to another viewpoint */
void rs2_transform_point_to_point(float to_point[3], const struct rs2_extrinsics * extrin, const float from_point[3])
{
    to_point[0] = extrin->rotation[0] * from_point[0] + extrin->rotation[3] * from_point[1] + extrin->rotation[6] * from_point[2] + extrin->translation[0];
    to_point[1] = extrin->rotation[1] * from_point[0] + extrin->rotation[4] * from_point[1] + extrin->rotation[7] * from_point[2] + extrin->translation[1];
    to_point[2] = extrin->rotation[2] * from_point[0] + extrin->rotation[5] * from_point[1] + extrin->rotation[8] * from_point[2] + extrin->translation[2];
}

/* Calculate horizontal and vertical feild of view, based on video intrinsics */
void rs2_fov(const struct rs2_intrinsics * intrin, float to_fov[2])
{
    to_fov[0] = (atan2f(intrin->ppx + 0.5f, intrin->fx) + atan2f(intrin->width - (intrin->ppx + 0.5f), intrin->fx)) * 57.2957795f;
    to_fov[1] = (atan2f(intrin->ppy + 0.5f, intrin->fy) + atan2f(intrin->height - (intrin->ppy + 0.5f), intrin->fy)) * 57.2957795f;
}

void next_pixel_in_line(float curr[2], const float start[2], const float end[2])
{
    float line_slope = (end[1] - start[1]) / (end[0] - start[0]);
    if (abs(end[0] - curr[0]) > abs(end[1] - curr[1]))
    {
        curr[0] = end[0] > curr[0] ? curr[0] + 1 : curr[0] - 1;
        curr[1] = end[1] - line_slope * (end[0] - curr[0]);
    }
    else
    {
        curr[1] = end[1] > curr[1] ? curr[1] + 1 : curr[1] - 1;
        curr[0] = end[0] - ((end[1] + curr[1]) / line_slope);
    }
}

bool is_pixel_in_line(const float curr[2], const float start[2], const float end[2])
{
    return ((end[0] >= start[0] && end[0] >= curr[0] && curr[0] >= start[0]) || (end[0] <= start[0] && end[0] <= curr[0] && curr[0] <= start[0])) &&
           ((end[1] >= start[1] && end[1] >= curr[1] && curr[1] >= start[1]) || (end[1] <= start[1] && end[1] <= curr[1] && curr[1] <= start[1]));
}

void adjust_2D_point_to_boundary(float p[2], int width, int height)
{
    if (p[0] < 0) p[0] = 0;
    if (p[0] > width) p[0] = width;
    if (p[1] < 0) p[1] = 0;
    if (p[1] > height) p[1] = height;
}

