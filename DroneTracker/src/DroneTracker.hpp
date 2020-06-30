/*
 * RealSenseTest.hpp
 *
 *  Created on: Sep 28, 2018
 *      Author: sdc01
 */

#ifndef DRONE_TRACKER_HPP_
#define DRONE_TRACKER_HPP_

#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

#define DETECTION		0
#define TRACKING		1
#define DUAL_TRACKING	2

extern volatile bool running;
extern volatile bool tracking;
extern volatile bool frame_cap;
extern volatile bool frame_proc;
extern volatile int mode;

// Declare RealSense pipeline, encapsulating the actual device and sensors
extern Mat depth_image;
extern Mat color_image;
extern Mat frame_bin;

extern int dr_b_min, dr_b_max;
extern int dr_g_min, dr_g_max;
extern int dr_r_min, dr_r_max;
extern int l1_b_min, l1_b_max;
extern int l1_g_min, l1_g_max;
extern int l1_r_min, l1_r_max;
extern int l2_b_min, l2_b_max;
extern int l2_g_min, l2_g_max;
extern int l2_r_min, l2_r_max;
extern int d_min, d_max;
int ModeChange(int mode);
extern volatile int selected;

// mouse clicked parameters -> from console
struct mouse_struct {
	uint8_t  clicked;	// 1 if left clicked
						// 2 if right clicked
						// 3 if center clicked
						// 0 otherwise
	Point pos;			// click position
};



#endif /* DRONE_TRACKER_HPP_ */
