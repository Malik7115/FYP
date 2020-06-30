/*
 * VidProc.hpp
 *
 *  Created on: Sep 28, 2018
 *      Author: sdc01
 */

#ifndef VIDPROC_HPP_
#define VIDPROC_HPP_

#include <mutex>
#include <condition_variable>
#include <vector>
#include "ImProcessing.hpp"
using namespace std;

extern mutex vid_mutex;
extern condition_variable cond;
extern volatile bool frame_done;

void VidProc();
void UpdateMouse(mouse_struct mouse);
void VidProcCmd(char c);


extern vector<obj_struct> obj_array;
extern vector<obj_struct> led1_array;
extern vector<obj_struct> led2_array;

extern obj_struct obj_meas;			// Measured target
extern obj_struct obj_est;			// Estimated target
extern obj_struct obj_pred;			// Predicted target

extern obj_struct obj_meas2;		// Second target to be tracked
extern obj_struct obj_est2;			// Estimated target
extern obj_struct obj_pred2;		// Predicted target

extern double dual_match;
extern double tgt_px, tgt_py, tgt_pz;
extern double tgt_yaw;

extern int drone_mode;
#define DRONE_SYS_IDENT		0
#define DRONE_CONTROL		1



extern Mat display_image;

#endif /* VIDPROC_HPP_ */
