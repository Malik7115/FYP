/*
 * VidProc.cpp
 *
 *  Created on: Sep 28, 2018
 *      Author: sdc01
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <pthread.h>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "DroneTracker.hpp"
#include "Tracker.hpp"
#include "ImProcessing.hpp"
#include "RealSense.hpp"
#include "VidProc.hpp"
#include "DroneControl.hpp"
#include "Thrust.hpp"



static void SingleTrack(void);
static void DualTrack(void);
static void SaveDroneProfile(void);
void SystemIdentification();

mutex vid_mutex;
condition_variable cond;
vector<obj_struct> obj_array; 	// Array to hold multiple detected objects
vector<obj_struct> led1_array;
vector<obj_struct> led2_array;

obj_struct obj_meas;			// Prime target to be tracked
obj_struct obj_est;				// Estimated target
obj_struct obj_pred;			// Predicted target

obj_struct obj_meas2;			// Second target to be tracked
obj_struct obj_est2;			// Estimated target
obj_struct obj_pred2;			// Predicted target

double dual_match;
double tgt_px, tgt_py, tgt_pz;
double tgt_yaw;


volatile bool frame_done;
volatile char drone_cmd;
volatile bool drone_cmd_flag;

int drone_mode;
int drone_si_count;

char VidProcString[100];
Mat display_image;
Mat led1_image;
Mat led2_image;
Mat channels[3];
Mat color_image2;

static mouse_struct mouse_data;
static volatile bool mouse_flag = false;
static mutex mouse_mutex;

drone_pos drone_profile[SYS_ID_COUNT];


void VidProc()
{
	static struct timespec end, start;

	vid_mutex.lock();
	cout << "Video Proc Thread ID: " << this_thread::get_id() << '\n';
	vid_mutex.unlock();

	while(running){
		{
			{
			unique_lock<mutex> lock(vid_mutex);
			cond.wait(lock, []{return frame_cap;});
			clock_gettime( CLOCK_REALTIME, &start );

			frame_cap = false;
			cvtColor(color_image, color_image2, COLOR_BGR2YCrCb);
			//color_image2 = color_image.clone();
			frame_proc = 0;
			}

			GaussianBlur(color_image2, color_image2, Size(3, 3), 0);

			// vid_mutex is automatically released when lock goes out of scope
			switch(mode){
			case DETECTION:
				color_image2 = DepthFilter(color_image2, depth_image, d_min/1000.0, d_max/1000.0);
				inRange(color_image2, Scalar(dr_b_min, dr_g_min, dr_r_min),
						Scalar(dr_b_max, dr_g_max, dr_r_max), frame_bin );
				cvtColor(frame_bin, display_image, COLOR_GRAY2BGR);

//				inRange(color_image2, Scalar(l1_b_min, l1_g_min, l1_r_min),
//						Scalar(l1_b_max, l1_g_max, l1_r_max), led1_image );
//				inRange(color_image2, Scalar(l2_b_min, l2_g_min, l2_r_min),
//						Scalar(l2_b_max, l2_g_max, l2_r_max), led2_image );
//			    split(display_image, channels);
//			    channels[0] = channels[0]&(~led1_image);
//			    channels[1] = channels[1]&(~led1_image);
//			    channels[2] = channels[2]|led1_image;
//			    channels[0] = channels[0]&(~led2_image);
//			    channels[1] = channels[1]|led2_image;
//			    channels[2] = channels[2]&(~led2_image);
//			    merge(channels, 3, display_image);

				ExtractObjects(frame_bin, depth_image, obj_array);

				if(mouse_flag) {
					mouse_flag = false;

					mouse_struct mouse;
					{
						lock_guard<mutex> lock(mouse_mutex);
						mouse = mouse_data;
						mouse_data.clicked = false;
					}
					if(mouse.clicked) {
						for(auto obj:obj_array){
							Rect r;
							r.x = obj.x;
							r.y = obj.y;
							r.width = obj.width;
							r.height = obj.height;
							if(r.contains(mouse.pos)){
								obj_meas 		= obj;
								obj_meas.index 	= 1;
								obj_est 		= obj_meas;
								obj_pred		= obj_meas;
								ModeChange(TRACKING);
							}
						}
					}
				}
				DroneActuate(HIGH, MID, MID, MID);
				break;
			case TRACKING:
				// Image processing
				color_image2 = DepthFilter(color_image2, depth_image, obj_pred.pz - 0.2, obj_pred.pz + 0.2);
				cvtColor(frame_bin, display_image, COLOR_GRAY2BGR);
				inRange(color_image2, Scalar(dr_b_min, dr_g_min, dr_r_min),
						Scalar(dr_b_max, dr_g_max, dr_r_max), frame_bin );

				ExtractObjects(frame_bin, depth_image, obj_array);

				// Track object
				SingleTrack();
				if(mouse_flag) {
					mouse_flag = false;

					mouse_struct mouse;
					{
						lock_guard<mutex> lock(mouse_mutex);
						mouse = mouse_data;
						mouse_data.clicked = false;
					}
					if(mouse.clicked) {
						for(auto obj:obj_array){
							Rect r;
							r.x = obj.x;
							r.y = obj.y;
							r.width = obj.width;
							r.height = obj.height;
							if(r.contains(mouse.pos)){
								obj_meas2 		= obj;
								obj_meas2.index 	= 1;
								obj_est2 		= obj_meas2;
								obj_pred2		= obj_meas2;
								ModeChange(DUAL_TRACKING);
							}
						}
					}
				}
				DroneActuate(HIGH, MID, MID, MID);
				break;
			case DUAL_TRACKING:
				// Image processing
				color_image2 = DepthFilter(color_image2, depth_image, obj_pred.pz - 0.2, obj_pred.pz + 0.2);
				cvtColor(frame_bin, display_image, COLOR_GRAY2BGR);
				inRange(color_image2, Scalar(dr_b_min, dr_g_min, dr_r_min),
						Scalar(dr_b_max, dr_g_max, dr_r_max), frame_bin );
				ExtractObjects(frame_bin, depth_image, obj_array);

				// Track two objects
				DualTrack();

				if(drone_mode == DRONE_SYS_IDENT){
					SystemIdentification();

				}
				else{
				}
			}
			clock_gettime( CLOCK_REALTIME, &end);
			double difference = (end.tv_sec - start.tv_sec) +
					(double)(end.tv_nsec - start.tv_nsec)/1000000000.0;
			sprintf(VidProcString, "Processing time: %4.1fms", difference * 1000);

		}
		frame_done = 1;
	}
}
static void SingleTrack(void)
{
	obj_meas = SearchObject(obj_pred);
	if(obj_meas.status < 0){ 					// Object not found
		obj_pred.miss_num++;
		if(obj_pred.miss_num > MAX_MISS_NUM){ 	// Track lost
			ModeChange(DETECTION);
		}
		else{
			obj_est   = obj_pred;
			obj_pred = PredObject(obj_pred);	// Open loop prediction (without measurement)
		}
	}
	else{
		obj_pred.miss_num = 0;
		obj_est = TrackObject(obj_meas, obj_pred);
		obj_pred = PredObject(obj_est);
	}

}

static void DualTrack(void)
{

	tie(obj_meas, obj_meas2, dual_match) = SearchPair(obj_pred, obj_pred2);

	if(obj_meas.status  == POOR_MATCH || obj_meas2.status == POOR_MATCH){
		obj_pred.miss_num++;
		obj_pred2.miss_num++;
		if(obj_pred.miss_num > MAX_MISS_NUM || obj_pred2.miss_num > MAX_MISS_NUM){ 	// Track lost
			tgt_px   = 0;
			tgt_py	 = 0;
			tgt_pz	 = 0;
			tgt_yaw  = 0;
			ModeChange(DETECTION);
		}
		else{
			tgt_px	  = (obj_pred.px + obj_pred2.px)/2;
			tgt_py	  = (obj_pred.py + obj_pred2.py)/2;
			tgt_pz	  = (obj_pred.pz + obj_pred2.pz)/2;
			tgt_yaw   = atan2(obj_meas.pz - obj_meas2.pz, obj_meas.px - obj_meas2.px);

			obj_est   = obj_pred;
			obj_est2  = obj_pred2;
			obj_pred  = PredObject(obj_pred);	// Open loop prediction (without measurement)
			obj_pred2 = PredObject(obj_pred2);	// Open loop prediction (without measurement)
		}
	}
	else{
		tgt_px		= (obj_meas.px + obj_meas2.px)/2;
		tgt_py		= (obj_meas.py + obj_meas2.py)/2;
		tgt_pz		= (obj_meas.pz + obj_meas2.pz)/2;
		tgt_yaw     = atan2(obj_meas.pz - obj_meas2.pz, obj_meas.px - obj_meas2.px);

		obj_pred.miss_num = 0;
		obj_est 	= TrackObject(obj_meas, obj_pred);
		obj_pred 	= PredObject(obj_est);
		obj_pred2.miss_num = 0;
		obj_est2 	= TrackObject(obj_meas2, obj_pred2);
		obj_pred2 	= PredObject(obj_est2);
		tgt_yaw     = atan2(obj_est.pz - obj_est2.pz, obj_est.px - obj_est2.px);
	}

}
void UpdateMouse(mouse_struct mouse)
{
	{
		lock_guard<mutex> lock(mouse_mutex);
		if(mouse_data.clicked == 0) {
			mouse_data = mouse;
			mouse_flag = true;
		}
	}

}
void VidProcCmd(char c)
{
	drone_cmd = c;
	drone_cmd_flag = true;
}


static void SaveDroneProfile(void)
{
	char s[100];
	ofstream outfile("MATLAB/drone_profile_data.m");

	outfile << "drone_profile = [";

	for(int i=0;i<SYS_ID_COUNT;i++){

		sprintf(s, "%f, %f, %f, %f; \n", drone_profile[i].px,
				drone_profile[i].py,drone_profile[i].pz, drone_profile[i].yaw);
		outfile << s;

	}
	outfile << "];";
	outfile.close();

}

void SystemIdentification()
{
	static int count;
	static double err_x, err_z;
	static double err_dx, err_dz;
	static double zr = 0.35;
	if(drone_cmd_flag){
		if(drone_si_count < SYS_ID_COUNT){
			if(drone_si_count == 0){
				thrust_observer.init(tgt_py);
				DroneActuate(MID + 1000, MID, MID, MID);
				count = 0;
			}
			if(count++){
//				if(count == 8)
//					count = 0;
				//return;
			}

			double delay = 0.0;
			double u;
			double x_hat;
			x_hat = x_pred.est;
			Mat y_hat;
			y_hat = thrust_observer.state();
			double z_hat;
			z_hat = z_pred.est;
			//cout << x_hat << endl;
			u = mu_to_uy(MID + 700);
			//u = thrust_observer.control();
			//cout << u << endl;

			thrust_pred.iterate(tgt_py);
			double y = thrust_pred.predict(delay);

			x_pred.iterate(tgt_px);
			double x = x_pred.predict(delay);
//			x = tgt_px;

			z_pred.iterate(tgt_pz - zr);
			double z = z_pred.predict(delay);

			double sign = -1;

			double ux = -100 * x ; //+ 200 * x_pred.est_dot + 20*x_pred.acc;
			double uz = -100 * z ; //- 100 * z_pred.est_dot - 50*z_pred.acc;

//			uz = 0;

#define CTRL_2D

			switch(count){
			case 1:
			case 2:
			case 3:
#ifdef CTRL_2D
				ux = 3000 * err_x; // + 2000 * err_dx;
				if(ux > 0 && fabs(err_x) > 0.01)
					ux += 1200;
				else if(ux < 0 && fabs(err_x) > 0.01)
					ux -= 1200;
#else
				ux = 1000 * err_x; // + 2000 * err_dx;
				if(ux > 0 && fabs(err_x) > 0.01){
					if(err_x > 0.1)
						ux += 2000;
					else
						ux += 200;
				}
				else if(ux < 0 && fabs(err_x) > 0.01){
					if(err_x < -0.1)
						ux -= 2000;
					else
						ux -= 300;
				}
#endif
				break;
			case 17:
				err_x = x;
				err_dx = x_pred.est_dot;
				break;
			case 10:
			case 11:
			case 12:
#ifdef CTRL_2D

				uz = -4000 * err_z;// - 2000 * err_dz;
				if(uz > 0 && fabs(err_z) > 0.01)  ///////////////////x-axis
					uz += 1200;
				else if(uz < 0 && fabs(err_z) > 0.01)
					uz -= 1200;
#else
				uz = -1000 * err_z;// - 2000 * err_dz;
				if(uz > 0 && fabs(err_z) > 0.01){
					if(err_z > 0.1)
						uz += 2000;
					else
						uz += 300;
				}
				else if(uz < 0 && fabs(err_z) > 0.01){
					if(err_z < -0.1)
						uz -= 2000;
					else
						uz -= 200;
				}

#endif
				break;
			case 8:
				err_z = z;
				err_dz = z_pred.est_dot;
				break;
			case 18:
				count = 0;
			default:
				ux = 0;
				uz = 0;
				break;
			}
//			ux = 0;
//			uz = 0;

//			if(uz>0)
//				uz += 800;
//			else if(uz<0)
//				uz -= 800;

			DroneActuate(uy_to_mu(u), MID, uz_to_mu(uz), ux_to_mu(ux));


			thrust_observer.iterate(u, y);
			thrust_array[drone_si_count][0] = x;
			//thrust_array[drone_si_count][0] = y;
			thrust_array[drone_si_count][1] = z + zr;
			thrust_array[drone_si_count][2] = ux;
			thrust_array[drone_si_count][3] = uz;

			drone_profile[drone_si_count].px  = tgt_px;
			drone_profile[drone_si_count].py  = tgt_py;
			drone_profile[drone_si_count].pz  = tgt_pz;
			drone_profile[drone_si_count].yaw = tgt_yaw;
			drone_si_count++;
		}
		else{
			DroneActuate(HIGH, MID, MID, MID);
			SaveDroneProfile();
			SaveThrustProfile(drone_si_count);
			drone_cmd_flag = 0;
			drone_si_count = 0;
			running = false;
		}

	}

}

