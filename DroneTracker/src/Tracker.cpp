/*
 * Tracker.cpp
 *
 *  Created on: Sep 1, 2018
 *      Author: bilal
 */

#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include "opencv2/opencv.hpp"
#include "ImProcessing.hpp"
#include "Tracker.hpp"
#include "DroneTracker.hpp"

using namespace std;
using namespace cv;

// A complete tracker for all dynamic parameters
//
// tgt_meas is the new measurement
// tgt_meas only carries spatial information
//
// tgt_est is the estimated value at current time
// tgt_est is in fact second output of the tracker
//
// tgt_pred is the predicted information from last time


obj_struct TrackObject(const obj_struct& tgt_meas, const obj_struct& tgt_pred)
{
	double K, K_thresh;
 	double K1, K2;
 	double lambda = 0.7; // Forgetting factor
 	obj_struct tgt_est = tgt_pred;


 	switch(tgt_pred.index){
 	case 0:			// Initialization only
		tgt_est = tgt_meas;
		return tgt_est;

		break;
 	case 1:			// Start tracking
		K  = 1;
		K1 = 1;
		K2 = 1;
		K_thresh = 0;
		break;
 	case 2:			// High initial gain
 	case 3:
 	case 4:
 	case 5:
 		lambda = 0.5;
		K  = 1-lambda;
		K1 = 1-lambda*lambda;
		K2 = (1-lambda)*(1-lambda);
		K_thresh = 0.5;
		break;
 	default:		// Continue tracking
 		lambda = 0.8;
		K  = 1-lambda;
		K1 = 1-lambda*lambda;
		K2 = (1-lambda)*(1-lambda);
		K_thresh = 0.3;
	}

 	// Prediction errors
 	double pred_cx_err 		= tgt_meas.cx - tgt_pred.cx;
 	double pred_cy_err 		= tgt_meas.cy - tgt_pred.cy;

 	double pred_px_err 		= tgt_meas.px - tgt_pred.px;
 	double pred_py_err 		= tgt_meas.py - tgt_pred.py;
 	double pred_pz_err 		= tgt_meas.pz - tgt_pred.pz;

 	double pred_area_err 	= tgt_meas.area - tgt_pred.area;
 	double pred_aspect_err	= tgt_meas.aspect - tgt_pred.aspect;
 	double pred_thresh_err	= tgt_meas.thresh - tgt_pred.thresh;


	// Estimation -> prediction + correction

	tgt_est.cx 		= tgt_pred.cx 		+ K1 * pred_cx_err;
	tgt_est.cvx 	= tgt_pred.cvx 		+ K2 * pred_cx_err;
	tgt_est.cy 		= tgt_pred.cy 		+ K1 * pred_cy_err;
	tgt_est.cvy 	= tgt_pred.cvy 		+ K2 * pred_cy_err;

	tgt_est.px 		= tgt_pred.px 		+ K1 * pred_px_err;
	tgt_est.vx 		= tgt_pred.vx 		+ K2 * pred_px_err;
	tgt_est.py 		= tgt_pred.py 		+ K1 * pred_py_err;
	tgt_est.vy 		= tgt_pred.vy 		+ K2 * pred_py_err;
	tgt_est.pz 		= tgt_pred.pz 		+ K1 * pred_pz_err;
	tgt_est.vz 		= tgt_pred.vz 		+ K2 * pred_pz_err;

	tgt_est.area 	= tgt_pred.area 	+ K * pred_area_err;
	tgt_est.aspect 	= tgt_pred.aspect 	+ K * pred_aspect_err;
	tgt_est.thresh 	= tgt_pred.thresh 	+ K_thresh * pred_thresh_err;
	if(tgt_est.thresh > max_thresh) tgt_est.thresh = max_thresh;
	if(tgt_est.thresh < min_thresh) tgt_est.thresh = min_thresh;

	tgt_est.x 		= tgt_meas.x - (tgt_meas.cx - tgt_est.cx); // Required for BBox only
	tgt_est.y 		= tgt_meas.y - (tgt_meas.cy - tgt_est.cy);
	tgt_est.width	= tgt_meas.width;
	tgt_est.height	= tgt_meas.height;



	return tgt_est;
}

obj_struct PredObject(const obj_struct& obj)
{
	obj_struct obj_pred = obj;

	obj_pred.x		= obj.x  + obj.cvx;
	obj_pred.y		= obj.y  + obj.cvy;
	obj_pred.cx		= obj.cx + obj.cvx;
	obj_pred.cy		= obj.cy + obj.cvy;

	obj_pred.px 	= obj.px + obj.vx;
	obj_pred.py 	= obj.py + obj.vy;
	obj_pred.pz 	= obj.pz + obj.vz;

	obj_pred.index	= obj.index + 1;

	return obj_pred;
}


