/*
 * RLS.hpp
 *
 *  Created on: Sep 2, 2018
 *      Author: bilal
 */

#ifndef TRACKER_HPP_
#define TRACKER_HPP_

#include "opencv2/opencv.hpp"
#include "ImProcessing.hpp"

#define TIME_INDEX_MAX		20

using namespace std;
using namespace cv;

void PrepareRLS(void);
obj_struct TrackObject(const obj_struct& tgt_meas, const obj_struct& tgt_est);
int CorrelateObjsComplete(vector<obj_struct>& obj_array_prev,
		vector<obj_struct>& obj_array, vector<int>& th_dist);
int CorrelateObjs(vector<obj_struct>& obj_array_prev, vector<obj_struct>& obj_array);
obj_struct PredObject(const obj_struct& obj);


#endif /* TRACKER_HPP_ */
