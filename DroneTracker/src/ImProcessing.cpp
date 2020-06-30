/*
 * im_processing.cpp
 *
 *  Created on: Sep 3, 2018
 *      Author: bilal
 */
#include <stdio.h>
#include <math.h>
#include <memory.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include "DroneTracker.hpp"
#include "RealSense.hpp"
#include "ImProcessing.hpp"
#include "Tracker.hpp"
#include "VidProc.hpp"

using namespace std;
using namespace cv;

static double CalcError(const obj_struct& tgt, const obj_struct& cand);
static void CalcPairErrors(obj_struct& obj1, obj_struct& obj2);

static double w_pos    = 0.4;
static double w_area   = 0.3;
double w_aspect = 0.3;
static double min_error;

Mat frame_in, frame_gray, frame_bin;

bool verbose;
int min_size = 8;
int max_size = 2000;
int min_thresh = 120;
int max_thresh = 245;
int base_thresh = 130;

// Extract Objects from a frame
// by performing following steps
// 1. Call connectedComponentsWithStats()
// 2. Extract only objects of interest
//			- within a threshold range
//			- within a ROI
// 3. Convert the info to an array of objects
//
int ExtractObjects(const Mat& frame, const Mat& depth_image, vector<obj_struct>& obj_array)
{
	int m, n;
	Mat labels, stats, centroids;
	Mat stats2, centroids2;

	n = connectedComponentsWithStats(frame, labels,
			stats, centroids);
	m = ExtractUsefulLabels(stats, stats2, centroids,
			centroids2, min_size, max_size);

	if(m > OBJS_MAX){
		obj_array.resize(OBJS_MAX);
	}
	if(verbose)
		cout << "Total labels: " << n << "   "
		<< "Useful labels: " << m << endl;

	ToObjects(depth_image, stats2, centroids2, obj_array);

	return m;
}

int ExtractUsefulLabels(Mat& in_stats, Mat& out_stats,
		Mat& in_centroids, Mat& out_centroids, int min_size, int max_size)
{
	int i, j;
	int area;

	out_stats = in_stats.clone();
	out_centroids = in_centroids.clone();
	for(j=0,i=1;i<in_stats.rows;i++){
		area = in_stats.at<int>(i,CC_STAT_AREA);
		if(area > min_size && area < max_size){
			in_stats.row(i).copyTo(out_stats.row(j));
			in_centroids.row(i).copyTo(out_centroids.row(j));
			j++;
		}
	}
	out_stats.resize(j);
	out_centroids.resize(j);
	return j;
}

int ToObjects(const Mat& depth_image, Mat& stats, Mat& centroids, vector<obj_struct>& obj_array)
{
	int i;
	float z = 0;
	float pixel[2];
	float point[3];

	obj_array.clear();

	for(i=0;i<stats.rows;i++){
	    obj_struct obj;
	    memset(&obj, 0, sizeof(obj));
	    obj.x 			= stats.row(i).at<int>(CC_STAT_LEFT);
	    obj.y 			= stats.row(i).at<int>(CC_STAT_TOP);
	    obj.width 		= stats.row(i).at<int>(CC_STAT_WIDTH);
	    obj.height 		= stats.row(i).at<int>(CC_STAT_HEIGHT);
	    obj.area 		= stats.row(i).at<int>(CC_STAT_AREA);

	    obj.aspect 		= (double)obj.width/obj.height;
	    obj.cx			= centroids.row(i).at<double>(0);
	    obj.cy			= centroids.row(i).at<double>(1);

		pixel[0] = obj.cx;
		pixel[1] = obj.cy;
		z = depth_image.at<double>((int)pixel[1], (int)pixel[0]);
		rs2_deproject_pixel_to_point(point, &intrinsics, pixel, z);
		obj.px = point[0];
		obj.py = point[1];
		obj.pz = point[2];
		if(point[2] >= MIN_Z && point[2] <= MAX_Z)
			obj_array.push_back(obj);

	}
	return i;
}

obj_struct ObjMove(obj_struct& obj, int x, int y)
{
	obj.x 		+= x;
	obj.y 		+= y;
	obj.cx 		+= x;
	obj.cy		+= y;
	obj.px		+= x;
	obj.py		+= y;

	return obj;
}

// Search obj in frame
// If object is not found -> match = 0, Status = POOR_MATCH
obj_struct SearchObject(const obj_struct& obj)
{
	obj_struct obj_meas;

	Rect r, boundary;
	boundary.x 		= 0;
	boundary.y 		= 0;
	boundary.width 	= frame_bin.cols;
	boundary.height = frame_bin.rows;

	r.x = obj.x;	// Object bounds
	r.y = obj.y;
	r.width = obj.width;
	r.height = obj.height;

	if(!Contains(boundary, r)){ // Object outside available region
		obj_meas.status = POOR_MATCH;
		return obj_meas;
	}


	int	j = BestMatch(obj, obj_array);
	if(j == POOR_MATCH){// Object not found
		obj_meas.status = POOR_MATCH;
	}
	else{
		obj_meas = obj_array[j];
	}

	return obj_meas;
}

double Distance(obj_struct& obj1, obj_struct& obj2)
{
	double x = (obj1.px - obj2.px);
	double y = (obj1.py - obj2.py);
	double z = (obj1.pz - obj2.pz);

	return  sqrt(x * x + y * y + z * z);

}
double PairMatch(double distance, obj_struct& obj1, obj_struct& obj2)
{
	double w_match = 0.4;
	double w_distance = 0.2;
	const double max_distance_error = 0.01;
	double d  = Distance(obj1, obj2);
	double e_net;

	// Test for right and left side objects
	if(obj2.px > obj1.px)
		return 0;

	// ***************************
	// Distance error calculation
	// ***************************

	double distance_error = fabs((d - distance)/max_distance_error);

	if(distance_error > 1)
		distance_error = 1;

	// Net Error
	e_net = w_match*(1-obj1.match) + w_match*(1-obj2.match2) + w_distance * distance_error;

	if(e_net > 1.0)  // Only for finite precision effects
		e_net = 1.0;

	return (1 - e_net);

}


tuple<obj_struct, obj_struct, double> SearchPair(obj_struct& obj1, obj_struct& obj2)
{
	obj_struct obj_meas, obj_meas2;
	CalcPairErrors(obj1, obj2);

	for(auto it = obj_array.begin(); it !=  obj_array.end();){
		if(it->match < MinPd && it->match2 < MinPd)
			obj_array.erase(it);
		else
			++it;
	}

	double match, best_match = 0;
	double distance = Distance(obj1, obj2);
    unsigned int i, j;
    int m = -1, n = -1;

	for(i = 0; i < obj_array.size(); i++){
		auto objA = obj_array[i];
		if(objA.match < MinPd)
			continue;
		for(j = 0; j < obj_array.size(); j++){
			auto objB = obj_array[j];
			if(i==j || objB.match2 < MinPd)
				continue;
			match = PairMatch(distance, objA, objB);
			if(match > max(MinPd,best_match)){
				m = i;
				n = j;
				best_match = match;
			}
		}
	}
	if(m<0 || n<0){
		obj_meas.status  = POOR_MATCH;
		obj_meas2.status = POOR_MATCH;
	}
	else{
		obj_meas  = obj_array[m];
		obj_meas2 = obj_array[n];
	}
	return tie(obj_meas, obj_meas2, best_match);

}

int BestMatch(const obj_struct& tgt, vector<obj_struct>& obj_array)
{

	unsigned int i, j;
	double error, match;

	if(obj_array.empty()){
		min_error = 1;
		return POOR_MATCH;
	}

	j = 0;
	min_error = CalcError(tgt, obj_array[0]);

	for(i=1;i<obj_array.size();i++){
		error = CalcError(tgt, obj_array[i]);
		if(error < min_error){
			min_error = error;
			j = i;
		}
	}
	match = 1-min_error;

	if(match < MinPd){
		return POOR_MATCH;
	}
	else{
		obj_array[j].match = match;
	}
	return j;
}

double MinErrorBestMatch(void)
{
	return min_error;
}




//****************************************
//               Graphics
//****************************************
int DrawBBoxes(Mat& frame, const Scalar& color, vector<obj_struct>& obj_array)
{
	int n = obj_array.size();
	for(obj_struct obj: obj_array){
		if(!obj.miss_num)
			DrawBBox(frame, color, obj);
		else
			DrawBBox(frame, color/2, obj);

	}
	return n;
}
int DrawBBox(Mat& frame, const Scalar& color, obj_struct box)
{
	Rect r;
	r.x = box.x;
	r.y = box.y;
	r.width = box.width;
	r.height = box.height;
	rectangle(frame, r, color);
	return 1;
}

int DrawCentroids(Mat& frame, const Scalar& color, vector<obj_struct>& obj_array)
{
	int n = obj_array.size();
	for(obj_struct tgt: obj_array){
		Rect r;
		r.x = tgt.cx-2;
		r.y = tgt.cy-2;
		r.width = 5;
		r.height = 5;
		rectangle(frame, r, color);

	}
	return n;
}
int VacateFrame(Mat& frame, vector<obj_struct>& obj_array)
{
	for(obj_struct obj: obj_array){
		Rect r;
		r.x = obj.x;
		r.y = obj.y;
		r.width = obj.width;
		r.height = obj.height;
		rectangle(frame, r, 0, FILLED);
	}
	return obj_array.size();
}
// return true if r1 contains r2
bool Contains(Rect r1, Rect r2)
{
	int ex2, ey2;
	ex2 = r2.x + r2.width - 1;
	ey2 = r2.y + r2.height - 1;

	if(!r1.contains(Point(r2.x, r2.y)))
		return 0;
	else if(!r1.contains(Point(r2.x, ey2)))
		return 0;
	else if(!r1.contains(Point(ex2, r2.y)))
		return 0;
	else if(!r1.contains(Point(ex2, ey2)))
		return 0;
	else
		return 1;
}
bool Overlap(Rect r1, Rect r2)
{
	int ex1, ey1;
	int ex2, ey2;
	ex1 = r1.x + r1.width - 1;
	ey1 = r1.y + r1.height - 1;
	ex2 = r2.x + r2.width - 1;
	ey2 = r2.y + r2.height - 1;
	// If one rectangle is to the left
	if(ex1 < r2.x || ex2 < r1.x)
		return 0;
	// If one rectangle is above
	else if(ey1 < r2.y || ey2 < r1.y)
		return 0;
	else
		return 1;
}
bool Overlap(obj_struct obj1, obj_struct obj2)
{
	Rect r1, r2;

	r1.x = obj1.x;
	r1.y = obj1.y;
	r1.width = obj1.width;
	r1.height = obj1.height;
	r2.x = obj2.x;
	r2.y = obj2.y;
	r2.width = obj2.width;
	r2.height = obj2.height;

	return Overlap(r1, r2);
}
void RemoveOverlappingObjs(vector<obj_struct>& obj_array, vector<obj_struct>& obj_array_local)
{
	for(auto iter = obj_array.begin(); iter < obj_array.end(); iter++){
		for(auto obj2:obj_array_local){
			if(Overlap(*iter, obj2)){
				obj_array.erase(iter);
				iter--; // To compensate for erasing
				break;
			}
		}
	}
}


static double CalcError(const obj_struct& tgt, const obj_struct& cand)
{

	const double max_area_ratio 	= 2.0;
	const double max_pos_error 		= 0.05;
	const double max_aspect_ratio 	= 1.5;
	double area_ratio;
	double area_error;
	double aspect_ratio;
	double aspect_error;
	double pos_error, pos_error_x, pos_error_y,  pos_error_z;
	double e_net;

	// ***********************
	// Area error calculation
	// ***********************

	// Ratio of areas >= 1
	if(tgt.area >= cand.area)
		area_ratio = (double)tgt.area/cand.area;
	else
		area_ratio = (double)cand.area/tgt.area;

	// Normalized area error
	area_error = (area_ratio - 1)/(max_area_ratio - 1);

	// Area error saturation
	if(area_error > 1.0)
		area_error = 1.0;


	// *************************
	// Aspect error calculation
	// *************************

	// Ratio of aspects >= 1
	if(tgt.aspect >= cand.aspect)
		aspect_ratio = (double)tgt.aspect/cand.aspect;
	else
		aspect_ratio = (double)cand.aspect/tgt.aspect;


	// ***************************
	// Position error calculation
	// ***************************

	pos_error_x = fabs((tgt.px - cand.px)/max_pos_error);
	pos_error_y = fabs((tgt.py - cand.py)/max_pos_error);
	pos_error_z = fabs((tgt.pz - cand.pz)/max_pos_error);

	// Position error saturation
	if(pos_error_x > 0.577)
		pos_error_x = 0.577;
	if(pos_error_y > 0.577)
		pos_error_y = 0.577;
	if(pos_error_z > 0.577)
		pos_error_z = 0.577;

	pos_error = sqrt(pos_error_x * pos_error_x +
					pos_error_y * pos_error_y +
					pos_error_z * pos_error_z);


	// Normalized area error
	aspect_error = (aspect_ratio - 1)/(max_aspect_ratio - 1);


	//aspect_error = fabs(tgt.aspect - cand.aspect)/max_aspect_error;

	//Aspect error saturation
	if(aspect_error > 1.0)
		aspect_error = 1.0;

	// Net Error
	e_net = w_pos*pos_error + w_area*area_error + w_aspect*aspect_error;

	if(e_net > 1.0)  // Only for finite precision effects
		e_net = 1.0;

	return e_net;
}

static void CalcPairErrors(obj_struct& obj1, obj_struct& obj2)
{
	for(auto& obj:obj_array){
		obj.match  = 1 - CalcError(obj, obj1);
		obj.match2 = 1 - CalcError(obj, obj2);
	}
}
Mat DepthFilter(Mat& color_image, Mat& depth_image, double min, double max)
{
	Mat m = (depth_image >= min) & (depth_image <= max);

	Mat channels[3] = {m, m, m};

    Mat mask;
    merge(channels, 3, mask);

    Mat color_image_rstd = color_image.clone();
    color_image_rstd = color_image_rstd & mask;

    return color_image_rstd;
}
