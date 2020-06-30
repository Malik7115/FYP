/*
 * ImProcessing.hpp
 *
 *  Created on: Sep 5, 2018
 *      Author: bilal
 */

#ifndef IMPROCESSING_HPP_
#define IMPROCESSING_HPP_
#include <vector>
#include "opencv2/opencv.hpp"

// Object defines
#define POOR_MATCH		-1
#define OBJ_NO_FREE		-2
#define OBJS_MAX		32

#define MinPd			0.7
#define MAX_MISS_NUM	30
#define MAX_MISS_TRAJ	10
#define MIN_Z			0.2
#define MAX_Z			2.5

#define SWELL			50

using namespace std;
using namespace cv;


/****************************** Structures ***********************************/
typedef struct window{
	int sx, sy, ex, ey;
}window;

typedef struct obj_struct
{
	int x, y;					// Upper left corner
	int width, height;

//  Main parameters
	double cx, cy;				// Centroid
	double cvx, cvy;			// velocity components
	double dir;					// atan(vy/vx)

	double aspect;				// Aspect ratio: width/height
	double area;

	double thresh;

	double px, py, pz;			// Physical position in space in meters
	double vx, vy, vz;			// velocity components

	double match;
	double match2;

	int miss_num;

	int index;					// Time index

	int status;


	obj_struct(void)
	{

		x = y 		= 0;
		width 		= 0;
		height 		= 0;

		cx = cy 	= 0;
		cvx = cvy 	= 0;
		px 			= 0;
		py 			= 0;
		pz 			= 0;
		vx 			= 0;
		vy 			= 0;
		vz 			= 0;
		dir 		= 0;

		aspect 		= 1;
		area 		= 1;

		match		= 0;
		match2      = 0;
		thresh 		= 0;

		miss_num 	= 0;

		index 		= 0;

		status 		= 0;

	}
	obj_struct(const obj_struct& q)
	{
		*this = q;
	}

}obj_struct;


typedef struct coord{
	int x, y;
}coord;




int ExtractObjects(const Mat& frame_bin, const Mat& depth_image, vector<obj_struct>& obj_array);
int ExtractUsefulLabels(Mat& in_stats, Mat& out_stats,
		Mat& in_centroids, Mat& out_centroids, int min_size, int max_size);
Mat Preprocess(Mat& src, Mat& dest, int);
int ToObjects(const Mat& depth_image, Mat& stats, Mat& centroids, vector<obj_struct>& obj_array);
int DrawBBoxes(Mat& frame, const Scalar& color, vector<obj_struct>& obj_array);
int DrawBBox(Mat& frame, const Scalar& color, obj_struct box);
int DrawCentroids(Mat& frame, const Scalar& color, vector<obj_struct>& obj_array);
		obj_struct SelectObject(String window, Mat& frame, int thresh);
int BestMatch(const obj_struct& tgt, vector<obj_struct>& obj_array);
double MinErrorBestMatch(void);
tuple<obj_struct, obj_struct, double> SearchPair(obj_struct& obj1, obj_struct& obj2);
obj_struct ExtractBestObject( Mat& frame,  Mat& frame2, vector<obj_struct>& obj_array,
		obj_struct tgt_pred, vector<int> thresh_dist);
obj_struct ExtractBestObject( Mat& frame,  Mat& frame2,
		vector<obj_struct>& obj_array, obj_struct tgt_pred);
obj_struct SearchObject(const obj_struct& obj);
int VacateFrame(Mat& frame, vector<obj_struct>& obj_array);
void RemoveOverlappingObjs(vector<obj_struct>& obj_array, vector<obj_struct>& obj_array_local);
bool Contains(Rect r1, Rect r2);
Mat DepthFilter(Mat& color_image, Mat& depth_image, double min, double max);
double Distance(obj_struct& obj1, obj_struct& obj2);

extern int min_size;
extern int max_size;
extern int min_thresh;
extern int max_thresh;
extern int base_thresh;
extern Mat frame_in, frame_gray, frame_bin;



#endif /* IMPROCESSING_HPP_ */
