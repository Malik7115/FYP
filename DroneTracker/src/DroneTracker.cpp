// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <sys/resource.h>
#include <sys/types.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <iostream>
#include <thread>
#include <pthread.h>
#include <mutex>
#include <condition_variable>
#include <time.h>
#include <signal.h>
#include <fstream>
#include "DroneTracker.hpp"
#include "RealSense.hpp"
#include "VidProc.hpp"
#include "ImProcessing.hpp"
#include "SerialComm.hpp"
#include "DroneControl.hpp"
#include "Thrust.hpp"

#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

using namespace std;
using namespace cv;
using namespace rs2;
using namespace chrono;

void SaveImage(void);

volatile bool running 	 = false;
volatile bool tracking 	 = false;
volatile bool frame_cap  = false;
volatile bool frame_proc = false;

volatile int mode;

char VidCapString[100];
extern char VidProcString[];

// Declare RealSense pipeline, encapsulating the actual device and sensors
Mat depth_image;
Mat color_image;
Mat depth_image_buf;
Mat color_image_buf;

int dr_b_min = 0, dr_b_max = 255;
int dr_g_min = 0, dr_g_max = 255;
int dr_r_min = 0, dr_r_max = 255;

int l1_b_min = 0, l1_b_max = 255;
int l1_g_min = 0, l1_g_max = 255;
int l1_r_min = 0, l1_r_max = 255;

int l2_b_min = 0, l2_b_max = 255;
int l2_g_min = 0, l2_g_max = 255;
int l2_r_min = 0, l2_r_max = 255;

int d_min = 1, d_max = 4000;

static void onMouse( int event, int x, int y, int, void * s);
static mouse_struct mouse;
volatile int selected = -1;

// When frame capture and alignment is done
// frame_cap flag is true
// color_image stores the current color frame (Type - 3 color/24bit)
// depth_image stores the current depth image aligned with color frame (Type - double/64bit)

void VidCap()
{
	static system_clock::time_point   current, prev, start, end;
    duration<double, milli> diff;

	char s[100];

	id_t tid;
	tid = gettid();
	int t_priority = getpriority(PRIO_PROCESS, tid);

	vid_mutex.lock();
	cout << tid << endl;
	cout << t_priority << endl;
	cout << setpriority(PRIO_PROCESS, tid, -20) << endl;
	cout << errno << endl << EACCES << endl;
	t_priority = getpriority(PRIO_PROCESS, tid);
	cout << t_priority << endl;
	sprintf(s, "sudo renice -20 -p %d", tid);
	system(s);
	t_priority = getpriority(PRIO_PROCESS, tid);
	cout << t_priority << endl;
	vid_mutex.unlock();


	static pipeline pipe;
	static pipeline_profile selection;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 360, RS2_FORMAT_RGB8, 60);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 360, RS2_FORMAT_Z16, 60);

	vid_mutex.lock();
	cout << "Video Cap Thread ID: " << this_thread::get_id() << '\n';
	vid_mutex.unlock();

	// https://github.com/IntelRealSense/librealsense/wiki/API-How-To#get-video-stream-intrinsics
	selection = pipe.start(cfg);
	auto color_stream = selection.get_stream(RS2_STREAM_COLOR)
	                            		 .as<rs2::video_stream_profile>();
	intrinsics = color_stream.get_intrinsics();
	// -----------

	prev = system_clock::now();
	for(int i=0;i<30;i++) { // Warm up
		frameset data = pipe.wait_for_frames();
		current = system_clock::now();
		double difference = (diff = current-prev).count();
		sprintf(VidCapString, "Frame interval: %05.1fms", difference);
		prev = current;
		cout << VidCapString << endl;

	}

	bool skip;
	prev = system_clock::now();

	while(running) {
		{
			// Realsense data
			frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
			current = system_clock::now();

			if(skip){
				skip = false;
				this_thread::sleep_for(chrono::milliseconds(10));
				continue;
			}
			else
				skip = true;

			// Frame Alignment
			// https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#frame-alignment
			start = system_clock::now();
			rs2::align align(RS2_STREAM_COLOR);
			auto aligned_frames = align.process(data);
			video_frame colors = aligned_frames.first(RS2_STREAM_COLOR);
			depth_frame depth = aligned_frames.get_depth_frame();
			// -----------

			//cout << (void*)depth.get_data() << endl;
			//cout << (void*)colors.get_data() << endl;


			// Create OpenCV matrices
			Mat color_image_loc = frame_to_mat(colors);

			// Mutex locked for shared data
			lock_guard<mutex> lock(vid_mutex);
			frame_proc  = true;
			color_image = color_image_loc.clone();
			depth_image = depth_frame_to_meters(pipe, depth);
			frame_cap 	= true;

			double difference = (diff = current-prev).count();
			sprintf(VidCapString, "Frame interval: %05.1fms", difference);
			if(difference < 30 || difference > 36)
				cerr << VidCapString << endl;
			end = system_clock::now();
			difference = (diff = end-start).count();
			sprintf(s, ",   Acquisition time: %4.1fms", difference);
			strcat(VidCapString, s);
			cout << VidCapString << endl;
			//cout << data.get_frame_number()<<"   ";
			//cout << data.get_timestamp() <<"   ";
			prev = current;
			// vid_mutex is automatically released when lock
			// goes out of scope
		}
		cond.notify_one();
		//this_thread::sleep_for(chrono::milliseconds(20));
	}
	lock_guard<mutex> lock(vid_mutex);
	frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
	pipe.stop();
	frame_cap = true;
	cond.notify_one();

}


int main(int argc, char * argv[]) try
{

	SerialInit();

//	TxPacket(7, -2233,3334,2335,6663);
//	usleep(100000);
//	exit(0);

//	cout << H1 << endl;
//	cout << K1 << endl;

//	thrust_observer.init(1);
//	thrust_observer.iterate(1, 1);
//	thrust_observer.control(1);
//
//	double y;
//	for(int i=2; i< 17; i++){
//		thrust_pred.iterate((double)i);
//		y = thrust_pred.predict(3.4);
//		cout << y << endl;
//	}
//
//	exit(0);

	ifstream infile("RealSenseData");
	infile >> dr_b_min;
	infile >> dr_b_max;
	infile >> dr_g_min;
	infile >> dr_g_max;
	infile >> dr_r_min;
	infile >> dr_r_max;

	infile >> l1_b_min;
	infile >> l1_b_max;
	infile >> l1_g_min;
	infile >> l1_g_max;
	infile >> l1_r_min;
	infile >> l1_r_max;

	infile >> l2_b_min;
	infile >> l2_b_max;
	infile >> l2_g_min;
	infile >> l2_g_max;
	infile >> l2_r_min;
	infile >> l2_r_max;

	infile >> d_min;
	infile >> d_max;
	infile.close();

	namedWindow("Data", WINDOW_AUTOSIZE);
	namedWindow("Color Display", WINDOW_AUTOSIZE);
	moveWindow("Color Display", 640, 0);
	namedWindow("Binary Display", WINDOW_AUTOSIZE);
	moveWindow("Binary Display", 0, 0);
	moveWindow("Data", 0, 500);

	createTrackbar("Drone B Min", "Color Display", &dr_b_min, 255, NULL, NULL);
	createTrackbar("Drone B Max", "Color Display", &dr_b_max, 255, NULL, NULL);
	createTrackbar("Drone G Min", "Color Display", &dr_g_min, 255, NULL, NULL);
	createTrackbar("Drone G Max", "Color Display", &dr_g_max, 255, NULL, NULL);
	createTrackbar("Drone R Min", "Color Display", &dr_r_min, 255, NULL, NULL);
	createTrackbar("Drone R Max", "Color Display", &dr_r_max, 255, NULL, NULL);
	createTrackbar("D Min",       "Color Display", &d_min,   4000, NULL, NULL);
	createTrackbar("D Max",       "Color Display", &d_max,   4000, NULL, NULL);



//	createTrackbar("LED1 B Min", "Binary Display", &l1_b_min, 255, NULL, NULL);
//	createTrackbar("LED1 B Max", "Binary Display", &l1_b_max, 255, NULL, NULL);
//	createTrackbar("LED1 G Min", "Binary Display", &l1_g_min, 255, NULL, NULL);
//	createTrackbar("LED1 G Max", "Binary Display", &l1_g_max, 255, NULL, NULL);
//	createTrackbar("LED1 R Min", "Binary Display", &l1_r_min, 255, NULL, NULL);
//	createTrackbar("LED1 R Max", "Binary Display", &l1_r_max, 255, NULL, NULL);
//
//
//	createTrackbar("LED2 B Min", "Binary Display", &l2_b_min, 255, NULL, NULL);
//	createTrackbar("LED2 B Max", "Binary Display", &l2_b_max, 255, NULL, NULL);
//	createTrackbar("LED2 G Min", "Binary Display", &l2_g_min, 255, NULL, NULL);
//	createTrackbar("LED2 G Max", "Binary Display", &l2_g_max, 255, NULL, NULL);
//	createTrackbar("LED2 R Min", "Binary Display", &l2_r_min, 255, NULL, NULL);
//	createTrackbar("LED2 R Max", "Binary Display", &l2_r_max, 255, NULL, NULL);
//


	Mat blank(300, 640, CV_8UC1, Scalar(0));

	Mat color_image_loc;


	running = true;
	auto t_VidCap = thread(VidCap);
	auto t_VidProc = thread(VidProc);



	setMouseCallback("Color Display", onMouse, NULL);
	char c;
	while (running)
	{
		c = waitKey(1);
		char s[100];

		while(!frame_done);  // wait for frame
		frame_done = 0;

		{
			lock_guard<mutex> lock(vid_mutex);
			color_image_loc = color_image.clone();
		}


		switch(mode){
		case DETECTION:
			DrawBBoxes(color_image_loc, Scalar(0, 0, 255), obj_array);

			putText(blank, "Mode: Detection", Point(0, 80), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			sprintf(s, "Number of detected objects: %d", (int)obj_array.size());
			putText(blank, s, Point(0, 100), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			putText(blank, VidCapString, Point(0, 120), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			putText(blank, VidProcString, Point(0, 140), FONT_HERSHEY_SIMPLEX, 0.5, 255);

			break;
		case TRACKING:
			DrawBBoxes(color_image_loc, Scalar(0, 0, 255), obj_array);
			DrawBBox(color_image_loc, Scalar(255, 255, 0), obj_meas);
			putText(blank, "Mode: Tracking", Point(0, 80), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			sprintf(s, "Measured Target Coords: (%3.3f, %3.3f, %3.3f)",
					obj_meas.px, obj_meas.py, obj_meas.pz);
			putText(blank, s, Point(0, 100), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			sprintf(s, "Estimated Target Coords: (%3.3f, %3.3f, %3.3f)",
					obj_est.px, obj_est.py, obj_est.pz);
			putText(blank, s, Point(0, 120), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			sprintf(s, "Predicted Target Coords: (%3.3f, %3.3f, %3.3f)",
					obj_pred.px, obj_pred.py, obj_pred.pz);
			putText(blank, s, Point(0, 140), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			sprintf(s, "Match: %3.2f, Miss Count: %d", obj_meas.match, obj_pred.miss_num);
			putText(blank, s, Point(0, 160), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			putText(blank, VidCapString, Point(0, 180), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			putText(blank, VidProcString, Point(0, 200), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			break;
		case DUAL_TRACKING:
			DrawBBox(color_image_loc, Scalar(255, 255, 0), obj_meas);
			DrawBBox(color_image_loc, Scalar(0, 255, 255), obj_meas2);
			putText(blank, "Mode: Dual Tracking", Point(0, 80), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			sprintf(s, "First Target Coords: (%3.3f, %3.3f, %3.3f)",
					obj_meas.px, obj_meas.py, obj_meas.pz);
			putText(blank, s, Point(0, 100), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			sprintf(s, "Second Target Coords: (%3.3f, %3.3f, %3.3f)",
					obj_meas2.px, obj_meas2.py, obj_meas2.pz);
			putText(blank, s, Point(0, 120), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			sprintf(s, "Target Coords: (%3.3f, %3.3f, %3.3f)",
					tgt_px, tgt_py, tgt_pz);
			putText(blank, s, Point(0, 140), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			sprintf(s, "LED Distance: %3.3f, Yaw: %3.3f)",
					Distance(obj_meas, obj_meas2), tgt_yaw * 180 / M_PI);
			putText(blank, s, Point(0, 160), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			sprintf(s, "Match: %3.2f, Miss Count: %d", dual_match, obj_pred.miss_num);
			putText(blank, s, Point(0, 180), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			putText(blank, VidCapString, Point(0, 200), FONT_HERSHEY_SIMPLEX, 0.5, 255);
			putText(blank, VidProcString, Point(0, 220), FONT_HERSHEY_SIMPLEX, 0.5, 255);
		}

		// cout << " Obj No:" << selected << " Mode: " << mode << endl;


		static int frame_miss;

		if(!frame_proc)
		{
			imshow("Color Display", color_image_loc);
			imshow("Binary Display", display_image);
		}
		else
			frame_miss++;

		sprintf(s, "Missed frame: %d", frame_miss);
		putText(blank, s, Point(0, 60), FONT_HERSHEY_SIMPLEX, 0.5, 255);

		imshow("Data", blank);
		memset(blank.data, 0, blank.rows*blank.cols);
		switch(c){
		case -1:
			break;
		case 'd':
		case 'D':
			ModeChange(DETECTION);
			break;
		case 'p':
		case 'P':
			waitKey(0);
			break;
		case 'i':
		case 'I':
			VidProcCmd(c);
			break;
		case 's':
		case 'S':
			color_image_buf = color_image.clone();
			depth_image_buf = depth_image.clone();
			break;
		case 27:
			running = 0;
		}
	}
	running = false;

	t_VidCap.join();
	t_VidProc.join();

	ofstream outfile("RealSenseData");
	outfile << dr_b_min << endl;
	outfile << dr_b_max << endl;
	outfile << dr_g_min << endl;
	outfile << dr_g_max << endl;
	outfile << dr_r_min << endl;
	outfile << dr_r_max << endl;

	outfile << l1_b_min << endl;
	outfile << l1_b_max << endl;
	outfile << l1_g_min << endl;
	outfile << l1_g_max << endl;
	outfile << l1_r_min << endl;
	outfile << l1_r_max << endl;

	outfile << l2_b_min << endl;
	outfile << l2_b_max << endl;
	outfile << l2_g_min << endl;
	outfile << l2_g_max << endl;
	outfile << l2_r_min << endl;
	outfile << l2_r_max << endl;


	outfile << d_min << endl;
	outfile << d_max << endl;
	outfile.close();
	SaveImage();

	return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
	cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}


static void onMouse( int event, int x, int y, int, void * s)
{
	if( event != EVENT_LBUTTONDOWN )
		return;

	mouse.pos 	  = Point(x, y);
	mouse.clicked = true;

	UpdateMouse(mouse);
}

int ModeChange(int new_mode)
{

	switch(new_mode){
	case DETECTION:
		selected = -1;
		setMouseCallback("Color Display", onMouse, NULL);
		memset(&obj_meas, 0, sizeof(obj_struct));
		obj_array.clear();
		mode = DETECTION;
		return 0;
	case TRACKING:
		mode = TRACKING;
		setMouseCallback("Color Display", onMouse, NULL );
		return 0;
	case DUAL_TRACKING:
		mode = DUAL_TRACKING;
		setMouseCallback("Color Display", NULL, NULL );
		return 0;
	default:
		return -1; // Error - Undefined mode
	}

}

string save_buf;

void SaveImage(void)
{
	if(!color_image_buf.rows)  // Empty image
		return;
	imwrite("color_image_file.bmp", color_image_buf);

	int i, j;
	char s[100];


	ofstream outfile("depth_image_file.m");

	outfile << "depth_image = [";

	for(i=0;i<depth_image_buf.rows;i++){
		save_buf.clear();
		for(j=0;j<depth_image_buf.cols - 1;j++){
			sprintf(s, "%f, ", depth_image_buf.at<float>(i,j));
			save_buf += s;
		}

		sprintf(s, "%f; \n", depth_image_buf.at<float>(i,j));
		save_buf += s;
		outfile << save_buf;

	}
	outfile << "];";
	outfile.close();

	float z = 0;
	float pixel[2];
	float point[3];

	ofstream outfile2("pixel_image_file.m");
	outfile2 << "pixel_image = [";

	for(i=0;i<color_image_buf.rows;i++){
		save_buf.clear();
		for(j=0;j<color_image_buf.cols - 1;j++){
			pixel[0] = j;
			pixel[1] = i;
			z = depth_image_buf.at<float>(i,j);
			rs2_deproject_pixel_to_point(point, &intrinsics, pixel, z);

			sprintf(s, "%f+%fi, ", point[0], point[1]);
			if(strstr(s, "nan") || strstr(s, "inf"))  //nan or inf
				sprintf(s, "0+0i, ");
			save_buf += s;
		}

		pixel[0] = j;
		pixel[1] = i;
		z = depth_image_buf.at<float>(i,j);
		rs2_deproject_pixel_to_point(point, &intrinsics, pixel, z);

		sprintf(s, "%f+%fi; \n", point[0], point[1]);
		if(strstr(s, "nan") || strstr(s, "inf"))  //nan or inf
			sprintf(s, "0+0i, ");
		save_buf += s;
		outfile2 << save_buf;

	}
	outfile2 << "];";
	outfile2.close();


}



