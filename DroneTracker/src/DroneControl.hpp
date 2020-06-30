/*
 * DroneControl.hpp
 *
 *  Created on: Apr 6, 2019
 *      Author: sdc
 */

#ifndef DRONECONTROL_HPP_
#define DRONECONTROL_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;
using namespace cv;


#define MID     2048
#define HIGH    4095
#define LOW     0

typedef struct drone_pos{
	double px, py, pz, yaw;
}drone_pos;

void DroneActuate(int joy1, int joy2, int joy3, int joy4);

#define MEM_LENGTH		8
#define MEM_LENGTH_1	(MEM_LENGTH-1)

class predictor{
private:
	Mat A, C, G, H;
	Mat y_array;
	vector<Mat> PseudoInv;
	int k;

public:
	double est;
	double est_dot;
	double est_ddot;
	double acc;
	predictor();
	void init(void);
	void init(double y);
	void iterate(double y);
	double predict(double time);
	pair<double, double> predict3(double samples);
};

#define THRUST_SAT		600

// Observer notation adapted from  Linear System Theory (Chapter29) by Rugh
class observer{
private:
	Mat A, F, G, H;
	Mat K;
	Mat x_hat;
	Mat x_hat_next;


	double sat(double u, double v){
		if (u > v)
			u = v;
		else if(u < -v)
			u = -v;
		return u;
	}

public:
	observer(Mat& A_, Mat& F_, Mat& G_, Mat& H_, Mat& K_);

	void iterate(double u, double y);
	void init(Mat& x);
	void init(double y);
	Mat state(void);
	Mat state_next(void);
	double control(void);
	double control_dist_rej(void);
	double control(double y);
};

// thrust axis matrices
extern Mat A1,B1, C1, H1, K1, F1;

double mu_to_uy(int m);
int ux_to_mu(double u);
int uy_to_mu(double u);
int uz_to_mu(double u);

#define SYS_ID_COUNT	700

extern double thrust_array[SYS_ID_COUNT][4];


void SaveThrustProfile(int m);


#endif /* DRONECONTROL_HPP_ */
