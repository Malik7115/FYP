/*
 * DroneControl.cpp
 *
 *  Created on: Apr 6, 2019
 *      Author: sdc
 */

#include <fstream>
#include "SerialComm.hpp"
#include "DroneControl.hpp"



double thrust_array[SYS_ID_COUNT][4];

double sat(double u, double s)
{
	if(u > s)
		u = s;
	else if(u < -s)
		u = -s;
	return u;
}

void DroneActuate(int joy1, int joy2, int joy3, int joy4)
{
	TxPacket(JOY_AUTO, joy1, joy2, joy3, joy4);
}

double mu_to_uy(int m)
{
	return m - (MID + 700);
}

int uy_to_mu(double u)
{
	return (MID + 700) + u;
}

int ux_to_mu(double u)
{
	u = sat(u, 2000);
	return MID + u;
}

int uz_to_mu(double u)
{
	u = sat(u, 2000);
	return MID  + u;
}

void SaveThrustProfile(int m)
{
	char s[100];
	ofstream outfile("MATLAB/thrust_profile_data.m");

	outfile << "thrust_array = [";

	for(int i=0;i<m;i++){

		sprintf(s, "%.16g, %.16g, %.16g, %.16g; \n", thrust_array[i][0],
				thrust_array[i][1], thrust_array[i][2], thrust_array[i][3]);
		outfile << s;

	}
	outfile << "];";
	outfile.close();

}

/***********************************************
 *                Observer Functions
 ***********************************************/

observer::observer(Mat& A_, Mat& F_, Mat& G_, Mat& H_, Mat& K_){
	A_.copyTo(A);
	F_.copyTo(F);
	G_.copyTo(G);
	H_.copyTo(H);
	K_.copyTo(K);
	x_hat.create(F.rows, 1, CV_64F);
	x_hat_next.create(F.rows, 1, CV_64F);
	x_hat.setTo(0);
	x_hat_next.setTo(0);

}

void observer::iterate(double u, double y){
	x_hat_next.copyTo(x_hat);
	x_hat_next = F*x_hat + G*u + H*y;
}
void observer::init(Mat& x){
	x.copyTo(x_hat);
	x.copyTo(x_hat_next);
}
void observer::init(double y){
	x_hat.setTo(0);
	x_hat_next.setTo(0);
	x_hat.at<double>(0) 	 = y;
	x_hat_next.at<double>(0) = y;
}
Mat observer::state(void){
	return x_hat;
}
Mat observer::state_next(void){
	return x_hat_next;
}
double observer::control(void)
{
	Mat roi(x_hat, Rect(0,0,1,x_hat.rows-1));
	Mat x = roi.clone();
	Mat u_ = -K*x;
	double u = u_.at<double>(0);
	u = sat(u, THRUST_SAT);
	return u;
}
double observer::control_dist_rej(void)
{
	Mat roi(x_hat, Rect(0,0,1,x_hat.rows-1));
	Mat x = roi.clone();
	Mat u_ = -K*x - x_hat.at<double>(2);
	double u = u_.at<double>(0);
	u = sat(u, THRUST_SAT);
	return u;
}
double observer::control(double y)
{
	Mat roi(x_hat, Rect(0,0,1,x_hat.rows-1));
	Mat x = roi.clone();
	x.at<double>(0) = y;
	Mat u_ = -K*x;
	double u = u_.at<double>(0);
	u = sat(u, THRUST_SAT);
	return u;
}


/***********************************************
 *                Predictor Functions
 ***********************************************/

predictor:: predictor(void)
{
	double A_data[3][3] = {{1, 1, 0.5},
		{0, 1, 1}, {0, 0, 1}};
	Mat A_(3, 3, CV_64F, A_data);
	A_.copyTo(A);

	double C_data[3] = {1, 0, 0};
	Mat C_(1, 3, CV_64F, C_data);
	C_.copyTo(C);

	H = C.clone();
	Mat L = C.clone();
	Mat A_inv, P;
	invert(A, A_inv);
	PseudoInv.clear();
	for(int i=0;i<MEM_LENGTH;i++){
		if(i == 1)
			P = (Mat_<double>(3,2) << 0, 1,
			                          -1,1,
									  0, 0);
		else
			invert(H, P, DECOMP_SVD);
		PseudoInv.push_back(P.clone());
		//cout << P << endl;
		L = L*A_inv;
		vconcat(L, H, H);
	}

	k 	     = 0;
	est 	 = 0;
	est_dot  = 0;
	est_ddot = 0;
	acc		 = 0;

}

void predictor::iterate(double y)
{
	if(k == 0){		// Initialization only
		y_array 	 = Mat_<double>(1,1) << y;
	}
	else if(k < MEM_LENGTH){
		y_array.push_back(y);
	}
	else{ 		// Sliding window (Array size is MEM_LENGTH)
		auto p = (double *)y_array.ptr();
		memmove(p, p+1, MEM_LENGTH_1*sizeof(double));
		p[MEM_LENGTH_1] = y;
		k = MEM_LENGTH_1;
	}


	Mat P = PseudoInv[k];


	est 		= P.row(0).dot(y_array.t());
	est_dot 	= P.row(1).dot(y_array.t());
	est_ddot 	= P.row(2).dot(y_array.t());

	acc        += est;

	k++;
}

double predictor::predict(double samples)
{
	double y = est  +  est_dot * samples;
	return y;
}

pair<double, double> predictor::predict3(double samples)
{
	double y, v;
	y = est  +  est_dot * samples  +
			0.5 * est_ddot * samples * samples;
	v = est_dot + est_ddot * samples;

	return {y, v};
}

void predictor::init(void)
{
	k = 0;
	y_array.resize(0);
	est 		= 0;
	est_dot 	= 0;
	est_ddot 	= 0;
	acc			= 0;

}
void predictor::init(double y)
{
	k = 1;
	y_array.create(1,1, CV_64F);
	y_array.at<double>(0) = y;
	est 		= y;
	est_dot 	= 0;
	est_ddot 	= 0;
	acc			= 0;

}

