#include <opencv2/opencv.hpp>
#include "DroneControl.hpp"
using namespace std;
using namespace cv; 

double A1_data[2][2] = {{1, 0.02834686894262108, },
{0, 0.7165313105737893, },
};
Mat A1(2, 2, CV_64F, A1_data);

double B1_data[2] = {9.972928781424515e-06, 0.0005669373788524214, };
Mat B1(2, 1, CV_64F, B1_data);

double C1_data[2] = {1, 0, };
Mat C1(1, 2, CV_64F, C1_data);

double K1_data[2] = {2000, 1000, };
Mat K1(1, 2, CV_64F, K1_data);

double H1_data[3] = {1.516531310573789, 18.91291556048384, 11429.8337730291, };
Mat H1(3, 1, CV_64F, H1_data);

double G1_data[3] = {9.972928781424515e-06, 0.0005669373788524214, 0, };
Mat G1(3, 1, CV_64F, G1_data);

double F1_data[3][3] = {{-0.5165313105737888, 0.02834686894262108, 9.972928781424515e-06, },
{-18.91291556048384, 0.7165313105737893, 0.0005669373788524214, },
{-11429.8337730291, 0, 1, },
};
Mat F1(3, 3, CV_64F, F1_data);

observer thrust_observer(A1, F1, G1, H1, K1);

predictor thrust_pred;

predictor x_pred;

predictor z_pred;

