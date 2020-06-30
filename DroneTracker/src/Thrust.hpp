/*
 * thrust.hpp
 *
 *  Created on: Apr 17, 2019
 *      Author: sdc
 */

#ifndef THRUST_HPP_
#define THRUST_HPP_

#include "DroneControl.hpp"


extern observer thrust_observer;
extern predictor thrust_pred;
extern predictor x_pred;
extern predictor z_pred;



void SaveThrustProfile(int n);


#endif /* THRUST_HPP_ */
