/*
 * SerialComm.hpp
 *
 *  Created on: Mar 31, 2019
 *      Author: sdc
 */

#ifndef SERIALCOMM_HPP_
#define SERIALCOMM_HPP_

#define JOY_MANUAL		0
#define JOY_AUTO		1


void SerialInit(void);

void TxPacket(unsigned char cmd, int joy1, int joy2, int joy3, int joy4);


#endif /* SERIALCOMM_HPP_ */
