/*
 * SerialComm.cpp
 *
 *  Created on: Mar 31, 2019
 *      Author: sdc
 */


#include <iostream>
#include <SerialPort.h>
#include <SerialStream.h>
#include <vector>
#include <numeric>

using namespace std;

//class SerialPort port(B115200, CS8, STOP_BITS_1, PARITY_NONE, FLOW_CONTROL_NONE);

class SerialPort port("/dev/ttyS0");

void SerialInit(void)
{
	system("sudo chmod 777 /dev/ttyS0");

	port.Open(SerialPort::BAUD_115200);

	cout << (port.IsOpen()?"Serial Port Open":"Unable to open Serial Port") << endl;

}


/******************************************
 * 					PROTOCOL			  *
 *****************************************/
void TxPacket(unsigned char cmd, int joy1, int joy2, int joy3, int joy4)
{
	SerialPort::DataBuffer data;
	data.clear();
	data.push_back(joy1 & 0xFF);
	data.push_back((joy1 >> 8) & 0xFF);
	data.push_back(joy2 & 0xFF);
	data.push_back((joy2 >> 8) & 0xFF);
	data.push_back(joy3 & 0xFF);
	data.push_back((joy3 >> 8) & 0xFF);
	data.push_back(joy4 & 0xFF);
	data.push_back((joy4 >> 8) & 0xFF);


	int ref_cmd1 = cmd & 0x0F;
	int ref_cmd2 = (cmd>>4) & 0x0F;


	// fill in commands in tailor
	data.push_back(ref_cmd1);
	data.push_back(ref_cmd2);

	uint8_t checksum = accumulate(data.begin(), data.end(), 0) & 0x7F;

	SerialPort::DataBuffer tx_buf = {
			(uint8_t)(0xA0|(data[data.size()-2])), 	// header low
			(uint8_t)(0xA0|(data[data.size()-1])),	// header high
			checksum, 										// checksum
			(uint8_t)(data.size())			// no of bytes
	};

	tx_buf.insert(tx_buf.end(), data.begin(), data.end());

	port.Write(tx_buf);
}

