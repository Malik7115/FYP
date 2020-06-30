/*
 * ComMotorControl.cpp
 *
 *  Created on: 3 Jan 2019
 *      Author: sdc
 */


#include <thread>
#include <numeric>

#include "ComTurret.h"

#include "SOT_main.hpp"
#include "Conversions.hpp"
#include "ComConsole.hpp"

using namespace std;


//ComTurret::ComTurret(timeout_sec read_tout, const speed_t speed, const char* dev_name) :
//							serial(read_tout, speed, dev_name) ,// calling constructor of serial port
//							serial_port(dev_name)
//{
//	// TODO Auto-generated constructor stub
//	rx_buf_count    = 0;
//	rx_buf_size		= 0;
//	rx_chksum_calc  = 0;
//	rx_state 		= 0;
//	rx_PSyncFlag 	= 0;
//
//	// set cmd1 & cmd2 as per operational modes
//	lrf_mode = LRF_INACTIVE;
//	ref_cmd1 = ref_cmd2 = 0;
//	ref_azim = ref_elev = 0; // to be filled by console / video processor
//	vset_mask = CMD_TI_WFOV;
//
//	mode = 0;
//	azim = elev = 0;
//
//	status = serial.get_status();
//}



void Receiver(ComTurret * turret)
{
	while(running) {
		turret->Receive();
	}
}


ComTurret::ComTurret(const char* dev_name) :
																		serial_port(dev_name)
{
	char buffer[100];

	// stty -F /dev/ttyS0 -a => prints all current settings in terminal; (in terminal output => -flag_name means flag is clear, flag_name means flag is set)
	sprintf(buffer, "sudo chown -R $USERNAME %s", dev_name);
	system(buffer);

	// TODO Auto-generated constructor stub
	rx_buf_count    = 0;
	rx_buf_size		= sizeof(rx_buf)/sizeof(uint8_t);
	rx_chksum_calc  = 0;
	rx_state 		= 0;
	rx_PSyncFlag 	= 0;

	// set cmd1 & cmd2 as per operational modes
	lrf_set.active = 0;
	lrf_set.mode   = LRF_SINGLE_PULSE;

	ref_cmd1 = ref_cmd2 = 0;
	ref_azim = ref_elev = 0; // to be filled by console / video processor
	vset_mask = CMD_TI_WFOV;

	mode = TM_CALIBRATION;
	azim = elev = 0;

	pkt_rcv = false;

	serial_port.Open(SerialPort::BAUD_115200);

	cout << "Serial Settings" << endl;
	cout << serial_port.GetBaudRate() << endl;
	cout << serial_port.GetCharSize() << endl;
	cout << serial_port.GetNumOfStopBits() << endl;
	cout << serial_port.GetParity() << endl;
	cout << serial_port.GetFlowControl() << endl;

	status = 0;
}

ComTurret::~ComTurret()
{
	serial_port.Close();
}

void ComTurret::Transmit( void )
{
	vector<unsigned char> data;
	GenerateData(data);
	TxAXAX(data);
}

void ComTurret::Receive(void)
{
	uint8_t data_recv;
	SerialPort::DataBuffer buf;
	data_recv = serial_port.ReadByte(0);
	if(RxAXAX(data_recv, buf)) {
		ExtractData(buf);
	}
}

void ComTurret::SetRefPos(float azim, float elev)
{
	lock_guard<mutex> lock(mtx);
	ref_azim = azim;
	ref_elev = elev;
}

void ComTurret::SetLRFMode(lrf_settings_struct lmode)
{
	lock_guard<mutex> lock(mtx);
	lrf_set = lmode;
}

void ComTurret::SetVidSrc(vid_settings_struct vset)
{
	lock_guard<mutex> lock(mtx);
	vsrc = vset;
}

pair<float, float> ComTurret::GetRefPos(void)
{
	lock_guard<mutex> lock(mtx);
	return make_pair(ref_azim, ref_elev);
}

float ComTurret::GetAzim(void)
{
	lock_guard<mutex> lock(mtx);
	return azim;
}
float ComTurret::GetElev(void)
{
	lock_guard<mutex> lock(mtx);
	return elev;
}
int ComTurret::GetStatus(void)
{
	return status;
}

/******************************************
 * 					PRIVATE				  *
 *****************************************/
void ComTurret::ExtractData(SerialPort::DataBuffer& data)
{
	uint8_t cmd1, cmd2;
	cmd1 = data[data.size() - 2] & 0x0F;	// 2nd last tailor byte
	cmd2 = data[data.size() - 1] & 0x0F;	//     last tailor byte
	int i;

	if((cmd1 == 1) && (cmd2 == 1))
		mode = TM_CALIBRATION;
	else if((cmd1 == 1) && (cmd2 == 0))
		mode = TM_TRACKING;
	else if((cmd1 == 2) && (cmd2 == 0))
		mode = TM_LRF_EN;
	else if((cmd1 == 8) && (cmd2 == 2)) {
		mode = TM_FAULT;
		return;
	}
	else  {
		cerr << " Turret Comm : Protocol error " << endl;
		return;
	}

	i = 0;
	{
		lock_guard<mutex> lock(mtx);
		azim = Enc2Deg((int16_t)(data[i] | data[i+1]<<8)); i += 2;
		azim = CircCorr<float>(azim, 360);
		elev = Enc2Deg((int16_t)(data[i] | data[i+1]<<8)); i += 2;
		elev = CircCorr<float>(elev, 360);

		if (mode == TM_LRF_EN) {
			lrf.range1 = (data[i] | data[i+1]<<8);			i += 2;
			lrf.range2 = (data[i] | data[i+1]<<8);			i += 2;
			lrf.range3 = (data[i] | data[i+1]<<8);			i += 2;
		}
	}
}

void ComTurret::GenerateData(SerialPort::DataBuffer& data)
{
	data.clear();
	int16_t ref;
	// fill transmission buffer
	{
		lock_guard<mutex> lock(mtx);


		ref = (int16_t)CircCorr2<float>(Deg2Enc(ref_azim), ENC_RESOLUTION/2);
		data.push_back(ref & 0xFF);
		data.push_back((ref >> 8) & 0xFF);

		ref = (int16_t)CircCorr2<float>(Deg2Enc(ref_elev), ENC_RESOLUTION/2);
		data.push_back(ref & 0xFF);
		data.push_back((ref >> 8) & 0xFF);

		// fill in modes information
		uint8_t command;
		command = CMD_SOT_OPERATIONAL_MODE;
		if(lrf_set.active) {
			switch (lrf_set.mode) {
			case LRF_INACTIVE:				// should never come when active
				command |= CMD_LRF_RANGING_INACTIVE;
				break;
			case LRF_SINGLE_PULSE:
				command |= CMD_LRF_SINGLE_PULSE;
				break;
			case LRF_1PPS:
				command |= CMD_LRF_ONE_PPS;
				break;
			case LEF_3PPS:
				command |= CMD_LRF_THREE_PPS;
				break;
			default:							// should never come when active
				command |= CMD_LRF_RANGING_INACTIVE;
				break;
			}
		}
		else {
			command |= CMD_LRF_RANGING_INACTIVE;
		}

		ref_cmd1 = command & 0x0F;
		ref_cmd2 = (command>>4) & 0x0F;


		// fill in commands in tailor
		data.push_back(ref_cmd1);
		data.push_back(ref_cmd2);


		// setting video settings mask
		switch (vsrc.src) {
		case VSRC_TI:
			if(vsrc.calib_en)
				vset_mask = CMD_TI_NO_DELAY;
			else {
				switch (vsrc.zoom) {
				case NFOV:
					vset_mask = CMD_TI_NFOV;
					break;
				case MFOV:
					vset_mask = CMD_TI_MFOV;
					break;
				case WFOV:
					vset_mask = CMD_TI_WFOV;
					break;
				default:
					break;
				}
			}
			break;
		case VSRC_CCD:
			if(vsrc.calib_en)
				vset_mask = CMD_CCD_NO_DELAY;
			else {
				switch (vsrc.zoom) {
				case NFOV:
					vset_mask = CMD_CCD_NFOV;
					break;
				case MFOV:
					vset_mask = CMD_CCD_MFOV;
					break;
				case WFOV:
					vset_mask = CMD_CCD_MFOV;
					break;
				default:
					break;
				}
			}
			break;
		default:
			vset_mask = CMD_TI_WFOV;
			break;
		}
	}
}

/******************************************
 * 					PROTOCOL			  *
 *****************************************/
void ComTurret::TxAXAX(SerialPort::DataBuffer& tx_data)
{
	uint8_t checksum = accumulate(tx_data.begin(), tx_data.end(), 0) & 0x7F;

	SerialPort::DataBuffer tx_buf = {
			(uint8_t)(0xA0|(tx_data[tx_data.size()-2])), 	// header low
			(uint8_t)(0xA0|(tx_data[tx_data.size()-1])),	// header high
			checksum, 										// checksum
			(uint8_t)(tx_data.size() | vset_mask)			// no of bytes | vset_mask
	};

	tx_buf.insert(tx_buf.end(), tx_data.begin(), tx_data.end());

	serial_port.Write(tx_buf);
}

bool ComTurret::RxAXAX(uint8_t data_recv, SerialPort::DataBuffer& data)
{
	uint8_t rx_cmdA, rx_cmdB;
	uint8_t bytes_recv;
	bool ret_val = 0;
	uint8_t chksum_recv = 0;
	{
		switch(rx_state&(0x0F)){
		case 0:											//	Reset, sync 0xAx awaited
			if((data_recv&0xF0) == 0xA0){
				if(rx_buf_count < rx_buf_size)
					rx_buf[rx_buf_count++] = data_recv;
				else {
					rx_buf_count = 0;
					rx_state &= 0xF0;					//	Reset, sync 0xAx awaited
					rx_PSyncFlag = 0;
					rx_buf[rx_buf_count++] = data_recv;
				}
				if(!rx_PSyncFlag){
					rx_PSyncFlag = 1;
				}
				else{
					rx_PSyncFlag = 0;
					rx_state  |= 0x01;				//	sync complete, chksum awaited
				}
			}
			else{
				rx_buf_count = 0;
				rx_state  &= 0xF0;						//	Reset, sync 0xAx awaited
				rx_PSyncFlag = 0;
			}
			break;
		case 1:											// Chksum awaited
			if(data_recv&0x80){
				if((data_recv&0xF0) == 0xA0){
					rx_buf[0] 		= rx_buf[--rx_buf_count];
					rx_buf[1]  		= data_recv;
					rx_buf_count 	= 2;
				}
				else{
					rx_buf_count 	 = 0;
					rx_state 		&= 0xF0;					//	Reset, sync 0xAx awaited
					rx_PSyncFlag 	 = 0;
				}
			}
			else{
				if(rx_buf_count < rx_buf_size) {
					rx_buf[rx_buf_count++] = data_recv;
					rx_state &= 0xF0;
					rx_state |= 0x02;		//	Packet size awaited
				}
				else {
					rx_buf_count = 0;
					rx_state 	&= 0xF0;						//	Reset, sync 0xAx awaited
					rx_PSyncFlag = 0;
				}
			}
			break;
		case 2:		 								//	Packet size awaited
			if(data_recv&0x80){
				if((data_recv&0xF0) == 0xA0){
					rx_buf[0] 		 = data_recv;
					rx_buf_count 	 = 1;
					rx_state 		&= 0xF0;					//	Reset, sync 0xAx awaited
					rx_PSyncFlag 	 = 1;
				}
				else{
					rx_buf_count = 0;
					rx_state 	&= 0xF0;					//	Reset, sync 0xAx awaited
					rx_PSyncFlag = 0;
				}
			}
			else{
				if(rx_buf_count < rx_buf_size) {
					rx_buf[rx_buf_count++] = data_recv;

					rx_chksum_calc   = 0;
					rx_state 		&= 0xF0;
					rx_state 		|= 0x03;						//	Data lsb awaited
				}
				else {
					rx_buf_count = 0;
					rx_state 	 &= 0xF0;					//	Reset, sync 0xAx awaited
					rx_PSyncFlag = 0;
				}
			}
			break;
		case 3:
			if(rx_buf_count < rx_buf_size) {							//	Data lsb awaited
				rx_buf[rx_buf_count++] = data_recv;
				rx_chksum_calc += data_recv;
				if((data_recv&0xF0) == 0xA0)
					rx_PSyncFlag = 1;
				rx_state &= 0xF0;
				rx_state |= 0x04;	//	Data msb awaited
			}
			else {
				rx_buf_count = 0;
				rx_state 	&= 0xF0;				//	Reset, sync 0xAx awaited
				rx_PSyncFlag = 0;
			}
			break;
		case 4:										//	Data msb awaited
			if((data_recv&0xF0) == 0xA0){
				if(rx_PSyncFlag){
					rx_buf_count--;
					rx_buf[0] 	  = rx_buf[rx_buf_count];
					rx_buf[1] 	  = data_recv;
					rx_buf_count  = 2;
					rx_state 	 &= 0xF0;
					rx_state 	 |= 0x01;
					rx_PSyncFlag  = 0;
				}
				else{
					rx_buf[0] = data_recv;
					rx_buf_count = 1;
					rx_state 	&= 0xF0;		//	Reset, PSync; sync 0xAx awaited
					rx_PSyncFlag = 1;
				}
			}
			else{
				if(rx_buf_count < rx_buf_size) {
					rx_buf[rx_buf_count++] = data_recv;
					rx_chksum_calc += data_recv;
					chksum_recv 	= (rx_buf[2]&0x7f);
					bytes_recv    	= (rx_buf[3]&0x7F);
					rx_chksum_calc &= 0x7f;

					if(rx_buf_count >= (bytes_recv+4)){
						rx_cmdA = (rx_buf[1]<<4)|(rx_buf[0]&0x0F);
						rx_cmdB = (rx_buf[bytes_recv+4-1]<<4)|(rx_buf[bytes_recv+4-2]&0x0F);
						if ( (rx_chksum_calc == chksum_recv) && (rx_cmdA == rx_cmdB)) {
							data.clear();
							data.insert(data.end(), begin(rx_buf) + 4, begin(rx_buf) + 4 + bytes_recv);
							ret_val = 1;
						}
						else {
							cerr << "Checksum failed" << endl;
						}
						rx_buf_count = 0;
						rx_state 	&= 0xF0;	//	Reset, sync 0xAx awaited
						rx_PSyncFlag = 0;
					}
					else{
						rx_state 	&= 0xF0;
						rx_state 	|= 0x03;		//	Data lsb awaited
						rx_PSyncFlag = 0;
					}
				}
				else {
					rx_buf_count = 0;
					rx_state 	&= 0xF0;					//	Reset, sync 0xAx awaited
					rx_PSyncFlag = 0;
				}
			}
			break;
		}
	}
	pkt_rcv = ret_val;
	return ret_val;
}
