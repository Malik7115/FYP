 /*
 * ComMotorControl.h
 *
 *  Created on: 3 Jan 2019
 *      Author: sdc
 */

#ifndef COMTURRET_H_
#define COMTURRET_H_

#include <SerialPort.h>
#include <mutex>
#include "ComGen.hpp"
//#include "ComConsole.hpp"

using namespace std;

// always set
#define CMD_SOT_OPERATIONAL_MODE	0x03

// lrf discharge prohibit
#define CMD_LRF_DISCHARGE_PROHIBIT	0x20	// SET = NMBR | CMD, CLEAR = NMBR & ~CMD

// lrf mode should be one of the following mode
#define CMD_LRF_RANGING_INACTIVE	0x00
#define CMD_LRF_SINGLE_PULSE		0x04
#define CMD_LRF_ONE_PPS				0x10
#define CMD_LRF_THREE_PPS			0x14

// to be filled in higher nibble of transmit no of bytes (Some weird logic :/)
#define CMD_TI_WFOV						0x00
#define CMD_TI_MFOV						0x10
#define CMD_TI_NFOV						0x20
#define CMD_TI_NO_DELAY					0x30
#define CMD_CCD_WFOV					0x40
#define CMD_CCD_MFOV					0x50
#define CMD_CCD_NFOV					0x60
#define CMD_CCD_NO_DELAY				0x70

enum TURRET_MODES {
	TM_CALIBRATION = 0,
	TM_TRACKING,
	TM_LRF_EN,
	TM_FAULT
};

// needs to be updated
enum LRF_MODES {
	LRF_INACTIVE = 0,
	LRF_SINGLE_PULSE, //static
	LRF_1PPS,//surface
	LEF_3PPS//aerial
};

enum VID_SRC {
	VSRC_TI = 0,
	VSRC_CCD
};

enum VID_ZOOM {
	WFOV = 0,
	MFOV,
	NFOV
};

struct lrf_settings_struct {
	uint8_t active;
	uint8_t mode;
};

struct lrf_data_struct {
	int range1;
	int range2;
	int range3;
};

struct vid_settings_struct {
	char src;		// TI/CCD
	char zoom;		// Narrow, Medium, Wide
	char calib_en;
};


class ComTurret {
private:
	SerialPort serial_port;
	uint8_t rx_buf[100];
	int rx_buf_count;
	int rx_buf_size;
	int rx_chksum_calc;
	int rx_state;
	bool rx_PSyncFlag;

	vid_settings_struct vsrc;

	// to be transmitted to turret
	float ref_azim;
	float ref_elev; 		// to be filled by console / video processor
	uint8_t ref_cmd1, ref_cmd2;
	uint8_t vset_mask;		// video settings mask
	lrf_settings_struct lrf_set;

	// to be received from turret
	uint8_t mode;
	float azim;
	float elev;

	// comm status
	int status;

	// for thread safety
	mutex mtx;

	void GenerateData(SerialPort::DataBuffer&);
	void TxAXAX(SerialPort::DataBuffer&);

	bool RxAXAX(uint8_t data_recv, SerialPort::DataBuffer& data);
	void ExtractData(SerialPort::DataBuffer&);

public:
	lrf_data_struct lrf;
	bool pkt_rcv;

	ComTurret(const char* dev_name = "/dev/ttyS0");
	~ComTurret();

	void Transmit( void );
	void Receive(void);

	void SetLRFMode(lrf_settings_struct lmode);
	void SetVidSrc(vid_settings_struct vset);
	void SetRefPos(float azim, float elev);

	int GetStatus(void);
	pair<float, float> GetRefPos(void);
	float GetAzim(void);
	float GetElev(void);
};

#endif /* COMTURRET_H_ */
