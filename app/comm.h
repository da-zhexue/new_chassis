#ifndef UPC_COMMUNICATE_H
#define UPC_COMMUNICATE_H
#include "typedef.h"
#include "DTM.h"

#define SOF_VALUE       0xA5
#define FRAME_HEADER_LEN  5
#define CMD_ID_LEN        2
#define CRC16_LEN         2

typedef enum
{
	SEND_PARAM = 0x200,
	SEND_ATTITUDE = 0x201,
	SEND_ONLINE = 0x202,
	SEND_START = 0x203,
	SEND_ONLINECB = 0x207,
	SEND_SUPERCAP = 0x208,
	SEDN_DEBUG = 0x233,

    // CMD_IMU_S_INFO = 0x101,
    CMD_IMU_L_INFO = 0x102,
	CMD_MOVE = 0x103,
	CMD_ROTATE = 0x104,
	CMD_MODE = 0x105,
	CMD_START = 0x106,
	CMD_ONLINECB = 0x107,
	CMD_POWER = 0x108,
	CMD_GIMBAL_ROTATE = 0x109,
	CMD_DEBUG = 0x133,

} upc_cmd_t;

uint8_t upc_decode(uint8_t* rx_data);
uint8_t cmd_onlinecb_handler(uint8_t on);
void send_attitude_handler(const TF_t *tf_ptr);
void send_onlinestate_handler();
void send_start_handler();
void send_power_ctrl_param_handler();
void send_onlinecb_handler();
void send_cap_handler();

#endif
