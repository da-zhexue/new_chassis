#ifndef UPC_COMMUNICATE_H
#define UPC_COMMUNICATE_H
#include "typedef.h"

#define SOF_VALUE       0xA5
#define FRAME_HEADER_LEN  5
#define CMD_ID_LEN        2
#define CRC16_LEN         2

typedef enum
{
	SEND_ATTITUDE = 0x201,
	SEND_ONLINE = 0x202,
	SEND_START = 0x203,

    CMD_IMU_S_INFO = 0x101,
    CMD_IMU_L_INFO = 0x102,
	CMD_MOVE = 0x103,
	CMD_ROTATE = 0x104,
	CMD_MODE = 0x105,
	CMD_START = 0x106
} upc_cmd_t;

uint8_t upc_decode(uint8_t* rx_data);

#endif
