#ifndef UPC_COMMUNICATE_H
#define UPC_COMMUNICATE_H
#include "typedef.h"

#define UPC_HEADER 0xA5
#define UPC_HEADER_LEN 5
#define UPC_DATA_LEN 0x0D
#define UPC_TOTAL_LEN (UPC_DATA_LEN + 9) 

typedef enum
{
	SEND_ATTITUDE = 0x301,
	SNED_REFEREE = 0x302,

    CMD_IMU_INFO = 0x0401, // 弃用
    CMD_MOVE = 0x0402, // Move command
    CMD_GIMBAL_ROTATION = 0x0403, // Gimbal rotation command
	CMD_SHOOT = 0x0404,
	CMD_MODE_SWITCH = 0x405,

    CMD_IMU_S_INFO = 0x101,
    CMD_IMU_L_INFO = 0x102
} upc_cmd_t;

uint8_t upc_decode(uint8_t* rx_data);
void cmd_imu_s_handler(const uint8_t* data);

#endif
