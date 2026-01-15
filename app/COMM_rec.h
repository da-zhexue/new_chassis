#ifndef UPC_COMMUNICATE_H
#define UPC_COMMUNICATE_H
#include "typedef.h"

#define UPC_HEADER 0xA5
#define UPC_HEADER_LEN 5
#define UPC_DATA_LEN 0x0D
#define UPC_TOTAL_LEN (UPC_DATA_LEN + 9) 

typedef enum
{
		SNED_DEBUG_ID = 0x300,
		SEND_ATTITUDE_ID = 0x301,
		SNED_REFEREE_ID = 0x302,
	
    MSG_DEBUG_ID = 0x0400, // Debug message
    MSG_IMU_INFO_ID = 0x0401,
    MSG_MOVE_CMD_ID = 0x0402, // Move command
    GIMBAL_ROTATION_ID = 0x0403, // Gimbal rotation command
		MSG_SHOOT_CMD_ID = 0x0404, 
		MSG_MODE_SWITCH_ID = 0x405,

    MSG_SMALL_GIMBAL_IMU_INFO_ID = 0x101,
    MSG_BIG_GIMBAL_IMU_INFO_ID = 0x102
} upc_cmd_t;

uint8_t upc_decode(uint8_t* rx_data);
void temp_imu_handler(uint8_t* data);

#endif
