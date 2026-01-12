#ifndef CAN_RX_H
#define CAN_RX_H

#include "typedef.h"

#define CMD_9025_INCREMENT_ANGLE_CONTROL 0xA8
#define CMD_9025_ANGLE_CONTROL 0xA6
#define CMD_9025_SPEED_CONTROL 0xA2
#define CMD_9025_READ_MEASURE 0x9C
#define CMD_9025_READ_ENCODER 0x90
#define CMD_9025_READ_CONTROL_PARAM 0xC0
#define CMD_9025_WRITE_CONTROL_PARAM 0xC1

#define CONTROL_PARAM_9025_ANGLE_PID 0x0A
#define CONTROL_PARAM_9025_SPEED_PID 0x0B
#define CONTROL_PARAM_9025_IQ_PID 0x0C

#define MOTOR_3508_CAN hcan1
#define MOTOR_9025_CAN hcan1

#define GIMBAL_CAN hcan2

typedef enum
{	
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203,
	CAN_3508_M4_ID = 0x204,

	CAN_9025_M1_RX_ID = 0x141,
	
	CBOARD_GIMBAL_1 = 0x333,
	CBOARD_GIMBAL_2 = 0x334,
}can_rx_id_e;

void CAN_Receive_Init(void);
float* get_small_gimbal_yaw(void);
float* get_big_gimbal_yaw(void);

#endif
