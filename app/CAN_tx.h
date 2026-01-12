#ifndef CAN_TX_H
#define CAN_TX_H

#include "typedef.h"

#define CMD_9025_SHUTDOWN 0x80
#define CMD_9025_STOP 0x81
#define CMD_9025_START 0x88

#define CMD_9025_SET_PID 0xC1
#define CMD_9025_INCREMENT_ANGLE_CONTROL 0xA8
#define CMD_9025_ANGLE_CONTROL 0xA6
#define CMD_9025_SPEED_CONTROL 0xA2
#define CMD_9025_READ_MEASURE 0x9C
#define CMD_9025_SET_ANGLE 0x95
#define CMD_9025_READ_ENCODER 0x90
#define CMD_9025_READ_CONTROL_PARAM 0xC0
#define CMD_9025_WRITE_CONTROL_PARAM 0xC1

#define MOTOR_3508_CAN hcan1
#define MOTOR_9025_CAN hcan1

#define GIMBAL_CAN hcan2

typedef enum
{	
	CAN_3508_SEND_ID = 0x200,
	
	CAN_MF_SEND_ID = 0x140,
	CAN_9025_M1_TX_ID = 0x141,
}can_tx_id_e;

void CAN_Control3508Current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_Manage9025State(uint32_t id,uint8_t CMD);
void CAN_Get9025Measure(uint32_t id, uint8_t CMD);
void CAN_Read9025CircleAngle(uint32_t id);
void CAN_Control9025CircleAngle(uint32_t id, uint8_t spinDirection, uint32_t maxSpeed, uint32_t angleControl);
void CAN_Control9025IncrementAngle(uint32_t id, uint32_t maxSpeed, int32_t angleControl);
void CAN_Control9025Speed(uint32_t id, uint16_t iqControl, uint32_t speedControl);
void CAN_CBoard_CMD(uint32_t id, uint8_t data[8]);
void CAN_Set9025PID(uint32_t id, uint8_t param, uint16_t kp, uint16_t ki, uint16_t kd);
void CAN_Set9025ZeroAngle(uint32_t id);

#endif
