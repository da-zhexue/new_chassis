#include "can.h"
#include "CAN_tx.h"

CAN_TxHeaderTypeDef  motor_3508_tx_message;
uint8_t              motor_3508_can_tx_data[8];

CAN_TxHeaderTypeDef  motor_6020_tx_message;
uint8_t              motor_6020_can_tx_data[8];

CAN_TxHeaderTypeDef  motor_2006_tx_message;
uint8_t              motor_2006_can_tx_data[8];

CAN_TxHeaderTypeDef  motor_9025_tx_message;
uint8_t              motor_9025_can_tx_data[8];

CAN_TxHeaderTypeDef  gimbal_tx_message;
uint8_t              gimbal_can_tx_data[8];

//3508motor
void CAN_Control3508Current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    motor_3508_tx_message.StdId = CAN_3508_SEND_ID;
    motor_3508_tx_message.IDE = CAN_ID_STD;
    motor_3508_tx_message.RTR = CAN_RTR_DATA;
    motor_3508_tx_message.DLC = 0x08;
    motor_3508_can_tx_data[0] = motor1 >> 8;
    motor_3508_can_tx_data[1] = motor1;
    motor_3508_can_tx_data[2] = motor2 >> 8;
    motor_3508_can_tx_data[3] = motor2;
    motor_3508_can_tx_data[4] = motor3 >> 8;
    motor_3508_can_tx_data[5] = motor3;
    motor_3508_can_tx_data[6] = motor4 >> 8;
    motor_3508_can_tx_data[7] = motor4;

    HAL_CAN_AddTxMessage(&MOTOR_3508_CAN, &motor_3508_tx_message, motor_3508_can_tx_data, &send_mail_box);
}

//注意：MF9025电机需要通过拨码打开接入终端电阻，且在广播模式下canid与协议不同，需在上位机中关闭广播模式!!!
//Note: The MF9025 motor requires the access terminal resistor to be turned on through dip switches. 
//Additionally, in broadcast mode, the canid and protocol are different, and the broadcast mode needs to be disabled in the host computer!!!
//9025motor
//control state of 9025: CMD_9025_SHUTDOWN, CMD_9025_STOP, CMD_9025_START
void CAN_Manage9025State(uint32_t id,uint8_t CMD)
{
	uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = CMD;//CMD_9025_SHUTDOWN CMD_9025_STOP CMD_9025_START
    motor_9025_can_tx_data[1] = 0x00;
    motor_9025_can_tx_data[2] = 0x00;
    motor_9025_can_tx_data[3] = 0x00;
    motor_9025_can_tx_data[4] = 0x00;
    motor_9025_can_tx_data[5] = 0x00;
    motor_9025_can_tx_data[6] = 0x00;
    motor_9025_can_tx_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);
}

void CAN_Read9025CircleAngle(uint32_t id)
{
	uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = 0x94;
    motor_9025_can_tx_data[1] = 0x00;
    motor_9025_can_tx_data[2] = 0x00;
    motor_9025_can_tx_data[3] = 0x00;
    motor_9025_can_tx_data[4] = 0x00;
    motor_9025_can_tx_data[5] = 0x00;
    motor_9025_can_tx_data[6] = 0x00;
    motor_9025_can_tx_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);
}

// CMD_9025_READ_ENCODER CMD_9025_READ_MEASURE CMD_9025_READ_CONTROL_PARAM
void CAN_Get9025Measure(uint32_t id, uint8_t CMD)
{
    uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = CMD;
    motor_9025_can_tx_data[1] = 0x00;
    motor_9025_can_tx_data[2] = 0x00;
    motor_9025_can_tx_data[3] = 0x00;
    motor_9025_can_tx_data[4] = 0x00;
    motor_9025_can_tx_data[5] = 0x00;
    motor_9025_can_tx_data[6] = 0x00;
    motor_9025_can_tx_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);
}

void CAN_Control9025Speed(uint32_t id, uint16_t iqControl, uint32_t speedControl)
{
		uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = CMD_9025_SPEED_CONTROL;
    motor_9025_can_tx_data[1] = 0x00;
    motor_9025_can_tx_data[2] = ((uint8_t *)&iqControl)[0]; 
    motor_9025_can_tx_data[3] = ((uint8_t *)&iqControl)[1]; 
    motor_9025_can_tx_data[4] = ((uint8_t *)&speedControl)[0]; 
    motor_9025_can_tx_data[5] = ((uint8_t *)&speedControl)[1];
    motor_9025_can_tx_data[6] = ((uint8_t *)&speedControl)[2];
    motor_9025_can_tx_data[7] = ((uint8_t *)&speedControl)[3];
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);	
}

void CAN_Control9025CircleAngle(uint32_t id, uint8_t spinDirection, uint32_t maxSpeed, uint32_t angleControl)
{
    uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = CMD_9025_INCREMENT_ANGLE_CONTROL;
    motor_9025_can_tx_data[1] = spinDirection;
    motor_9025_can_tx_data[2] = ((uint8_t *)&maxSpeed)[0]; // 访问 maxSpeed 的第一个字节
    motor_9025_can_tx_data[3] = ((uint8_t *)&maxSpeed)[1]; // 访问 maxSpeed 的第二个字节
    motor_9025_can_tx_data[4] = ((uint8_t *)&angleControl)[0]; // 访问 angleControl 的第一个字节
    motor_9025_can_tx_data[5] = ((uint8_t *)&angleControl)[1]; // 访问 angleControl 的第二个字节
    motor_9025_can_tx_data[6] = ((uint8_t *)&angleControl)[2];
    motor_9025_can_tx_data[7] = ((uint8_t *)&angleControl)[3];
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);	
}

void CAN_Control9025IncrementAngle(uint32_t id, uint32_t maxSpeed, int32_t angleControl)//TODO MAX SPEED确定
{
	uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = CMD_9025_INCREMENT_ANGLE_CONTROL;
    motor_9025_can_tx_data[1] = 0x00;
    motor_9025_can_tx_data[2] = ((uint8_t *)&maxSpeed)[0];
    motor_9025_can_tx_data[3] = ((uint8_t *)&maxSpeed)[1];
    motor_9025_can_tx_data[4] = * (uint8_t*)(&angleControl);
    motor_9025_can_tx_data[5] = *((uint8_t*)(&angleControl)+1);
    motor_9025_can_tx_data[6] = *((uint8_t*)(&angleControl)+2);
    motor_9025_can_tx_data[7] = *((uint8_t*)(&angleControl)+3);
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);
}

// param: CONTROL_PARAM_9025_ANGLE_PID CONTROL_PARAM_9025_SPEED_PID CONTROL_PARAM_9025_IQ_PID
void CAN_Set9025PID(uint32_t id, uint8_t param, uint16_t kp, uint16_t ki, uint16_t kd) 
{
	uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = CMD_9025_SET_PID;
    motor_9025_can_tx_data[1] = param;
    motor_9025_can_tx_data[2] = ((uint8_t *)&kp)[0];
    motor_9025_can_tx_data[3] = ((uint8_t *)&kp)[1];
    motor_9025_can_tx_data[4] = * (uint8_t*)(&ki);
    motor_9025_can_tx_data[5] = *((uint8_t*)(&ki)+1);
    motor_9025_can_tx_data[6] = *((uint8_t*)(&kd)+0);
    motor_9025_can_tx_data[7] = *((uint8_t*)(&kd)+1);
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);
}

void CAN_Set9025ZeroAngle(uint32_t id)
{
	uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = CMD_9025_SET_ANGLE;
    motor_9025_can_tx_data[1] = 0;
    motor_9025_can_tx_data[2] = 0;
    motor_9025_can_tx_data[3] = 0;
    motor_9025_can_tx_data[4] = 0;
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);
}

void CAN_CBoard_CMD(uint32_t id, uint8_t data[8])
{
	uint32_t send_mail_box;
    gimbal_tx_message.StdId = id;
    gimbal_tx_message.IDE   = CAN_ID_STD;
    gimbal_tx_message.RTR   = CAN_RTR_DATA;
    gimbal_tx_message.DLC   = 0x08;

    gimbal_can_tx_data[0] = data[0];
    gimbal_can_tx_data[1] = data[1];
    gimbal_can_tx_data[2] = data[2]; 
    gimbal_can_tx_data[3] = data[3]; 
    gimbal_can_tx_data[4] = data[4]; 
    gimbal_can_tx_data[5] = data[5];
    gimbal_can_tx_data[6] = data[6];
    gimbal_can_tx_data[7] = data[7];
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_tx_data, &send_mail_box);	
}
