#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "typedef.h"
#include "pid.h"
#include "data_transfer.h"

#define M3508_SPEED_PID_KP 15.0f
#define M3508_SPEED_PID_KI 0.0f
#define M3508_SPEED_PID_KD 10.0f
#define M3508_SPEED_PID_OUT_MAX 16000.0f
#define M3508_SPEED_PID_IOUT_MAX 3000.0f
#define M3508_MAX_POSITION_ACCEL 10.0f
#define M3508_MAX_NEGATIVE_ACCEL 40.0f
#define M3508_DEADZONE 0.0f

#define MF9025_ANGLE_PID_KP 0.01f
#define MF9025_ANGLE_PID_KI 0.0f
#define MF9025_ANGLE_PID_KD 10.0f
#define MF9025_ANGLE_PID_OUT_MAX 30000.0f
#define MF9025_ANGLE_PID_IOUT_MAX 3000.0f
#define MF9025_MAX_POSITION_ACCEL 10000.0f
#define MF9025_MAX_NEGATIVE_ACCEL 40000.0f
#define MF9025_DEADZONE 0.0f
#define MF9025_MAX_IQ 2048

#define MF9025_ANGLE_PID_KP2 1.0f
#define MF9025_ANGLE_PID_KI2 0.0f
#define MF9025_ANGLE_PID_KD2 30.0f
#define MF9025_ANGLE_PID_KP3 150.0f
#define MF9025_ANGLE_PID_KI3 0.0f
#define MF9025_ANGLE_PID_KD3 0.0f

#define ROOT_2 1.41421356237309504880l

typedef enum 
{
		STOPPING = 0,
    FOLLOW_CHASSIS = 1,
    FOLLOW_GIMBAL = 2,
    SPINNING_TOP = 3
} CHASSIS_CTRL_MODE;

typedef struct
{
    pid_t pid;
    int16_t given_speed;
    motor_3508_measure_t* measure;
} motor_3508_ctrl_t;
typedef struct
{
    pid_t pid;
    fp32 given_angle;
    motor_9025_measure_t* measure; // 为了TF_task中单独读取编码器数据，将编码器数据通过指针独立出来
} motor_9025_ctrl_t;

void motor_ctrl_init(void);
void motor_ctrl_update(chassis_ctrl_t* chassis_ctrl);

#endif
