#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "typedef.h"
#include "pid.h"
#include "chassis_ctrl.h"  

#define M3508_SPEED_PID_KP 20.0f
#define M3508_SPEED_PID_KI 0.0f
#define M3508_SPEED_PID_KD 10.0f
#define M3508_SPEED_PID_OUT_MAX 16000.0f
#define M3508_SPEED_PID_IOUT_MAX 3000.0f
#define M3508_MAX_POSITION_ACCEL 10.0f
#define M3508_MAX_NEGATIVE_ACCEL 40.0f
#define M3508_DEADZONE 0.0f

#define MF9025_ANGLE_PID_KP 150.0f
#define MF9025_ANGLE_PID_KI 0.0f
#define MF9025_ANGLE_PID_KD 1000.0f
#define MF9025_ANGLE_PID_OUT_MAX 16000.0f
#define MF9025_ANGLE_PID_IOUT_MAX 3000.0f
#define MF9025_MAX_POSITION_ACCEL 1000.0f
#define MF9025_MAX_NEGATIVE_ACCEL 4000.0f
#define MF9025_DEADZONE 0.0f
#define MF9025_MAX_IQ 2048

#define MF9025_ANGLE_PID_KP2 800.0f
#define MF9025_ANGLE_PID_KI2 0.0f
#define MF9025_ANGLE_PID_KD2 200.0f
#define MF9025_ANGLE_PID_KP3 1500.0f
#define MF9025_ANGLE_PID_KI3 0.0f
#define MF9025_ANGLE_PID_KD3 0.0f

#define ROOT_2 1.41421356237309504880l

typedef struct 
{
   uint16_t ecd;
   int16_t speed;
   int16_t current;
   int16_t temperature;
   int16_t last_ecd;
   fp32 last_online;
}motor_3508_measure_t;

typedef struct
{
    int8_t temperate;
    int16_t iq;
    int16_t speed;
    uint16_t ecd;
    fp32 last_online;
		uint16_t ecd_offset;
}motor_9025_measure_t;

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

typedef enum 
{
		STOPPING = 0,
    FOLLOW_CHASSIS = 1,
    FOLLOW_GIMBAL = 2,
    SPINNING_TOP = 3
} CHASSIS_CTRL_MODE;


void motor_ctrl_init(void);
void motor_ctrl_update(chassis_ctrl_t* chassis_ctrl);
motor_9025_measure_t* get_motor_9025_measure_data(void);
motor_3508_measure_t* get_motor_3508_measure_data(uint8_t motor_index);

#endif
