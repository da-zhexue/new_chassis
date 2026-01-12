#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "typedef.h"
#include "BMI088driver.h"

#define X 0
#define Y 1
#define Z 2

typedef struct
{
	float roll_deg, yaw_deg, pitch_deg;
	float roll_rad, yaw_rad, pitch_rad;
	float yaw_total_angle;
	int16_t yaw_round_count;
	float yaw_angle_last;
} angle_t;

typedef struct
{
    float q[4]; 
    float Gyro[3]; 
    float Accel[3]; 

	fp32 Small_Gimbal_IMU_last_online_time;
	fp32 Big_Gimbal_IMU_last_online_time;
	angle_t Chassis_angle;
	angle_t Small_Gimbal_angle;
	angle_t Big_Gimbal_angle;

} TF_t;

void TF_Task(void const * argurment);
TF_t* get_TF(void);
#endif
