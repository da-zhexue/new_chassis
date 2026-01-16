#ifndef DATA_TRANSFER_H
#define DATA_TRANSFER_H
#include "typedef.h"
#define DEBUG_WITH_GLOBAL_VAR

typedef struct __RC__
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s1;
    uint8_t s2;
    uint8_t sw2;
	
	fp32 last_online_time;
} rc_ctrl_t;
rc_ctrl_t *get_rc_ctrl_data(void);

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

	fp32 small_gimbal_imu_last_online_time;
	fp32 big_gimbal_imu_last_online_time;

	angle_t Chassis_angle;
	angle_t Small_Gimbal_angle;
	angle_t Big_Gimbal_angle;

} TF_t;
TF_t* get_TF(void);

typedef struct
{
    fp32 given_chassis_v[2];
    fp32 given_chassis_w;
    fp32 given_gimbal_yaw;
    fp32 given_chassis_yaw;

    uint8_t ctrl;
    uint8_t mode;

    uint8_t gimbal_shutdown_flag;
    uint8_t last_gimbal_shutdown_flag;

}chassis_ctrl_t;
chassis_ctrl_t* get_chassis_ctrl_data(void);


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
		fp32 imu_yaw_offset;
}motor_9025_measure_t;
motor_9025_measure_t* get_motor_9025_measure_data(void);
motor_3508_measure_t* get_motor_3508_measure_data(uint8_t motor_index);

typedef struct 
{
    uint8_t start_upc_flag;
		uint8_t mode;
		uint8_t shoot;

    float vx, vy, vw; 
    float gimbal_yaw, chassis_yaw;
		float small_gimbal_yaw, small_gimbal_pitch;
    float x, y, z;

} upc_t;
typedef struct
{
    fp32 big_gimbal_angle[3];
    fp32 big_gimbal_imu_last_online_time;
} big_gimbal_angle_t; // 暂时将大小云台拆为两个结构体
big_gimbal_angle_t* get_big_gimbal_angle(void);
typedef struct
{
    fp32 small_gimbal_angle[3];
    fp32 small_gimbal_imu_last_online_time;
} small_gimbal_angle_t; //
small_gimbal_angle_t* get_small_gimbal_angle(void);
upc_t* get_upc_data(void);

#endif
