#ifndef ONLINE_MONITOR_H
#define ONLINE_MONITOR_H

#include "typedef.h"

typedef struct 
{
    uint8_t rc;
    uint8_t upc;
    uint8_t big_gimbal_imu;
    uint8_t small_gimbal_imu;
    uint8_t chassis_motor[4];
    uint8_t gimbal_motor;
    uint8_t referee;
}online_monitor_t;

typedef enum
{
    RC_ONLINE = 0,
    UPC_ONLINE = 1,
    BIG_GIMBAL_IMU_ONLINE = 2,
    SMALL_GIMBAL_IMU_ONLINE = 3,
    CHASSIS_MOTOR_0_ONLINE = 4,
    CHASSIS_MOTOR_1_ONLINE = 5,
    CHASSIS_MOTOR_2_ONLINE = 6,
    CHASSIS_MOTOR_3_ONLINE = 7,
    GIMBAL_MOTOR_ONLINE = 8,
    REFEREE_ONLINE = 9
} online_online_type;

uint8_t Online_Monitors(fp32 last_online, uint16_t type);
uint16_t error_code_get(void);

#endif
