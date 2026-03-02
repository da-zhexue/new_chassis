#ifndef DTM_H_
#define DTM_H_
#include "typedef.h"

typedef enum {
    DTM_SUCCESS = 0,
    DTM_ERR_UNKNOWN_ID,
    DTM_ERR_SIZE_MISMATCH
} DTM_Error_t;

typedef enum {
    RC_DATA = 0,
    TF_DATA,
    M9025_DATA,
    M3508_DATA,
    GIMBAL_S_DATA,
    GIMBAL_L_DATA,
    UPC_DATA,
    PARAM_DATA,
    POWER_DATA,
    DTM_DATA_COUNT        
} DTM_DataID_t;

DTM_Error_t DTM_Write(DTM_DataID_t data_id, const void *data, size_t size);
DTM_Error_t DTM_Read(DTM_DataID_t data_id, void *data, size_t size);

/*下面是不同任务需要用到的数据结构体*/
typedef struct
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s1;
    uint8_t s2;
    uint8_t sw2;
} rc_t;
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

    angle_t Chassis_angle;
    angle_t Small_Gimbal_angle;
    angle_t Big_Gimbal_angle;

} TF_t;
typedef struct
{
    uint16_t ecd;
    int16_t speed;
    int16_t current;
    int16_t temperature;
    int16_t last_ecd;
}m3508_t;
typedef struct
{
    int8_t temperate;
    int16_t iq;
    int16_t speed;
    uint16_t ecd;
    uint16_t ecd_offset;
    fp32 imu_yaw_offset;
    uint8_t get_imu_offset;
}m9025_t;
typedef struct
{
    uint8_t start_upc_flag;
    uint8_t game_start;
    uint8_t mode;

    float vx, vy, vw;
    float gimbal_yaw, chassis_yaw;
    float x, y, z;

} upc_t;

#endif
