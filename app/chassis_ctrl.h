#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H
#include "typedef.h"
#include "pid.h"

#define RC_VAL_MAX 660.0f
#define GIMBAL_ANGLE_DELTA_MAX 0.12f
#define CHASSIS_MAX_V 8.0f
#define CHASSIS_MAX_W 8.0f

#define CHASSIS_FILTER_VX_BETA 0.2f
#define CHASSIS_FILTER_VY_BETA 0.2f
#define CHASSIS_FILTER_VW_BETA 0.2f
#define CHASSIS_CONTROL_TIME 0.002f

#define M3508_SPEED_PID_KP 8.0f
#define M3508_SPEED_PID_KI 0.0f
#define M3508_SPEED_PID_KD 0.0f
#define M3508_SPEED_PID_OUT_MAX 16000.0f
#define M3508_SPEED_PID_IOUT_MAX 3000.0f
#define M3508_MAX_POSITION_ACCEL 1000.0f
#define M3508_MAX_NEGATIVE_ACCEL 4000.0f
#define M3508_DEADZONE 0.0f

#define MF9025_ANGLE_PID_KP 800.0f
#define MF9025_ANGLE_PID_KI 0.0f
#define MF9025_ANGLE_PID_KD 45000.0f
#define MF9025_ANGLE_PID_OUT_MAX 30000.0f
#define MF9025_ANGLE_PID_IOUT_MAX 3000.0f
#define MF9025_MAX_POSITION_ACCEL 10000.0f
#define MF9025_MAX_NEGATIVE_ACCEL 40000.0f
#define MF9025_DEADZONE 0.0f
#define MF9025_MAX_IQ 2048

#define MF9025_SPEED_PID_KP 80.0f
#define MF9025_SPEED_PID_KI 10.0f
#define MF9025_SPEED_PID_KD 30.0f

#define REDUCTION_RATIO 19.2032f // 减速比
#define WHEEL_RADIUS 0.08f // 轮子半径，单位m
#define LINEAR_TO_RPM (PI * 2.0f / WHEEL_RADIUS * REDUCTION_RATIO) // 轮子线速度到电机转子转速比例系数
#define ROOT_2 1.41421356237309504880l

typedef enum
{
    CHASSIS_RC_OFFLINE = 0,
    CHASSIS_RC = 1,
    CHASSIS_UPC = 2,
    GIMBAL_RC = 3
}CHASSIS_CTRL_STATE;

typedef struct
{
    fp32 given_chassis_v[2];
    fp32 given_chassis_w;
    fp32 given_gimbal_l_yaw;
    fp32 given_chassis_yaw;

    uint8_t ctrl;
    uint8_t mode;

    uint8_t gimbal_auto_rotate;
    uint8_t gimbal_shutdown_flag;
    uint8_t last_gimbal_shutdown_flag;

}chassis_t;

typedef enum
{
    FOLLOW_CHASSIS = 0,
    SPINNING_TOP = 1
} CHASSIS_CTRL_MODE;

typedef struct
{
    pid_t pid;
    int16_t given_speed;
} m3508_ctrl_t;
typedef struct
{
    pid_t pid;
    fp32 given_angle;
    fp32 cur_angle;
    fp32 ff_speed;
} m9025_ctrl_t;

void ctrl_data_update(void);
void motor_ctrl_update(void);
void motor_ctrl_send(void);
void motor_param_get(void);
void chassis_ctrl_init(void);

#endif
