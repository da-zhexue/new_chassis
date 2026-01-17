/**
 * @file data_transfer.c
 * @brief 数据中转模块
 * 该文件用于保存需要用于多个任务的数据，并通过返回指针的方式提供给其他模块使用。
 * 注: 使用返回指针的方式仅因为个人不喜欢extern一个全局变量。
 * @version 1.0
 * @date 2026-01-17
 */

#include "data_transfer.h"

#ifdef DEBUG_WITH_GLOBAL_VAR
rc_ctrl_t RC_CtrlData;
TF_t TF;
motor_9025_measure_t motor_9025_measure;
motor_3508_measure_t motor_3508_measure[4];
small_gimbal_angle_t small_gimbal_angle_deg;
big_gimbal_angle_t big_gimbal_angle_deg;
upc_t upc;
#else
static rc_ctrl_t RC_CtrlData;
static TF_t TF;
static motor_9025_measure_t motor_9025_measure;
static motor_3508_measure_t motor_3508_measure[4];
static small_gimbal_angle_t small_gimbal_angle_deg;
static big_gimbal_angle_t big_gimbal_angle_deg;
static upc_t upc;
#endif

rc_ctrl_t *get_rc_ctrl_data(void)
{
	return &RC_CtrlData;
}


TF_t* get_TF(void)
{
	return &TF;
}

motor_9025_measure_t* get_motor_9025_measure_data(void)
{
    return &motor_9025_measure;
}

motor_3508_measure_t* get_motor_3508_measure_data(uint8_t motor_index)
{
    if(motor_index < 4)
        return &motor_3508_measure[motor_index];
    return NULL;
}

small_gimbal_angle_t* get_small_gimbal_angle(void)
{
	return &small_gimbal_angle_deg;
}

big_gimbal_angle_t* get_big_gimbal_angle(void)
{
	return &big_gimbal_angle_deg;
}

upc_t* get_upc_data(void)
{
	return &upc;
}
