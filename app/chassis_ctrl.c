/**
 * @file chassis_ctrl.c
 * @brief 底盘控制模块
 * 处理遥控器或上位机控制数据转化为底盘控制数据。
 * @version 2.0
 * @changelog
 * - 2026-01-16 增加遥控器控制云台
 * - 2026-01-17 增加速度滤波
 * - 2026-02-09 合并chassis_ctrl和motor_ctrl，将云台控制统一至此
 */

#include "chassis_ctrl.h"
#include "pid.h"
#include "math.h"
#include "user_lib.h"
#include "OMM.h"
#include "CAN_tx.h"

#ifdef DEBUG_MODE
chassis_t chassis_ptr;
m3508_ctrl_t m3508_ctrl[4];
m9025_ctrl_t m9025_ctrl;
#else
static chassis_t chassis_ptr;
static m3508_ctrl_t m3508_ctrl[4];
static m9025_ctrl_t m9025_ctrl;
#endif

static first_order_filter_type_t chassis_vx_filter, chassis_vy_filter, chassis_vw_filter;
static float filter_vx_num[1] = {CHASSIS_FILTER_VX_BETA};
static float filter_vy_num[1] = {CHASSIS_FILTER_VY_BETA};
static float filter_vw_num[1] = {CHASSIS_FILTER_VW_BETA};

static pid_t follow_gimbal_pid;

static fp32 M3508_SPEED_PID[3] = {M3508_SPEED_PID_KP, M3508_SPEED_PID_KI, M3508_SPEED_PID_KD};
static fp32 MF9025_ANGLE_PID[3] = {MF9025_ANGLE_PID_KP, MF9025_ANGLE_PID_KI, MF9025_ANGLE_PID_KD};
static fp32 MF9025_ANGLE_MULTI_PID [3][4] = {
    {5,MF9025_ANGLE_PID_KP, MF9025_ANGLE_PID_KI, MF9025_ANGLE_PID_KD},
    {180,MF9025_ANGLE_PID_KP2, MF9025_ANGLE_PID_KI2, MF9025_ANGLE_PID_KD2},
    {14400,MF9025_ANGLE_PID_KP3, MF9025_ANGLE_PID_KI3, MF9025_ANGLE_PID_KD3}
};
static fp32 FOLLOW_GIMBAL_PID[3] = {FOLLOW_GIMBAL_PID_KP, FOLLOW_GIMBAL_PID_KI, FOLLOW_GIMBAL_PID_KD};

void ctrl_data_update(void)
{
    static rc_t rc_ptr;
    static upc_t upc_ptr;

    DTM_Read(RC_DATA, &rc_ptr, sizeof(rc_t));
    DTM_Read(UPC_DATA, &upc_ptr, sizeof(upc_t));

    if(OMM_detect(RC_ONLINE))
        chassis_ptr.ctrl = rc_ptr.s1;
    else
        chassis_ptr.ctrl = CHASSIS_RC_OFFLINE;

    static fp32 vx, vy, vw, yaw_delta;
    static fp32 vx_filter, vy_filter, vw_filter;
    static fp32 norm_v;
    if(chassis_ptr.ctrl == CHASSIS_RC)
    {
        chassis_ptr.mode = 1;//rc_ptr.s2;
        upc_ptr.start_upc_flag = 0;
        vx = (fp32)rc_ptr.ch3;
        vy = (fp32)rc_ptr.ch2;
        vw = (fp32)(rc_ptr.ch1);

        chassis_ptr.gimbal_shutdown_flag = 0;
        yaw_delta = (fp32)rc_ptr.ch0 * GIMBAL_ANGLE_DELTA_MAX / 660.0f;
        chassis_ptr.given_gimbal_l_yaw += yaw_delta;
    }
    else if(chassis_ptr.ctrl == CHASSIS_UPC)
    {
        upc_ptr.start_upc_flag = 1;
        if(OMM_detect(UPC_ONLINE))
        {
            chassis_ptr.mode = 1;//upc_ptr.mode;
           
            vx = upc_ptr.vx * 300;
            vy = upc_ptr.vy * 300;
            vw = upc_ptr.vw * 300;

            chassis_ptr.given_gimbal_s_yaw = upc_ptr.small_gimbal_yaw;
            chassis_ptr.given_gimbal_s_pitch = upc_ptr.small_gimbal_pitch;

            chassis_ptr.gimbal_shutdown_flag = 0;
            chassis_ptr.given_gimbal_l_yaw = upc_ptr.gimbal_yaw;
        }
        else
        {
            chassis_ptr.mode = 0;
            vx = 0.0f;
            vy = 0.0f;
            vw = 0.0f;
            chassis_ptr.gimbal_shutdown_flag = 1;
        }
    }
    else if(chassis_ptr.ctrl == GIMBAL_RC)
    {
        chassis_ptr.mode = 1;//rc_ptr.s2;
        upc_ptr.start_upc_flag = 0;

        chassis_ptr.given_gimbal_s_pitch -= ((fp32)rc_ptr.ch3 * 0.0001f);
        chassis_ptr.given_gimbal_s_yaw -= ((fp32)rc_ptr.ch2 * 0.0001f);

        float_constrain(&chassis_ptr.given_gimbal_s_pitch, -45.0f, 45.0f);
        float_constrain(&chassis_ptr.given_gimbal_s_yaw, -45.0f, 45.0f);
    }
    else
    {
		chassis_ptr.mode = 0;
        upc_ptr.start_upc_flag = 0;
        vx = 0.0f;
        vy = 0.0f;
        vw = 0.0f;
        chassis_ptr.gimbal_shutdown_flag = 1;
    }

    first_order_filter_cali(&chassis_vx_filter, vx);
    first_order_filter_cali(&chassis_vy_filter, vy);
    first_order_filter_cali(&chassis_vw_filter, vw);
    vx_filter = chassis_vx_filter.out;
    vy_filter = chassis_vy_filter.out;
    vw_filter = chassis_vw_filter.out;

    norm_v = (sqrt(vx_filter * vx_filter + vy_filter * vy_filter) / 660.0f) > 1.0f ? 1.0f : sqrtf(vx_filter * vx_filter + vy_filter * vy_filter) / 660.0f;
    chassis_ptr.given_chassis_v[0] = norm_v * CHASSIS_MAX_V;
    chassis_ptr.given_chassis_v[1] = atan2f(vy_filter, vx_filter);
    chassis_ptr.given_chassis_w = vw_filter * CHASSIS_MAX_W;
}

void motor_ctrl_update(void)
{
    static TF_t tf_ptr;
    DTM_Read(TF_DATA, &tf_ptr, sizeof(tf_ptr));
    const fp32 chassis_v = chassis_ptr.given_chassis_v[0];
    const fp32 theta = chassis_ptr.given_chassis_v[1];
    fp32 chassis_w = chassis_ptr.given_chassis_w + tf_ptr.Chassis_angle.yaw_rad;
    m9025_ctrl.cur_angle = -tf_ptr.Big_Gimbal_angle.yaw_total_angle;
    m9025_ctrl.ff_speed = (5729.5779513f * tf_ptr.Gyro[2]);
    switch(chassis_ptr.mode)
    {
        case SPINNING_TOP:
            chassis_w = 4000.0f; // case穿透，将转速设为定值后继续执行跟随底盘模式逻辑
		    // Case penetration, set the speed to a constant value and continue to execute the logic of FOLLOW_CHASSIS mode
        case FOLLOW_CHASSIS:
        {
            const fp32 sin_yaw = sinf(radian_format(tf_ptr.Chassis_angle.yaw_rad - tf_ptr.Small_Gimbal_angle.yaw_rad));
            const fp32 cos_yaw = cosf(radian_format(tf_ptr.Chassis_angle.yaw_rad - tf_ptr.Small_Gimbal_angle.yaw_rad));
            const fp32 vx = chassis_v * cosf(theta);
			const fp32 vy = chassis_v * sinf(theta);
            const fp32 vx_set = vx * cos_yaw - vy * sin_yaw;
			const fp32 vy_set = vx * sin_yaw + vy * cos_yaw;

            m3508_ctrl[0].given_speed = (int16_t)((vx_set + vy_set) / ROOT_2 + chassis_w);
            m3508_ctrl[1].given_speed = (int16_t)((-vx_set + vy_set) / ROOT_2 + chassis_w);
            m3508_ctrl[2].given_speed = (int16_t)((-vx_set - vy_set) / ROOT_2 + chassis_w);
            m3508_ctrl[3].given_speed = (int16_t)((vx_set - vy_set) / ROOT_2 + chassis_w);

            m9025_ctrl.given_angle = chassis_ptr.given_gimbal_l_yaw;
            break;
        }
        case FOLLOW_GIMBAL:
            m9025_ctrl.given_angle = chassis_ptr.given_gimbal_l_yaw;
            chassis_w = PID_calc(&follow_gimbal_pid, -tf_ptr.Chassis_angle.yaw_total_angle, chassis_ptr.given_gimbal_l_yaw);
            m3508_ctrl[0].given_speed = (int16_t)chassis_w;
            m3508_ctrl[1].given_speed = (int16_t)chassis_w;
            m3508_ctrl[2].given_speed = (int16_t)chassis_w;
            m3508_ctrl[3].given_speed = (int16_t)chassis_w;
            break;
        case STOPPING:
        default:
			m3508_ctrl[0].given_speed = 0;
            m3508_ctrl[1].given_speed = 0;
            m3508_ctrl[2].given_speed = 0;
            m3508_ctrl[3].given_speed = 0;

            break;
    }
}

void motor_ctrl_send(void)
{
    static m3508_t m3508_ptr[4];
    DTM_Read(M3508_DATA, m3508_ptr, sizeof(m3508_ptr));
    for(int i = 0; i < 4; i++){
        if(!OMM_detect(M3508_0_ONLINE + i))
            for(int j = 0; j < 4; j++)
                m3508_ctrl[j].given_speed = 0;
    }
    OMM_detect(M9025_ONLINE);
    if((!OMM_detect(GIMBAL_L_ONLINE)))// || motor_9025_ctrl.measure->ecd_offset == 0)
        chassis_ptr.gimbal_shutdown_flag = 1;
    for(int i = 0; i < 4; i++){
        PID_calc(&m3508_ctrl[i].pid, m3508_ptr[i].speed, m3508_ctrl[i].given_speed);
    }
    CAN_Control3508Current((int16_t)*m3508_ctrl[0].pid.out, (int16_t)*m3508_ctrl[1].pid.out, (int16_t)*m3508_ctrl[2].pid.out, (int16_t)*m3508_ctrl[3].pid.out);
    // 其实就是取pid.out[0]
    if(!chassis_ptr.gimbal_shutdown_flag)
    {
        PID_calc(&m9025_ctrl.pid, m9025_ctrl.cur_angle, m9025_ctrl.given_angle);
        CAN_Control9025Speed(CAN_9025_M1_TX_ID, MF9025_MAX_IQ, (int32_t)(*m9025_ctrl.pid.out + m9025_ctrl.ff_speed)); // 前馈补偿 底盘yaw轴角速度
    }
    else
        CAN_Control9025Speed(CAN_9025_M1_TX_ID, MF9025_MAX_IQ, 0);

    uint8_t send_data[8];
    pack_float_to_4bytes(chassis_ptr.given_gimbal_s_yaw, &send_data[0]);
    pack_float_to_4bytes(chassis_ptr.given_gimbal_s_pitch, &send_data[4]);
    CAN_CBoard_CMD(0x222, send_data);
}

void chassis_ctrl_init(void)
{
    chassis_ptr.given_chassis_v[0] = 0.0f;
    chassis_ptr.given_chassis_v[1] = 0.0f;
    chassis_ptr.given_chassis_w = 0.0f;
    chassis_ptr.ctrl = CHASSIS_RC_OFFLINE;
    chassis_ptr.mode = 1;
    chassis_ptr.gimbal_shutdown_flag = 1;
    chassis_ptr.last_gimbal_shutdown_flag = 1;

    first_order_filter_init(&chassis_vx_filter, CHASSIS_CONTROL_TIME, filter_vx_num);
    first_order_filter_init(&chassis_vy_filter, CHASSIS_CONTROL_TIME, filter_vy_num);
    first_order_filter_init(&chassis_vw_filter, CHASSIS_CONTROL_TIME, filter_vw_num);


    #ifdef DEBUG_MODE
    static uint8_t debug_ready = 0;
    static fp32 debug_param[3];
    while(!debug_ready)
    {
        DTM_Read(DEBUG_READY, &debug_ready, sizeof(debug_ready));
        osDelay(2);
    }
    DTM_Read(PARAM_DATA, debug_param, sizeof(debug_param));
    M3508_SPEED_PID[0] = debug_param[0];
    M3508_SPEED_PID[1] = debug_param[1];
    M3508_SPEED_PID[2] = debug_param[2];
    #endif
    for(int i = 0; i < 4; i++)
    {
        PID_init(&m3508_ctrl[i].pid, PID_POSITION, M3508_SPEED_PID, M3508_SPEED_PID_OUT_MAX, M3508_SPEED_PID_IOUT_MAX,
          M3508_MAX_POSITION_ACCEL, M3508_MAX_NEGATIVE_ACCEL, M3508_DEADZONE);
    }
    PID_init(&m9025_ctrl.pid, PID_POSITION, MF9025_ANGLE_PID, MF9025_ANGLE_PID_OUT_MAX, MF9025_ANGLE_PID_IOUT_MAX,
      MF9025_MAX_POSITION_ACCEL, MF9025_MAX_NEGATIVE_ACCEL, MF9025_DEADZONE);
    fp32 (*multi_Kpid_ptr)[4] = MF9025_ANGLE_MULTI_PID;
    PID_multi_Kp_init(&m9025_ctrl.pid,multi_Kpid_ptr,3);
    PID_init(&follow_gimbal_pid, PID_POSITION, FOLLOW_GIMBAL_PID, FOLLOW_GIMBAL_PID_OUT_MAX, FOLLOW_GIMBAL_PID_IOUT_MAX,
        FOLLOW_GIMBAL_MAX_POSITION_ACCEL, FOLLOW_GIMBAL_MAX_NEGATIVE_ACCEL, FOLLOW_GIMBAL_DEADZONE);

}
