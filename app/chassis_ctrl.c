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
#include "bsp_dwt.h"
#include "pid.h"
#include "math.h"
#include "user_lib.h"
#include "OMM.h"
#include "DTM.h"
#include "CAN_tx.h"
#include "ulog.h"
#include "bsp_rng.h"
#include "CAN_rx.h"
#include "power_ctrl.h"
#include "power_ctrl_param_get.h"

#ifdef DEBUG_MODE
chassis_t chassis_ptr;
m3508_ctrl_t m3508_ctrl[4];
m9025_ctrl_t m9025_ctrl;

#else
static chassis_t chassis_ptr;
static m3508_ctrl_t m3508_ctrl[4];
static m9025_ctrl_t m9025_ctrl;
#endif

int32_t mf9025_given_speed;
static PowerControllerConfig power_ctrl_config;
PowerControlParam ctx;

static first_order_filter_type_t chassis_vx_filter, chassis_vy_filter, chassis_vw_filter;
static float filter_vx_num[1] = {CHASSIS_FILTER_VX_BETA};
static float filter_vy_num[1] = {CHASSIS_FILTER_VY_BETA};
static float filter_vw_num[1] = {CHASSIS_FILTER_VW_BETA};

static fp32 M3508_SPEED_PID[3] = {M3508_SPEED_PID_KP, M3508_SPEED_PID_KI, M3508_SPEED_PID_KD};
static fp32 MF9025_ANGLE_PID[3] = {MF9025_ANGLE_PID_KP, MF9025_ANGLE_PID_KI, MF9025_ANGLE_PID_KD};
static uint16_t MF9025_SPEED_PID[3] = {MF9025_SPEED_PID_KP, MF9025_SPEED_PID_KI, MF9025_SPEED_PID_KD};

void ctrl_data_update(void)
{
    static rc_t rc_ptr;
    static upc_t upc_ptr;

    DTM_Read(RC_DATA, &rc_ptr, sizeof(rc_t));
    DTM_Read(UPC_DATA, &upc_ptr, sizeof(upc_t));
    //if (!upc_ptr.game_start) return;
    if(OMM_detect(RC_ONLINE))
        chassis_ptr.ctrl = rc_ptr.s1;
    else
        chassis_ptr.ctrl = CHASSIS_RC_OFFLINE;

    static fp32 vx, vy, vw, yaw_delta;
    static fp32 vx_filter, vy_filter, vw_filter;
    static fp32 norm_v;
    if(chassis_ptr.ctrl == CHASSIS_RC)
    {
        chassis_ptr.mode = 0;
        upc_ptr.start_upc_flag = 0;
        vx = (fp32)rc_ptr.ch3 * CHASSIS_MAX_V / RC_VAL_MAX;
        vy = (fp32)rc_ptr.ch2 * CHASSIS_MAX_V / RC_VAL_MAX;
        vw = (fp32)rc_ptr.ch1 * CHASSIS_MAX_W / RC_VAL_MAX;

        chassis_ptr.gimbal_shutdown_flag = 0;
        yaw_delta = (fp32)rc_ptr.ch0 * GIMBAL_ANGLE_DELTA_MAX / RC_VAL_MAX;
        chassis_ptr.given_gimbal_l_yaw += yaw_delta;
    }
    else if(chassis_ptr.ctrl == CHASSIS_UPC) // @TODO: 理论上需要做不同控制模式切换时数据不突变，但哨兵应该用不到，不想做了qwq
    {
        upc_ptr.start_upc_flag = 1;
        // if(OMM_detect(UPC_ONLINE))
        // {
            chassis_ptr.mode = upc_ptr.mode;
           
            vx = loop_float_constrain(upc_ptr.vx, -CHASSIS_MAX_V, CHASSIS_MAX_V);
            vy = loop_float_constrain(upc_ptr.vy, -CHASSIS_MAX_V, CHASSIS_MAX_V);
            vw = loop_float_constrain(upc_ptr.vw, -CHASSIS_MAX_W, CHASSIS_MAX_W);

            chassis_ptr.gimbal_shutdown_flag = 0;
            chassis_ptr.gimbal_auto_rotate = upc_ptr.auto_rotate;
            if (!chassis_ptr.gimbal_auto_rotate)
                chassis_ptr.given_gimbal_l_yaw = upc_ptr.gimbal_yaw;
            else
            {
                chassis_ptr.given_gimbal_l_yaw += 0.05f;
            }

        // }
        // else
        // {
        //     //chassis_ptr.mode = 0;
        //     vx = 0.0f;
        //     vy = 0.0f;
        //     vw = 0.0f;
        //     chassis_ptr.gimbal_shutdown_flag = 1;
        // }
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

    // 速度滤波
    first_order_filter_cali(&chassis_vx_filter, vx);
    first_order_filter_cali(&chassis_vy_filter, vy);
    first_order_filter_cali(&chassis_vw_filter, vw);
    vx_filter = chassis_vx_filter.out;
    vy_filter = chassis_vy_filter.out;
    vw_filter = chassis_vw_filter.out;

    norm_v = sqrtf(vx_filter * vx_filter + vy_filter * vy_filter);
    chassis_ptr.given_chassis_v[0] = norm_v * LINEAR_TO_RPM;
    chassis_ptr.given_chassis_v[1] = atan2f(vy_filter, vx_filter);
    chassis_ptr.given_chassis_w = vw_filter * LINEAR_TO_RPM;
//    LOG_INFO("given vx: %.2f, vy: %.2f, vw: %.2f", vx_filter, vy_filter, vw_filter);
		
		// norm_v = sqrtf(vx * vx + vy * vy);
    // chassis_ptr.given_chassis_v[0] = norm_v * LINEAR_TO_RPM;
    // chassis_ptr.given_chassis_v[1] = atan2f(vy, vx);
    // chassis_ptr.given_chassis_w = vw * LINEAR_TO_RPM;
			
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
            chassis_w = 2000.0f;
            // case穿透，将转速设为定值后继续执行跟随底盘模式逻辑
        case FOLLOW_CHASSIS:
        {
            const fp32 sin_yaw = sinf(radian_format(tf_ptr.Chassis_angle.yaw_rad));// - tf_ptr.Big_Gimbal_angle.yaw_rad));
            const fp32 cos_yaw = cosf(radian_format(tf_ptr.Chassis_angle.yaw_rad));// - tf_ptr.Big_Gimbal_angle.yaw_rad));
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
        default:
			m3508_ctrl[0].given_speed = 0;
            m3508_ctrl[1].given_speed = 0;
            m3508_ctrl[2].given_speed = 0;
            m3508_ctrl[3].given_speed = 0;

            break;
    }
    static uint8_t log_num = 0;
    if (log_num > 100)
    {
        LOG_INFO("given speed: %d %d %d %d, given angle: %.2f", m3508_ctrl[0].given_speed, m3508_ctrl[1].given_speed, m3508_ctrl[2].given_speed, m3508_ctrl[3].given_speed, m9025_ctrl.given_angle);
        log_num = 0;
    }
    log_num++;


}
MotorPowerObj motorpower[4];
PowerAllocationResult result;
void motor_ctrl_send(void)
{
    static m3508_t m3508_ptr[4];
    DTM_Read(M3508_DATA, m3508_ptr, sizeof(m3508_ptr));
    for(int i = 0; i < 4; i++){
        if(!OMM_detect(M3508_0_ONLINE + i))
            for(int j = 0; j < 4; j++)
                m3508_ctrl[j].given_speed = 0;
    }
    //CAN_Get9025Measure(CAN_9025_M1_TX_ID);
    OMM_detect(M9025_ONLINE);
    if((!OMM_detect(GIMBAL_L_ONLINE)))// || motor_9025_ctrl.measure->ecd_offset == 0)
        chassis_ptr.gimbal_shutdown_flag = 1;
    for(int i = 0; i < 4; i++){
        PID_calc(&m3508_ctrl[i].pid, m3508_ptr[i].speed, m3508_ctrl[i].given_speed);
    }
    if(!chassis_ptr.gimbal_shutdown_flag)
    {
        PID_calc(&m9025_ctrl.pid, m9025_ctrl.cur_angle, m9025_ctrl.given_angle);
        mf9025_given_speed = (int32_t)(*m9025_ctrl.pid.out + m9025_ctrl.ff_speed);
    }
    else
        mf9025_given_speed = 0;
    CAN_Control9025Speed(CAN_MF_SEND_ID, MF9025_MAX_IQ, mf9025_given_speed);


    //CAN_Control3508Current((int16_t)m3508_ctrl[0].pid.out[0], (int16_t)m3508_ctrl[1].pid.out[0], (int16_t)m3508_ctrl[2].pid.out[0], (int16_t)m3508_ctrl[3].pid.out[0]);
    // 功率控制
    static upc_t upc_ptr;
    static power_data_t power_data_ptr;
    static fp32 maxpower;
    DTM_Read(UPC_DATA, &upc_ptr, sizeof(upc_t));
    DTM_Read(POWER_DATA, &power_data_ptr, sizeof(power_data_ptr));

    switch (upc_ptr.nav_state) // 该策略仅作联盟赛哨兵使用
    {
    case 0: // NAV_NORMAL: 常规情况，不使用超电
        maxpower = SENTINEL_MAXPOWER;
        break;
    case 1: // NAV_RUSH: 冲刺，叠加超电功率
        maxpower = SENTINEL_MAXPOWER + power_data_ptr.supercap_power;
        break;
    default:
        maxpower = SENTINEL_MAXPOWER;
        break;
    }

    // 超电剩余能量过低时不使用超电
    if (power_data_ptr.remain_power < REMAINPOWER_MIN)
        maxpower = SENTINEL_MAXPOWER;

    setMaxPower(&power_ctrl_config, maxpower);

    static fp32 energy_buffer;
    DTM_Read(BUFFER_DATA, &energy_buffer, sizeof(energy_buffer));
    limitMaxPower(&power_ctrl_config, energy_buffer);

    for (int i = 0; i < 4; i++)
    {
        motorpower[i].curAv = (float)m3508_ptr[i].speed * RPM_TO_RADS / REDUCTION_RATIO;
        motorpower[i].setAv = (float)m3508_ctrl[i].given_speed * RPM_TO_RADS / REDUCTION_RATIO;
        motorpower[i].pidOutput = m3508_ctrl[i].pid.out[0];
        motorpower[i].pidMaxOutput = m3508_ctrl[i].pid.max_out;
        motorpower[i].Current = m3508_ptr[i].current;
    }
    MotorPowerObj *motors[4] = {&motorpower[0], &motorpower[1], &motorpower[2], &motorpower[3]};
    allocatePowerWithLimit(motors, &power_ctrl_config, &result);

    CAN_Control3508Current((int16_t)result.newTorqueCurrent[0], (int16_t)result.newTorqueCurrent[1],
                           (int16_t)result.newTorqueCurrent[2], (int16_t)result.newTorqueCurrent[3]);
}

float param_ptr[2];
void motor_param_get()
{
    static m3508_t m3508_ptr[4];
    static power_data_t power_data_ptr;

    DTM_Read(M3508_DATA, m3508_ptr, sizeof(m3508_ptr));
    DTM_Read(POWER_DATA, &power_data_ptr, sizeof(power_data_ptr));

    float torqueFeedback[4];
    float rpmFeedback[4];

    for (int i = 0; i < 4; i++) {
        torqueFeedback[i] = (float)m3508_ptr[i].current * 20.0f / 16384.0f * 0.3f;
        rpmFeedback[i] = m3508_ptr[i].speed;
    }

    float effectivePower = 0.0f;
    for (int i = 0; i < 4; i++) {
        const float angularVelocity = rpmFeedback[i] * (3.1415926535f / 30.0f);
        effectivePower += torqueFeedback[i] * angularVelocity;
    }

    PowerControl_CollectMotorData(&ctx, torqueFeedback, rpmFeedback, power_data_ptr.total_power, 4);
    PowerControl_Update(&ctx, effectivePower);
    param_ptr[0] = ctx.k1;
    param_ptr[1] = ctx.k2;
}

void chassis_ctrl_init(void)
{
    static m9025_t m9025_ptr;

    chassis_ptr.given_chassis_v[0] = 0.0f;
    chassis_ptr.given_chassis_v[1] = 0.0f;
    chassis_ptr.given_chassis_w = 0.0f;
    chassis_ptr.ctrl = CHASSIS_RC_OFFLINE;
    chassis_ptr.mode = 0;
    chassis_ptr.gimbal_auto_rotate = 0;
    chassis_ptr.gimbal_shutdown_flag = 1;
    chassis_ptr.last_gimbal_shutdown_flag = 1;

    first_order_filter_init(&chassis_vx_filter, CHASSIS_CONTROL_TIME, filter_vx_num);
    first_order_filter_init(&chassis_vy_filter, CHASSIS_CONTROL_TIME, filter_vy_num);
    first_order_filter_init(&chassis_vw_filter, CHASSIS_CONTROL_TIME, filter_vw_num);

    rng_init(0.98f);

    #ifdef DEBUG_MODE
    // static uint8_t debug_ready = 0;
    // static fp32 debug_param[6];
    // while(!debug_ready)
    // {
    //     DTM_Read(FLAG_DATA, &debug_ready, sizeof(debug_ready));
    //     osDelay(2);
    // }
    //
    // DTM_Read(PARAM_DATA, debug_param, sizeof(debug_param));
    // MF9025_ANGLE_PID[0] = debug_param[0];
    // MF9025_ANGLE_PID[1] = debug_param[1];
    // MF9025_ANGLE_PID[2] = debug_param[2];
    // MF9025_SPEED_PID[0] = (uint16_t)debug_param[3];
    // MF9025_SPEED_PID[1] = (uint16_t)debug_param[4];
    // MF9025_SPEED_PID[2] = (uint16_t)debug_param[5];
    #endif
    while (m9025_ptr.ecd_offset == 0)
    {
        CAN_Get9025Measure(CAN_MF_SEND_ID);
        osDelay(5);
        DTM_Read(M9025_DATA, &m9025_ptr, sizeof(m9025_ptr));
    }
    CAN_Set9025PID(CAN_MF_SEND_ID, CONTROL_PARAM_9025_SPEED_PID, MF9025_SPEED_PID[0], MF9025_SPEED_PID[1], MF9025_SPEED_PID[2]);
    osDelay(5);
    // CAN_Read9025Param(CAN_MF_SEND_ID, CONTROL_PARAM_9025_SPEED_MAX);
    // osDelay(5);
    for(int i = 0; i < 4; i++)
    {
        PID_init(&m3508_ctrl[i].pid, PID_POSITION, M3508_SPEED_PID, M3508_SPEED_PID_OUT_MAX, M3508_SPEED_PID_IOUT_MAX,
          M3508_MAX_POSITION_ACCEL, M3508_MAX_NEGATIVE_ACCEL, M3508_DEADZONE);
    }
    PID_init(&m9025_ctrl.pid, PID_POSITION, MF9025_ANGLE_PID, MF9025_ANGLE_PID_OUT_MAX, MF9025_ANGLE_PID_IOUT_MAX,
      MF9025_MAX_POSITION_ACCEL, MF9025_MAX_NEGATIVE_ACCEL, MF9025_DEADZONE);
    // fp32 (*multi_Kpid_ptr)[4] = MF9025_ANGLE_MULTI_PID;
    // PID_multi_Kp_init(&m9025_ctrl.pid,multi_Kpid_ptr,3);

    initPowerControllerConfig(&power_ctrl_config, M3508_TORQUE_CONST, M3508_CURRENT_LIMIT, M3508_OUTPUT_LIMIT,
        K1_CONST,  K2_CONST, K3_CONST, SENTINEL_MAXPOWER);
    PowerControl_Init(&ctx);
}
