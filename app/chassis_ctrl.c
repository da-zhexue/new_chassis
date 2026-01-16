#include "chassis_ctrl.h"
#include "bsp_dwt.h"
#include "pid.h"
#include "math.h"
#include "user_lib.h"
#include "Online_Monitor.h"
#include "DBUS.h"
#include "CAN_tx.h"

chassis_ctrl_t chassis_ctrl;

void ctrl_data_update(rc_ctrl_t *rc_ctrl_ptr, upc_t *upc_ptr)
{
    if(Online_Monitors(rc_ctrl_ptr->last_online_time, RC_ONLINE))
        chassis_ctrl.ctrl = rc_ctrl_ptr->s1;
    else
        chassis_ctrl.ctrl = CHASSIS_RC_OFFLINE;

    static fp32 vx, vy, vw, yaw_delta;
    static fp32 norm_v;
    static fp32 small_gimbal_pitch = 0.0f, small_gimbal_yaw = 0.0f;
    if(chassis_ctrl.ctrl == CHASSIS_RC)
    {
        chassis_ctrl.mode = 1;//rc_ctrl_ptr->s2;
        upc_ptr->start_upc_flag = 0;
        vx = (fp32)rc_ctrl_ptr->ch3;
        vy = (fp32)rc_ctrl_ptr->ch2;
        vw = (fp32)(rc_ctrl_ptr->ch1);

        chassis_ctrl.gimbal_shutdown_flag = 0;
        yaw_delta = (fp32)(rc_ctrl_ptr->ch0 * GIMBAL_ANGLE_DELTA_MAX / 660.0f);
        chassis_ctrl.given_gimbal_yaw += yaw_delta;
    }
    else if(chassis_ctrl.ctrl == CHASSIS_UPC)
    {
        chassis_ctrl.mode = 1;//upc_ptr->mode;
        upc_ptr->start_upc_flag = 1;

        vx = upc_ptr->vx * 300;
        vy = upc_ptr->vy * 300;
        vw = upc_ptr->vw * 300;

        chassis_ctrl.gimbal_shutdown_flag = 0;
        chassis_ctrl.given_gimbal_yaw = upc_ptr->gimbal_yaw;
    }
    else if(chassis_ctrl.ctrl == GIMBAL_RC)
    {
        chassis_ctrl.mode = 1;//rc_ctrl_ptr->s2;
        upc_ptr->start_upc_flag = 0;

        uint8_t send_data1[8] = {0};
        send_data1[1] = 1;
        //CAN_CBoard_CMD(0x223, send_data1);
        small_gimbal_pitch += ((fp32)rc_ctrl_ptr->ch3 * 0.05f); // small_gimbal_pitch
        small_gimbal_yaw += ((fp32)rc_ctrl_ptr->ch2 * 0.05f); // small_gimbal_yaw
        uint8_t shoot = (rc_ctrl_ptr->ch1 > 0) ? 1 : 0;

        uint8_t send_data2[8];
        pack_float_to_4bytes(small_gimbal_yaw, &send_data2[0]);
        pack_float_to_4bytes(small_gimbal_pitch, &send_data2[4]);
        //CAN_CBoard_CMD(0x222, send_data2);

        uint8_t send_data3[8] = {0};
        send_data3[0] = shoot;
        //CAN_CBoard_CMD(0x223, send_data3);
    }
    else
    {
		chassis_ctrl.mode = 0;
        upc_ptr->start_upc_flag = 0;
        vx = 0.0f;
        vy = 0.0f;
        vw = 0.0f;
        chassis_ctrl.gimbal_shutdown_flag = 1;
    }

    norm_v = (sqrt(vx * vx + vy * vy) / 660.0f) > 1.0f ? 1.0f : (sqrt(vx * vx + vy * vy) / 660.0f);
    chassis_ctrl.given_chassis_v[0] = norm_v * CHASSIS_MAX_V;
    chassis_ctrl.given_chassis_v[1] = atan2(vy, vx);
    chassis_ctrl.given_chassis_w = vw * CHASSIS_MAX_W;
}

void chassis_ctrl_init(void)
{
    chassis_ctrl.given_chassis_v[0] = 0.0f;
    chassis_ctrl.given_chassis_v[1] = 0.0f;
    chassis_ctrl.given_chassis_w = 0.0f;
    chassis_ctrl.ctrl = CHASSIS_RC_OFFLINE;
    chassis_ctrl.mode = 1;
    chassis_ctrl.gimbal_shutdown_flag = 1;
    chassis_ctrl.last_gimbal_shutdown_flag = 1;
}

chassis_ctrl_t* get_chassis_ctrl_data(void)
{
    return &chassis_ctrl;
}
