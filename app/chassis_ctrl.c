#include "chassis_ctrl.h"
#include "bsp_dwt.h"
#include "pid.h"
#include "math.h"
#include "Online_Monitor.h"
#include "DBUS.h"

chassis_ctrl_t chassis_ctrl;

void ctrl_data_update(rc_ctrl_t *rc_ctrl_ptr, upc_t *upc_ptr)
{
    if(Online_Monitors(rc_ctrl_ptr->last_online_time, RC_ONLINE))
        chassis_ctrl.ctrl = rc_ctrl_ptr->s1;
    else
        chassis_ctrl.ctrl = CHASSIS_RC_OFFLINE;

    static fp32 vx, vy, vw, yaw_delta;
    static fp32 norm_v;
    if(chassis_ctrl.ctrl == CHASSIS_RC)
    {
        chassis_ctrl.mode = 1;//rc_ctrl_ptr->s2;
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
        vx = upc_ptr->vx * 300;
        vy = upc_ptr->vy * 300;
        vw = upc_ptr->vw * 300;

        chassis_ctrl.gimbal_shutdown_flag = 0;
        chassis_ctrl.given_gimbal_yaw = upc_ptr->gimbal_yaw;
    }
    else
    {
				chassis_ctrl.mode = 0;
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
