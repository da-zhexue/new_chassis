#include "motor_ctrl.h"
#include "Online_Monitor.h"
#include "TF_task.h"
#include "user_lib.h"
#include "math.h"
#include "CAN_tx.h"

motor_3508_ctrl_t motor_3508_ctrl[4];
motor_9025_ctrl_t motor_9025_ctrl;
static motor_9025_measure_t motor_9025_measure;
static motor_3508_measure_t motor_3508_measure[4];

static fp32 M3508_SPEED_PID[3] = {M3508_SPEED_PID_KP, M3508_SPEED_PID_KI, M3508_SPEED_PID_KD};
static fp32 MF9025_ANGLE_PID[3] = {MF9025_ANGLE_PID_KP, MF9025_ANGLE_PID_KI, MF9025_ANGLE_PID_KD};

static fp32 MF9025_ANGLE_MULTI_PID [3][4] = {
    {10,MF9025_ANGLE_PID_KP, MF9025_ANGLE_PID_KI, MF9025_ANGLE_PID_KD},
    {30,MF9025_ANGLE_PID_KP2, MF9025_ANGLE_PID_KI2, MF9025_ANGLE_PID_KD2},
    {40,MF9025_ANGLE_PID_KP3, MF9025_ANGLE_PID_KI3, MF9025_ANGLE_PID_KD3}
};

TF_t *tf_ptr;

void motor_ctrl_init(void)
{
    tf_ptr = get_TF();
    motor_9025_ctrl.measure = &motor_9025_measure;
    for(int i = 0; i < 4; i++)
    {
        motor_3508_ctrl[i].measure = &motor_3508_measure[i];
        PID_init(&motor_3508_ctrl[i].pid, PID_POSITION, M3508_SPEED_PID, M3508_SPEED_PID_OUT_MAX, M3508_SPEED_PID_IOUT_MAX,
          M3508_MAX_POSITION_ACCEL, M3508_MAX_NEGATIVE_ACCEL, M3508_DEADZONE);
    }
    PID_init(&motor_9025_ctrl.pid, PID_POSITION, MF9025_ANGLE_PID, MF9025_ANGLE_PID_OUT_MAX, MF9025_ANGLE_PID_IOUT_MAX,
      MF9025_MAX_POSITION_ACCEL, MF9025_MAX_NEGATIVE_ACCEL, MF9025_DEADZONE);
    fp32 (*multi_Kpid_ptr)[4];
    multi_Kpid_ptr = MF9025_ANGLE_MULTI_PID;
    PID_multi_Kp_init(&motor_9025_ctrl.pid,multi_Kpid_ptr,3);
}

void motor_ctrl_update(chassis_ctrl_t* chassis_ctrl)
{
    fp32 chassis_v = chassis_ctrl->given_chassis_v[0];
    fp32 theta = chassis_ctrl->given_chassis_v[1];
    fp32 chassis_w = chassis_ctrl->given_chassis_w + tf_ptr->Chassis_angle.yaw_rad;

    fp32 sin_yaw, cos_yaw;
    fp32 vx, vy;
    fp32 vx_set, vy_set;
    
    switch(chassis_ctrl->mode)
    {
        case SPINNING_TOP:
            chassis_w = 4000.0f; // case穿透，将转速设为定值后继续执行跟随底盘模式逻辑
		    // Case penetration, set the speed to a constant value and continue to execute the logic of FOLLOW_CHASSIS mode
        case FOLLOW_CHASSIS:
        {
            sin_yaw = sin(radian_format(tf_ptr->Chassis_angle.yaw_rad - tf_ptr->Small_Gimbal_angle.yaw_rad));
            cos_yaw = cos(radian_format(tf_ptr->Chassis_angle.yaw_rad - tf_ptr->Small_Gimbal_angle.yaw_rad));
            vx = chassis_v * cos(theta);
			vy = chassis_v * sin(theta);
            vx_set = vx * cos_yaw - vy * sin_yaw;
			vy_set = vx * sin_yaw + vy * cos_yaw;

            motor_3508_ctrl[0].given_speed = (vx_set + vy_set) / ROOT_2 + chassis_w;
            motor_3508_ctrl[1].given_speed = (-vx_set + vy_set) / ROOT_2 + chassis_w;
            motor_3508_ctrl[2].given_speed = (-vx_set - vy_set) / ROOT_2 + chassis_w;
            motor_3508_ctrl[3].given_speed = (vx_set - vy_set) / ROOT_2 + chassis_w;

            motor_9025_ctrl.given_angle = chassis_ctrl->given_gimbal_yaw;
            break;
        }
        case FOLLOW_GIMBAL:
            break;
				case STOPPING:
        default:
						motor_3508_ctrl[0].given_speed = 0;
            motor_3508_ctrl[1].given_speed = 0;
            motor_3508_ctrl[2].given_speed = 0;
            motor_3508_ctrl[3].given_speed = 0;
						
            break;
    }
		for(int i = 0; i < 4; i++){
        if(!Online_Monitors(motor_3508_ctrl[i].measure->last_online, CHASSIS_MOTOR_0_ONLINE + i))
						for(int j = 0; j < 4; j++)
								motor_3508_ctrl[j].given_speed = 0;
    }
    Online_Monitors(motor_9025_ctrl.measure->last_online, GIMBAL_MOTOR_ONLINE);
		if((!Online_Monitors(tf_ptr->big_gimbal_imu_last_online_time, BIG_GIMBAL_IMU_ONLINE)))// || motor_9025_ctrl.measure->ecd_offset == 0)
			chassis_ctrl->gimbal_shutdown_flag = 1;
		for(int i = 0; i < 4; i++){
        PID_calc(&motor_3508_ctrl[i].pid, motor_3508_ctrl[i].measure->speed, motor_3508_ctrl[i].given_speed);
    }
		CAN_Control3508Current(*motor_3508_ctrl[0].pid.out, *motor_3508_ctrl[1].pid.out, *motor_3508_ctrl[2].pid.out, *motor_3508_ctrl[3].pid.out);
		// 其实就是取pid.out[0]
		if(!chassis_ctrl->gimbal_shutdown_flag)
		{
				PID_calc(&motor_9025_ctrl.pid, -tf_ptr->Big_Gimbal_angle.yaw_total_angle, motor_9025_ctrl.given_angle);
				CAN_Control9025Speed(CAN_9025_M1_TX_ID, MF9025_MAX_IQ, (int32_t)(*motor_9025_ctrl.pid.out + (5729.5779513f * tf_ptr->Gyro[2]))); // 前馈补偿 底盘yaw轴角速度
		}
		else 
				CAN_Control9025Speed(CAN_9025_M1_TX_ID, MF9025_MAX_IQ, 0);
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
