#include "TF_task.h"
#include "bsp_dwt.h"
#include "user_lib.h"
#include "transfer_function.h"
#include "COMM_rec.h"
#include "Online_Monitor.h"
#include "motor_ctrl.h"
#include "CAN_tx.h"
#include "CAN_rx.h"

void TF_Update(angle_t *angle, float new_angle_deg[3]);
void TF_Reset(angle_t *angle);

TF_t TF;

void TF_Task(void const * argurment)
{
		float Yaw_diff = 0.0f;
		float chassis_angle_temp[3] = {0.0f, 0.0f, 0.0f};
		small_gimbal_angle_t_temp *small_gimbal_angle_deg_ptr = get_small_gimbal_angle_temp();
		big_gimbal_angle_t *big_gimbal_angle_deg_ptr = get_big_gimbal_angle();
		motor_9025_measure_t* motors_9025_measure_ptr = get_motor_9025_measure_data();
		while(1)
		{
			if(Online_Monitors(small_gimbal_angle_deg_ptr->small_gimbal_imu_last_online_time, SMALL_GIMBAL_IMU_ONLINE))
			{
				TF_Update(&TF.Small_Gimbal_angle, small_gimbal_angle_deg_ptr->small_gimbal_angle);
				TF.small_gimbal_imu_last_online_time = small_gimbal_angle_deg_ptr->small_gimbal_imu_last_online_time;
			}
			if(Online_Monitors(big_gimbal_angle_deg_ptr->big_gimbal_imu_last_online_time, BIG_GIMBAL_IMU_ONLINE))
			{
				TF_Update(&TF.Big_Gimbal_angle, big_gimbal_angle_deg_ptr->big_gimbal_angle);
				TF.big_gimbal_imu_last_online_time = big_gimbal_angle_deg_ptr->big_gimbal_imu_last_online_time;
				if(motors_9025_measure_ptr->ecd_offset == 0) motors_9025_measure_ptr->ecd_offset = motors_9025_measure_ptr->ecd;
				Yaw_diff = radian_format((motors_9025_measure_ptr->ecd - motors_9025_measure_ptr->ecd_offset)/ 32768.0f * PI); // -PI~PI
				chassis_angle_temp[0] = radian_format(Yaw_diff + TF.Small_Gimbal_angle.yaw_rad) * 57.295779513f;
				TF_Update(&TF.Chassis_angle, chassis_angle_temp);
			}

			BMI088_Read(&BMI088);
			TF.Gyro[Z] = BMI088.Gyro[Z]; // 读取底盘yaw轴角速度用于旋转时大云台前馈补偿

			osDelay(1);
		}
}

void TF_Update(angle_t *angle, float new_angle_deg[3])
{
	angle->yaw_deg = new_angle_deg[0];
	angle->pitch_deg = new_angle_deg[1];
	angle->roll_deg = new_angle_deg[2];

	angle->yaw_rad = angle->yaw_deg / 57.295779513f;
	angle->pitch_rad = angle->pitch_deg / 57.295779513f;
	angle->roll_rad = angle->roll_deg / 57.295779513f;

	if (angle->yaw_deg - angle->yaw_angle_last > 180.0f)
		angle->yaw_round_count--;
	else if (angle->yaw_deg - angle->yaw_angle_last < -180.0f)
		angle->yaw_round_count++;
	angle->yaw_total_angle = 360.0f * angle->yaw_round_count + angle->yaw_deg;
	angle->yaw_angle_last = angle->yaw_deg;
}

void TF_Reset(angle_t *angle)
{
		static float angle_zero[3] = {0.0f, 0.0f, 0.0f};
		TF_Update(angle, angle_zero);
}

TF_t* get_TF(){
	return &TF;
}
