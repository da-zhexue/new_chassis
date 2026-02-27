/**
 * @file TF_task.c
 * @brief 坐标系变换任务模块
 * 计算底盘坐标系和大小云台坐标系的角度。
 * @version 1.1
 * @changelog
 * - 2026-02-09 适配新的数据中转模块，将自动校准从修改云台yaw改为修改底盘yaw
 */

#include "TF_task.h"

#include <string.h>

#include "user_lib.h"
#include "OMM.h"
#include "DTM.h"

void TF_Update(angle_t *angle, const float new_angle_deg[3]);

void TF_Task(void const * argurment)
{
		static float Yaw_diff = 0.0f;
		static float chassis_angle_temp[3] = {0.0f, 0.0f, 0.0f};
		static float gimbal_s_offset[3] = {0.0f, 0.0f, 0.0f};
		static float gimbal_l_offset[3] = {0.0f, 0.0f, 0.0f};
		static uint8_t gimbal_s_get_offset = 0, gimbal_l_get_offset = 0;
		static TF_t tf_ptr;
		static fp32 gimbal_l_ptr[3], gimbal_s_ptr[3];
		static m9025_t m9025_ptr;
		while(1)
		{
			if(OMM_detect(GIMBAL_L_ONLINE))
			{
				DTM_Read(GIMBAL_L_DATA, gimbal_l_ptr, sizeof(gimbal_l_ptr));
				DTM_Read(M9025_DATA, &m9025_ptr, sizeof(m9025_ptr));
				if (!gimbal_l_get_offset){
					gimbal_l_get_offset = 1;
					memcpy(gimbal_l_offset, gimbal_l_ptr, sizeof(gimbal_l_ptr));
				}
				else
					for (int i = 0; i < 3; i++) gimbal_l_ptr[i] -= gimbal_l_offset[i];
				if (!gimbal_s_get_offset){
					gimbal_s_get_offset = 1;
					memcpy(gimbal_s_offset, gimbal_s_ptr, sizeof(gimbal_s_ptr));
				}
				else
					for (int i = 0; i < 3; i++) gimbal_s_ptr[i] -= gimbal_s_offset[i];
				Yaw_diff = theta_format((fp32)(m9025_ptr.ecd - m9025_ptr.ecd_offset)/ 32768.0f * 180.0f); // -PI~PI
				#ifdef AUTO_CORRECTION_ENABLE
					if(m9025_ptr.imu_yaw_offset == 0)
						m9025_ptr.imu_yaw_offset = theta_format((fp32)(m9025_ptr.ecd - MF9025_ECD_IN_ZERO)/ 32768.0f * 180.0f);
					chassis_angle_temp[0] = theta_format(Yaw_diff + gimbal_l_ptr[0] + m9025_ptr.imu_yaw_offset);
				#else
					chassis_angle_temp[0] = theta_format(Yaw_diff + gimbal_l_ptr[0]);
				#endif

				TF_Update(&tf_ptr.Big_Gimbal_angle, gimbal_l_ptr);
				TF_Update(&tf_ptr.Chassis_angle, chassis_angle_temp);
			}
			if(OMM_detect(GIMBAL_S_ONLINE))
			{
				DTM_Read(GIMBAL_S_DATA, gimbal_s_ptr, sizeof(gimbal_s_ptr));
				TF_Update(&tf_ptr.Small_Gimbal_angle, gimbal_s_ptr);
			}
			
			BMI088_Read(&BMI088);
			tf_ptr.Gyro[Z] = BMI088.Gyro[Z]; // 读取底盘yaw轴角速度用于旋转时大云台前馈补偿
			DTM_Write(TF_DATA, &tf_ptr, sizeof(tf_ptr));
			osDelay(1);
		}

}

void TF_Update(angle_t *angle, const float new_angle_deg[3])
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
	angle->yaw_total_angle = 360.0f * (fp32)angle->yaw_round_count + angle->yaw_deg;
	angle->yaw_angle_last = angle->yaw_deg;
}
