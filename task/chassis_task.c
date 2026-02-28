/**
 * @file comm_task.c
 * @brief 底盘控制任务模块
 * 获取控制量，发送给控制函数。
 * @version 1.1
 * @changelog
 * - 2026-02-09  减少头文件引用
 */

#include "chassis_task.h"
#include "cmsis_os.h"
#include "chassis_ctrl.h"

void ChassisTask(void const * argument)
{
	chassis_ctrl_init();
	while(1)
	{
		ctrl_data_update();
		motor_ctrl_update();
		motor_ctrl_send();
		osDelay(3);
	}
}
