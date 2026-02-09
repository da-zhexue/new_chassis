/**
 * @file comm_task.c
 * @brief 与通信板通信任务模块
 * 向通信板发送数据，详见通信协议。
 * @version 1.1
 * @changelog
 * - 2026-01-17 适配新的数据中转模块
 */

#include "comm_task.h"
#include "cmsis_os.h"
#include "COMM_rec.h"
#include "usart.h"
#include "user_lib.h"
#include "crc.h"
#include "DTM.h"

void send_attitude_handler(const TF_t *tf_ptr);

void commTask(void const * argument)
{
	static TF_t tf_ptr;
    while(1)
    {
    	DTM_Read(TF_DATA, &tf_ptr, sizeof(tf_ptr));
		send_attitude_handler(&tf_ptr);
        osDelay(5);
			
    }
}

void send_attitude_handler(const TF_t *tf_ptr)
{
	static uint8_t send_data[UPC_TOTAL_LEN] = {0};
	send_data[0] = UPC_HEADER;
	send_data[1] = UPC_DATA_LEN;
	send_data[2] = 0;send_data[3] = 0;
	send_data[5] = SEND_ATTITUDE & 0xFF;
	send_data[6] = (SEND_ATTITUDE >> 8) & 0xFF;

	pack_float_to_4bytes(tf_ptr->Chassis_angle.yaw_deg, &send_data[7]);
	pack_float_to_4bytes(tf_ptr->Small_Gimbal_angle.yaw_deg, &send_data[11]);
	pack_float_to_4bytes(tf_ptr->Small_Gimbal_angle.pitch_deg, &send_data[15]);

	Append_CRC8_Check_Sum(send_data, UPC_HEADER_LEN);
	Append_CRC16_Check_Sum(send_data, UPC_TOTAL_LEN);
	HAL_UART_Transmit_DMA(&huart1, send_data, sizeof(send_data));
}
