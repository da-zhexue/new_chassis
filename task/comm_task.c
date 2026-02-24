/**
 * @file comm_task.c
 * @brief 与通信板通信任务模块
 * 向通信板发送数据，详见通信协议。
 * @version 1.1
 * @changelog
 * - 2026-01-17 适配新的数据中转模块
 */

#include "comm_task.h"

#include <string.h>

#include "cmsis_os.h"
#include "COMM_rec.h"
#include "usart.h"
#include "user_lib.h"
#include "crc.h"
#include "DTM.h"
#include "OMM.h"
#include "ulog.h"

void send_attitude_handler(const TF_t *tf_ptr);
void send_onlinestate_handler();

void commTask(void const * argument)
{
	static TF_t tf_ptr;
    while(1)
    {
    	DTM_Read(TF_DATA, &tf_ptr, sizeof(tf_ptr));
		send_attitude_handler(&tf_ptr);
    	send_onlinestate_handler();

        osDelay(5);
			
    }
}

void CMD_PackPacket(const uint8_t *data_in, const uint16_t data_len, const uint16_t cmd_id) //, uint8_t *data_out)
{
	const uint16_t total_len = FRAME_HEADER_LEN + CMD_ID_LEN + data_len + 2;
	uint8_t data_out[total_len];
	uint32_t offset = 0;
	data_out[offset++] = SOF_VALUE;
	data_out[offset++] = data_len & 0xFF;
	data_out[offset++] = (data_len >> 8) & 0xFF;

	data_out[offset++] = 0;
	data_out[offset++] = 0;
	data_out[offset++] = cmd_id & 0xFF;
	data_out[offset++] = (cmd_id >> 8) & 0xFF;
	if (data_in != NULL && data_len > 0) {
		memcpy(&data_out[offset], data_in, data_len);
		offset += data_len;
	}
	Append_CRC8_Check_Sum(data_out, FRAME_HEADER_LEN);
	Append_CRC16_Check_Sum(data_out, total_len);

	HAL_UART_Transmit_DMA(&huart1, data_out, sizeof(data_out));
}

void send_attitude_handler(const TF_t *tf_ptr)
{
	static uint8_t send_yaw[4] = {0};
	pack_float_to_4bytes(tf_ptr->Chassis_angle.yaw_deg, &send_yaw[0]);
	LOG_INFO("send chassis yaw: %.2f", SEND_ATTITUDE, tf_ptr->Chassis_angle.yaw_deg);
	CMD_PackPacket(send_yaw, sizeof(send_yaw), SEND_ATTITUDE);
}

void send_onlinestate_handler()
{
	static uint8_t send_online[1] = {0};
	send_online[0] |= IS_ONLINE(M3508_0_ONLINE) < 0;
	send_online[0] |= IS_ONLINE(M3508_1_ONLINE) < 1;
	send_online[0] |= IS_ONLINE(M3508_2_ONLINE) < 2;
	send_online[0] |= IS_ONLINE(M3508_3_ONLINE) < 3;
	send_online[0] |= IS_ONLINE(M9025_ONLINE) < 4;

	CMD_PackPacket(send_online, sizeof(send_online), SEND_ONLINE);
}

void send_start_handler()
{
	static uint8_t send_start[1] = {1};
	CMD_PackPacket(send_start, sizeof(send_start), SEND_START);
	LOG_INFO("START!");
}
