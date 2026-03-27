/**
 * @file comm.c
 * @brief 与通信板串口通信模块
 * 用于处理通信板发到串口usart1的数据，并将其解析到相应的数据结构中。
 * 具体见通信协议。
 * @version 1.1
 * @changelog
 * - 2026-02-09 适配新的数据中转模块
 */

#include "comm.h"

#include <string.h>

#include "crc.h"
#include "user_lib.h"
#include "bsp_dwt.h"
#include "OMM.h"
#include "DTM.h"
#include "ulog.h"
#include "usart.h"

void cmd_move_handler(const uint8_t* data, upc_t *upc_ptr);
void cmd_rotate_handler(const uint8_t* data, upc_t *upc_ptr);
void cmd_shart_handler(const uint8_t* data, upc_t *upc_ptr);
void cmd_over_handler(const uint8_t* data, upc_t *upc_ptr);
void cmd_state_handler(const uint8_t* data, upc_t *upc_ptr);
void cmd_imu_l_handler(const uint8_t* data);
void cmd_buffer_handler(const uint8_t* data);
void cmd_debug_handler(const uint8_t* data);
void cmd_rc_ctrl_handler(const uint8_t* data, upc_t *upc_ptr);

void upc_send_attitude_handler(void);

uint8_t upc_decode(uint8_t* rx_data)
{
	static upc_t upc_ptr;
	// if(!upc_ptr.start_upc_flag)
	// 	return 0;
	if(rx_data[0] != SOF_VALUE || rx_data[3] != 0)
		return 1;
	const uint16_t data_len = (rx_data[2] << 8) | rx_data[1];
	if(!Verify_CRC8_Check_Sum(rx_data, FRAME_HEADER_LEN) || !Verify_CRC16_Check_Sum(rx_data , data_len+9))
		return 2;

	const uint16_t cmd_id = (rx_data[6] << 8) | rx_data[5];

	switch(cmd_id)
	{
		case CMD_MOVE:
			cmd_move_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN], &upc_ptr);
			break;
		case CMD_ROTATE:
			cmd_rotate_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN], &upc_ptr);
			break;
		case CMD_START:
			cmd_shart_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN], &upc_ptr);
			break;
		case CMD_OVER:
			cmd_over_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN], &upc_ptr);
			break;
		case CMD_MODE:
			cmd_state_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN], &upc_ptr);
			break;
		case CMD_IMU_L_INFO:
			cmd_imu_l_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
		case CMD_ONLINECB:
			cmd_onlinecb_handler(1);
			break;
		case CMD_POWER:
			cmd_buffer_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
		case CMD_RC:
			cmd_rc_ctrl_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN], &upc_ptr);
			break;
//		case CMD_DEBUG:
//			cmd_debug_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN]);
//			break;
		default:
			break;
	}
	TF_t tf_ptr;
	DTM_Read(TF_DATA, &tf_ptr, sizeof(tf_ptr));
	if (upc_ptr.auto_rotate)
		upc_ptr.gimbal_yaw = -tf_ptr.Big_Gimbal_angle.yaw_total_angle;
	if ((DWT_GetTimeline_us() - upc_ptr.last_rc_ctrl > 2000000) && upc_ptr.rc_ctrl) // 2s无遥控信号则由上位机控制
	{
		upc_ptr.rc_ctrl = 0;
		upc_ptr.gimbal_yaw = -tf_ptr.Big_Gimbal_angle.yaw_total_angle;
	}
	DTM_Write(UPC_DATA, &upc_ptr, sizeof(upc_t));
	return 0; 
}

uint16_t last_bag = 0;
uint16_t loss_bag = 0;
float loss_rate = 0.0f;
void cmd_debug_handler(const uint8_t* data)
{
	uint16_t bag = data[0];
	loss_bag += (bag - last_bag - 1);
	loss_rate = (float)loss_bag / (float)bag * 1.0f;
	last_bag = bag;
}

void cmd_move_handler(const uint8_t* data, upc_t *upc_ptr)
{
	unpack_4bytes_to_floats(&data[0], &upc_ptr->vx);
	unpack_4bytes_to_floats(&data[4], &upc_ptr->vy);
	unpack_4bytes_to_floats(&data[8], &upc_ptr->vw);
	//LOG_INFO("Get move cmd vx: %.2f, vy: %.2f, vw: %.2f", upc_ptr->vx, upc_ptr->vy, upc_ptr->vw);
	OMM_update(UPC_ONLINE);
	// uint16_t bag = (data[0] << 8) | data[1];
	// loss_bag += (bag - last_bag - 1);
	// loss_rate = (float)loss_bag / (float)bag * 1.0f;
	// last_bag = bag;
}

void cmd_rotate_handler(const uint8_t* data, upc_t *upc_ptr)
{
	unpack_4bytes_to_floats(&data[0], &upc_ptr->chassis_yaw);
	unpack_4bytes_to_floats(&data[4], &upc_ptr->gimbal_yaw);
	//LOG_INFO("Get rotate cmd chassis: %.2f, gimbal: %.2f", upc_ptr->chassis_yaw, upc_ptr->gimbal_yaw);
	OMM_update(UPC_ONLINE);
}

void cmd_state_handler(const uint8_t* data, upc_t *upc_ptr)
{
	upc_ptr->mode = data[12] & 0x01;
	upc_ptr->auto_rotate = (data[12] >> 1) & 0x01;
	upc_ptr->nav_state = (data[12] >> 2) & 0x01;
	OMM_update(UPC_ONLINE);
}

void cmd_imu_l_handler(const uint8_t* data)
{
	static fp32 gimbal_l_ptr[3];
	unpack_4bytes_to_floats(&data[0], &gimbal_l_ptr[0]);
	unpack_4bytes_to_floats(&data[4], &gimbal_l_ptr[1]);
	unpack_4bytes_to_floats(&data[8], &gimbal_l_ptr[2]);
	DTM_Write(GIMBAL_L_DATA, gimbal_l_ptr, sizeof(gimbal_l_ptr));
	//LOG_INFO("Get big imu: %.2f", gimbal_l_ptr[0]);
	OMM_update(GIMBAL_L_ONLINE);
}

void cmd_shart_handler(const uint8_t* data, upc_t *upc_ptr)
{
	//send_start_handler();
	upc_ptr->game_start = 1;
}

void cmd_over_handler(const uint8_t* data, upc_t *upc_ptr)
{
	//send_start_handler();
	upc_ptr->game_start = 0;
}

void cmd_rc_ctrl_handler(const uint8_t* data, upc_t *upc_ptr)
{
	upc_ptr->last_rc_ctrl = DWT_GetTimeline_us();
	upc_ptr->rc_ctrl = 1;
	upc_ptr->rc_data.ch0 = (int16_t)((data[1] << 8) | data[0]);
	upc_ptr->rc_data.ch1 = (int16_t)((data[3] << 8) | data[2]);
	upc_ptr->rc_data.ch2 = (int16_t)((data[5] << 8) | data[4]);
	upc_ptr->rc_data.ch3 = (int16_t)((data[7] << 8) | data[6]);
	OMM_update(UPC_ONLINE);
}

uint8_t cmd_onlinecb_handler(const uint8_t on)
{
	static uint8_t onlinecb = 0;
	if (on == 1)
	{
		onlinecb = 1;
		return 0;
	}
	if (onlinecb == 1)
	{
		onlinecb = 0;
		return 1;
	}
	return 0;
}

void cmd_buffer_handler(const uint8_t* data)
{
	static fp32 buffer_ptr;
	unpack_4bytes_to_floats(&data[0], &buffer_ptr);
	DTM_Write(BUFFER_DATA, &buffer_ptr, sizeof(buffer_ptr));
}

void CMD_PackPacket(const uint8_t *data_in, const uint16_t data_len, const uint16_t cmd_id)
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

	HAL_UART_Transmit(&huart6, data_out, sizeof(data_out), 100);
}

void send_attitude_handler(const TF_t *tf_ptr)
{
	static uint8_t send_yaw[4] = {0};
	pack_float_to_4bytes(tf_ptr->Chassis_angle.yaw_deg, &send_yaw[0]);
	//LOG_INFO("send chassis yaw: %.2f", SEND_ATTITUDE, tf_ptr->Chassis_angle.yaw_deg);
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
	//LOG_INFO("START!");
}

void send_onlinecb_handler()
{
	static uint8_t send_onlinecb[1] = {1};
	CMD_PackPacket(send_onlinecb, sizeof(send_onlinecb), SEND_ONLINECB);
}