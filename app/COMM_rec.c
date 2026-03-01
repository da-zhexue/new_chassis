/**
 * @file COMM_rec.c
 * @brief 与通信板串口通信模块
 * 用于处理通信板发到串口usart1的数据，并将其解析到相应的数据结构中。
 * 具体见通信协议。
 * @version 1.1
 * @changelog
 * - 2026-02-09 适配新的数据中转模块
 */

#include "COMM_rec.h"
#include "crc.h"
#include "user_lib.h"
#include "bsp_dwt.h"
#include "OMM.h"
#include "DTM.h"
#include "ulog.h"

void cmd_move_handler(const uint8_t* data, upc_t *upc_ptr);
void cmd_rotate_handler(const uint8_t* data, upc_t *upc_ptr);
void cmd_shart_handler(const uint8_t* data, upc_t *upc_ptr);
void cmd_mode_handler(const uint8_t* data, upc_t *upc_ptr);
void cmd_imu_s_handler(const uint8_t* data);
void cmd_imu_l_handler(const uint8_t* data);

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
		case CMD_MODE:
			cmd_mode_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN], &upc_ptr);
			break;
		case CMD_IMU_S_INFO:
			cmd_imu_s_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
		case CMD_IMU_L_INFO:
			cmd_imu_l_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
		default:
			break;
	}
	DTM_Write(UPC_DATA, &upc_ptr, sizeof(upc_t));
	return 0; 
}

void cmd_move_handler(const uint8_t* data, upc_t *upc_ptr)
{
	unpack_4bytes_to_floats(&data[0], &upc_ptr->vx);
	unpack_4bytes_to_floats(&data[4], &upc_ptr->vy);
	unpack_4bytes_to_floats(&data[8], &upc_ptr->vw);
	LOG_INFO("Get move cmd vx: %.2f, vy: %.2f, vw: %.2f", upc_ptr->vx, upc_ptr->vy, upc_ptr->vw);
	OMM_update(UPC_ONLINE);
}

void cmd_rotate_handler(const uint8_t* data, upc_t *upc_ptr)
{
	unpack_4bytes_to_floats(&data[0], &upc_ptr->chassis_yaw);
	unpack_4bytes_to_floats(&data[4], &upc_ptr->gimbal_yaw);
	LOG_INFO("Get rotate cmd chassis: %.2f, gimbal: %.2f", upc_ptr->chassis_yaw, upc_ptr->gimbal_yaw);
	OMM_update(UPC_ONLINE);
}

void cmd_mode_handler(const uint8_t* data, upc_t *upc_ptr)
{
	upc_ptr->mode = data[12];
	OMM_update(UPC_ONLINE);
}

void cmd_imu_s_handler(const uint8_t* data)
{
	static fp32 gimbal_s_ptr[3];
	unpack_4bytes_to_floats(&data[0], &gimbal_s_ptr[0]);
	unpack_4bytes_to_floats(&data[4], &gimbal_s_ptr[1]);
	unpack_4bytes_to_floats(&data[8], &gimbal_s_ptr[2]);
	DTM_Write(GIMBAL_S_DATA, gimbal_s_ptr, sizeof(gimbal_s_ptr));
	LOG_INFO("Get small imu: %.2f", gimbal_s_ptr[0]);
	OMM_update(GIMBAL_S_ONLINE);
}

void cmd_imu_l_handler(const uint8_t* data)
{
	static fp32 gimbal_l_ptr[3];
	unpack_4bytes_to_floats(&data[0], &gimbal_l_ptr[0]);
	unpack_4bytes_to_floats(&data[4], &gimbal_l_ptr[1]);
	unpack_4bytes_to_floats(&data[8], &gimbal_l_ptr[2]);
	DTM_Write(GIMBAL_L_DATA, gimbal_l_ptr, sizeof(gimbal_l_ptr));
	LOG_INFO("Get big imu: %.2f", gimbal_l_ptr[0]);
	OMM_update(GIMBAL_L_ONLINE);
}

extern void send_start_handler();
void cmd_shart_handler(const uint8_t* data, upc_t *upc_ptr)
{
	send_start_handler();
	upc_ptr->game_start = 1;
}