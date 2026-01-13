#include "usart.h"
#include "COMM_rec.h"
#include "crc.h"
#include "user_lib.h"
#include "bsp_dwt.h"
#include "CAN_tx.h"

static upc_t upc;

static big_gimbal_angle_t big_gimbal_angle;

void upc_cmd_imu_handler(uint8_t* data);
void upc_cmd_move_handler(uint8_t* data);
void upc_cmd_gimbal_handler(uint8_t* data);
void upc_cmd_shoot_handler(uint8_t* data);
void upc_cmd_mode_handler(uint8_t* data);
void comm_cmd_small_gimbal_imu_handler(uint8_t* data);
void comm_cmd_big_gimbal_imu_handler(uint8_t* data);

void upc_send_attitude_handler(void);

uint8_t upc_decode(uint8_t* rx_data)
{
//	if(sizeof(rx_data) != UPC_TOTAL_LEN)
//		return 1; 
	// if(car.pChassis->state != STATE_UPC)
	// 	return 0;
	if(rx_data[0] != UPC_HEADER || rx_data[2] != 0 || rx_data[3] != 0)
		return 2; 
	if(rx_data[1] != UPC_DATA_LEN)
		return 3; 
	if(!Verify_CRC8_Check_Sum(rx_data, UPC_HEADER_LEN) || !Verify_CRC16_Check_Sum(rx_data , UPC_TOTAL_LEN))
		return 4;

	uint16_t cmd_id = (rx_data[6] << 8) | rx_data[5];

	switch(cmd_id)
	{
		case MSG_IMU_INFO_ID:
			break;
		case MSG_MOVE_CMD_ID:
			upc_cmd_move_handler(&rx_data[UPC_HEADER_LEN+2]);
			break;
		case GIMBAL_ROTATION_ID:
			upc_cmd_gimbal_handler(&rx_data[UPC_HEADER_LEN+2]);
			break;
		case MSG_SHOOT_CMD_ID:
			upc_cmd_shoot_handler(&rx_data[UPC_HEADER_LEN+2]);
			break;
		case MSG_MODE_SWITCH_ID:
			upc_cmd_mode_handler(&rx_data[UPC_HEADER_LEN+2]);
			break;
		case MSG_SMALL_GIMBAL_IMU_INFO_ID:
			//comm_cmd_small_gimbal_imu_handler(&rx_data[UPC_HEADER_LEN+2]);
			break;
		case MSG_BIG_GIMBAL_IMU_INFO_ID:
			comm_cmd_big_gimbal_imu_handler(&rx_data[UPC_HEADER_LEN+2]);
			break;
		default:
			break;
	}
	return 0; 
}

void upc_cmd_move_handler(uint8_t* data)
{
	unpack_4bytes_to_floats(&data[0], &upc.vx);
	unpack_4bytes_to_floats(&data[4], &upc.vy);
	unpack_4bytes_to_floats(&data[8], &upc.vw);
}

void upc_cmd_gimbal_handler(uint8_t* data)
{
	unpack_4bytes_to_floats(&data[0], &upc.gimbal_yaw);
	unpack_4bytes_to_floats(&data[4], &upc.small_gimbal_yaw);
	unpack_4bytes_to_floats(&data[8], &upc.small_gimbal_pitch);
	uint8_t send_data[8];
	pack_float_to_4bytes(upc.small_gimbal_yaw, &send_data[0]);
	pack_float_to_4bytes(upc.small_gimbal_pitch, &send_data[4]);
	CAN_CBoard_CMD(0x222, send_data);
}

void upc_cmd_shoot_handler(uint8_t* data)
{
	uint8_t send_data[8] = {0};
	send_data[0] = data[12];
	CAN_CBoard_CMD(0x223, send_data);
	
}

void upc_cmd_mode_handler(uint8_t* data) // 暂时用于摩擦轮控制
{
	//upc.mode = data[12];
	uint8_t send_data[8] = {0};
	send_data[1] = data[12];
	CAN_CBoard_CMD(0x223, send_data);
}

// void comm_cmd_small_gimbal_imu_handler(uint8_t* data)
// {
// 	unpack_4bytes_to_floats(&data[0], &small_gimbal_angle_deg[0]);
// 	unpack_4bytes_to_floats(&data[4], &small_gimbal_angle_deg[1]);
// 	unpack_4bytes_to_floats(&data[8], &small_gimbal_angle_deg[2]);
// }

void comm_cmd_big_gimbal_imu_handler(uint8_t* data)
{
	unpack_4bytes_to_floats(&data[0], &big_gimbal_angle.big_gimbal_angle[0]);
	unpack_4bytes_to_floats(&data[4], &big_gimbal_angle.big_gimbal_angle[1]);
	unpack_4bytes_to_floats(&data[8], &big_gimbal_angle.big_gimbal_angle[2]);
	big_gimbal_angle.big_gimbal_imu_last_online_time = DWT_GetTimeline_s();
}

big_gimbal_angle_t* get_big_gimbal_angle(void)
{
	return &big_gimbal_angle;
}

upc_t* get_upc_data(void)
{
	return &upc;
}

void temp_imu_handler(uint8_t* data)
{
	// if (sizeof(data) != 16)
	// 	return;
	unpack_4bytes_to_floats(&data[0], &big_gimbal_angle.big_gimbal_angle[0]);
	unpack_4bytes_to_floats(&data[4], &big_gimbal_angle.big_gimbal_angle[1]);
	unpack_4bytes_to_floats(&data[8], &big_gimbal_angle.big_gimbal_angle[2]);
	big_gimbal_angle.big_gimbal_imu_last_online_time = DWT_GetTimeline_s();
}
