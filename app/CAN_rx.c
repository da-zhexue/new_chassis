/**
 * @file CAN_rx.c
 * @brief CAN接收模块
 * 该文件用于处理CAN总线接收到的数据，并将其解析到相应的数据结构中。
 * 其中电机走can1，与云台c板、通信板、上位机通信走can2。
 * @version 1.0
 * @date 2026-01-17
 */

#include "CAN_rx.h"
#include "can.h"
#include "user_lib.h"
#include "bsp_dwt.h"
#include "data_transfer.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**********************************************************************/
motor_9025_measure_t* motor_9025_measure_ptr = NULL;
motor_3508_measure_t* motors_3508_measure_ptr[4] = {NULL};
small_gimbal_angle_t* small_gimbal_angle_deg_ptr = NULL;
void CAN_Receive_Init(void)
{
    motor_9025_measure_ptr = get_motor_9025_measure_data();
    for(int i = 0; i < 4; i++){
        motors_3508_measure_ptr[i] = get_motor_3508_measure_data(i);
    }
    small_gimbal_angle_deg_ptr = get_small_gimbal_angle();
}

void get_motor_3508_measure(motor_3508_measure_t* motor_3508_measure, uint8_t* rx_data)
{
    if(rx_data == NULL || motor_3508_measure == NULL)
    {
        return;
    }
    motor_3508_measure->last_ecd = motor_3508_measure->ecd;
    motor_3508_measure->ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);
    motor_3508_measure->speed = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]);
    motor_3508_measure->current = (uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);
    motor_3508_measure->temperature = (rx_data)[6];
    motor_3508_measure->last_online = DWT_GetTimeline_s();
}

void CAN_9025_MeasureProcess(motor_9025_measure_t* motor_9025_measure, uint8_t* rx_data)
{
    if(rx_data == NULL || motor_9025_measure == NULL)
    {
        return;
    }
    switch(rx_data[0])
    {
        case CMD_9025_READ_MEASURE: //read measure
        case CMD_9025_SPEED_CONTROL:
        case CMD_9025_ANGLE_CONTROL:
        case CMD_9025_INCREMENT_ANGLE_CONTROL:
            motor_9025_measure->temperate = (int8_t)   (rx_data)[1];
            motor_9025_measure->iq        = (int16_t)  ((rx_data)[3]<<8 | (rx_data)[2]);
            motor_9025_measure->speed     = (int16_t)  ((rx_data)[5]<<8 | (rx_data)[4]);
            motor_9025_measure->ecd       = (uint16_t) ((rx_data)[7]<<8 | (rx_data)[6]);
            motor_9025_measure->last_online = DWT_GetTimeline_s();
            break;
        case CMD_9025_READ_ENCODER: //read encoder
            //get_motor_9025_ecd_data(motor_9025->motor_9025_ecd_data, rx_data); 
            //该数据需要发送指令读取，即使发送指令不知为何在非调试模式下无法更新数据
            break;
        case CMD_9025_READ_CONTROL_PARAM: //read control param
            //get_motor_9025_control_param(motor_9025->motor_9025_pid, rx_data);
            break;
        default:
            break;
    }
}

static uint8_t rx_data[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		if(hcan -> Instance == CAN1)
		{
			switch (rx_header.StdId)
			{
				case CAN_3508_M1_ID:
                case CAN_3508_M2_ID:
                case CAN_3508_M3_ID:
                case CAN_3508_M4_ID:
                {
                        static uint8_t i = 0;
                        //get motor id
                        i = rx_header.StdId - CAN_3508_M1_ID;
                        get_motor_3508_measure(motors_3508_measure_ptr[i], rx_data);
                        break;
                }

				case CAN_9025_M1_RX_ID:
				{
					CAN_9025_MeasureProcess(motor_9025_measure_ptr, rx_data);
					break;
				}
						
				default: break;	
			}
		}
		else if (hcan -> Instance == CAN2)
		{
				switch (rx_header.StdId)
				{
					case CBOARD_GIMBAL_1:
					{
                        unpack_4bytes_to_floats(&rx_data[0], &small_gimbal_angle_deg_ptr->small_gimbal_angle[0]);
                        unpack_4bytes_to_floats(&rx_data[4], &small_gimbal_angle_deg_ptr->small_gimbal_angle[1]);
                        small_gimbal_angle_deg_ptr->small_gimbal_imu_last_online_time = DWT_GetTimeline_s();
						break;
					}
					
					case CBOARD_GIMBAL_2:
					{
                        unpack_4bytes_to_floats(&rx_data[0], &small_gimbal_angle_deg_ptr->small_gimbal_angle[2]);
                        small_gimbal_angle_deg_ptr->small_gimbal_imu_last_online_time = DWT_GetTimeline_s();
						break;
					}
					
					default:
						break;
				}
		}
}
