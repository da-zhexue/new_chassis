/**
 * @file CAN_rx.c
 * @brief CAN接收模块
 * 该文件用于处理CAN总线接收到的数据，并将其解析到相应的数据结构中。
 * 其中电机走can1，与云台c板、通信板、上位机通信走can2。
 * @version 1.1
 * @changelog
 * - 2026-02-09 适配新的数据中转模块
 */

#include "CAN_rx.h"
#include "can.h"
#include "user_lib.h"
#include "OMM.h"
#include "DTM.h"

/**********************************************************************/
void get_motor_3508_measure(const uint8_t motor, const uint8_t* rx_data)
{
	static m3508_t motor_3508_measure[4];
    if(rx_data == NULL)
        return;
    motor_3508_measure[motor].last_ecd = (int16_t)motor_3508_measure[motor].ecd;
    motor_3508_measure[motor].ecd = (uint16_t)((rx_data)[0] << 8 | (rx_data)[1]);
    motor_3508_measure[motor].speed = (int16_t)((rx_data)[2] << 8 | (rx_data)[3]);
    motor_3508_measure[motor].current = (int16_t)((rx_data)[4] << 8 | (rx_data)[5]);
    motor_3508_measure[motor].temperature = (rx_data)[6];
	DTM_Write(M3508_DATA, motor_3508_measure, sizeof(motor_3508_measure));
    OMM_update(M3508_0_ONLINE + motor);
}

void CAN_9025_MeasureProcess(const uint8_t* rx_data)
{
	static m9025_t motor_9025_measure;
    if(rx_data == NULL)
        return;

    switch(rx_data[0])
    {
        case CMD_9025_READ_MEASURE: //read measure
        case CMD_9025_SPEED_CONTROL:
        case CMD_9025_ANGLE_CONTROL:
        case CMD_9025_INCREMENT_ANGLE_CONTROL:
            motor_9025_measure.temperate = (int8_t)   (rx_data)[1];
            motor_9025_measure.iq        = (int16_t)  ((rx_data)[3]<<8 | (rx_data)[2]);
            motor_9025_measure.speed     = (int16_t)  ((rx_data)[5]<<8 | (rx_data)[4]);
            motor_9025_measure.ecd       = (uint16_t) ((rx_data)[7]<<8 | (rx_data)[6]);
    		if (motor_9025_measure.ecd_offset == 0)
    		{
    			motor_9025_measure.ecd_offset = motor_9025_measure.ecd;
    			motor_9025_measure.imu_yaw_offset = theta_format((fp32)(motor_9025_measure.ecd - MF9025_ECD_IN_ZERO)/ 32768.0f * 180.0f);
    		}
            OMM_update(M9025_ONLINE);
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
	DTM_Write(M9025_DATA, &motor_9025_measure, sizeof(motor_9025_measure));
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
                    get_motor_3508_measure(i, rx_data);
                    break;
            }

			case CAN_9025_M1_RX_ID:
			{
				CAN_9025_MeasureProcess(rx_data);
				break;
			}

			default: break;
		}
	}
	else if (hcan -> Instance == CAN2)
	{
		//static fp32 gimbal_s_ptr[3];
		switch (rx_header.StdId)
		{
			// case CBOARD_GIMBAL_1:
			// {
			// 	unpack_4bytes_to_floats(&rx_data[0], &gimbal_s_ptr[0]);
			// 	unpack_4bytes_to_floats(&rx_data[4], &gimbal_s_ptr[1]);
			// 	DTM_Write(GIMBAL_S_DATA, gimbal_s_ptr, sizeof(gimbal_s_ptr));
			// 	OMM_update(GIMBAL_S_DATA);
			// 	break;
			// }
   //
			// case CBOARD_GIMBAL_2:
			// {
   //              unpack_4bytes_to_floats(&rx_data[0], &gimbal_s_ptr[2]);
			// 	DTM_Write(GIMBAL_S_DATA, gimbal_s_ptr, sizeof(gimbal_s_ptr));
   //              OMM_update(GIMBAL_S_DATA);
			// 	break;
			// }

			default:
				break;
		}
	}
}
