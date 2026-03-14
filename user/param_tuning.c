#include "param_tuning.h"

#include <string.h>

#include "bsp_uart.h"
#include "chassis_ctrl.h"
#include "usart.h"
#include "DTM.h"
#include "user_lib.h"
#include "CAN_tx.h"

#ifdef DEBUG_MODE
extern chassis_t chassis_ptr;
extern int32_t mf9025_given_speed;
void tx_handler(void);

void param_tuning_Task(void const *argument)
{
    uart6_init();
    while(1) {
        tx_handler();
        osDelay(1);
    }
}

void tx_handler(void)
{
    static uint8_t tx_buf[SEND_TOTAL_SIZE];
    static TF_t tf_ptr;
    static m9025_t m9025_ptr;
    DTM_Read(TF_DATA, &tf_ptr, sizeof(tf_ptr));
    DTM_Read(M9025_DATA, &m9025_ptr, sizeof(m9025_ptr));
    pack_float_to_4bytes(theta_format(-tf_ptr.Big_Gimbal_angle.yaw_deg), &tx_buf[0]);
    pack_float_to_4bytes(theta_format(chassis_ptr.given_gimbal_l_yaw), &tx_buf[4]);
    pack_float_to_4bytes(m9025_ptr.speed, &tx_buf[8]);
    pack_float_to_4bytes((fp32)mf9025_given_speed, &tx_buf[12]);

    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    memcpy(&tx_buf[SEND_DATA_LEN], tail, sizeof(tail));
    HAL_UART_Transmit(&PARAM_HUART, tx_buf, sizeof(tx_buf), 100);
}

void param_decode(uint8_t *rx_buf)
{
    static float param_data[RECV_PARAM_NUM];
    static uint8_t ready = 0;
    if(rx_buf[0] != PARAM_HEADER || rx_buf[3] != 0 || rx_buf[4] != 0x5A)
		return ; 
	if(rx_buf[1] != RECV_DATA_LEN)
		return ; 
	// if(!Verify_CRC8_Check_Sum(rx_buf, PARAM_HEADER_LEN) || !Verify_CRC16_Check_Sum(rx_buf , RECV_TOTAL_SIZE))
	// 	return ;
    switch (rx_buf[5])
    {
        case 0x01:
            unpack_4bytes_to_floats(&rx_buf[7], &param_data[0]);
            ready |= 0x01;
            break;
        case 0x02:
            unpack_4bytes_to_floats(&rx_buf[7], &param_data[1]);
            ready |= 0x02;
            break;
        case 0x03:
            unpack_4bytes_to_floats(&rx_buf[7], &param_data[2]);
            ready |= 0x04;
            break;
        case 0x04:
            unpack_4bytes_to_floats(&rx_buf[7], &param_data[3]);
            ready |= 0x08;
            break;
        case 0x05:
            unpack_4bytes_to_floats(&rx_buf[7], &param_data[4]);
            ready |= 0x10;
            break;
        case 0x06:
            unpack_4bytes_to_floats(&rx_buf[7], &param_data[5]);
            ready |= 0x20;
            break;
        case 0x07:
            CAN_Control9025Speed(CAN_9025_M1_TX_ID, MF9025_MAX_IQ, 0);
             __HAL_RCC_CLEAR_RESET_FLAGS();
            osDelay(100);
            HAL_NVIC_SystemReset();
            break;

        default:
            break;
    }
    if (ready == 0x3F)
    {
        DTM_Write(PARAM_DATA, param_data, sizeof(param_data));
        DTM_Write(FLAG_DATA, &ready, sizeof(ready));
        ready = 0;
    }
}
#else
void param_decode(uint8_t *rx_buf)
{
    ;
}
#endif