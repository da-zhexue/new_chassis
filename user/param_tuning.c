#include "param_tuning.h"
#include "bsp_uart.h"
#include "chassis_ctrl.h"
#include "usart.h"
#include "crc.h"
#include "DTM.h"
#include "user_lib.h"

#ifdef DEBUG_MODE
extern chassis_t chassis_ptr;
extern m9025_ctrl_t m9025_ctrl;
void tx_handler(void);

void param_tuning_Task(void const *argument)
{
    uart6_init();
    while(1) {
        tx_handler();
        osDelay(5);
    }
}

void tx_handler(void)
{
    static uint8_t tx_buf[SEND_TOTAL_SIZE];
    static TF_t tf_ptr;
    DTM_Read(TF_DATA, &tf_ptr, sizeof(tf_ptr));
    tx_buf[0] = PARAM_HEADER; tx_buf[1] = SEND_DATA_LEN; 
    tx_buf[2] = 0x00; tx_buf[3] = 0x00;
    tx_buf[5] = 0x00; tx_buf[6] = 0x03;

    pack_float_to_4bytes(tf_ptr.Big_Gimbal_angle.yaw_deg, &tx_buf[7]);
    pack_float_to_4bytes(chassis_ptr.given_gimbal_l_yaw, &tx_buf[11]);
    pack_float_to_4bytes(m9025_ctrl.given_angle, &tx_buf[15]);
    Append_CRC8_Check_Sum(tx_buf, PARAM_HEADER_LEN);
    Append_CRC16_Check_Sum(tx_buf, SEND_TOTAL_SIZE);

    HAL_UART_Transmit_DMA(&PARAM_HUART, tx_buf, sizeof(tx_buf));
}

void param_decode(uint8_t *rx_buf)
{
    static float param_data[RECV_PARAM_NUM];
    static uint8_t ready = 0;
    if(rx_buf[0] != PARAM_HEADER || rx_buf[3] != 0)
		return ; 
	if(rx_buf[1] != RECV_DATA_LEN)
		return ; 
	if(!Verify_CRC8_Check_Sum(rx_buf, PARAM_HEADER_LEN) || !Verify_CRC16_Check_Sum(rx_buf , RECV_TOTAL_SIZE))
		return ;

    unpack_4bytes_to_floats(&rx_buf[7], &param_data[0]);
	unpack_4bytes_to_floats(&rx_buf[11], &param_data[1]);
	unpack_4bytes_to_floats(&rx_buf[15], &param_data[2]);
    ready = 1;
    DTM_Write(PARAM_DATA, param_data, sizeof(param_data));
    DTM_Write(FLAG_DATA, &ready, sizeof(ready));
}
#else
void param_decode(uint8_t *rx_buf)
{
    ;
}
#endif