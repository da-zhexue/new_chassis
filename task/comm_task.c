#include "comm_task.h"
#include "cmsis_os.h"
#include "COMM_rec.h"
#include "TF_task.h"
#include "usart.h"
#include "user_lib.h"
#include "crc.h"
#include "DBUS.h"

TF_t *TF_ptr;
rc_ctrl_t *rc_ptr;

void comm_init(void);
void upc_send_attitude_handler(void);
void debug(void);

void commTask(void const * argument)
{
		comm_init();
    while(1)
    {
        while((&huart1)->gState != HAL_UART_STATE_READY)
            ; 
        //upc_send_attitude_handler();
				//debug();
        osDelay(5);
    }
}

void comm_init(void)
{
	TF_ptr = get_TF();
	rc_ptr = get_rc_ctrl_data();
}

void upc_send_attitude_handler(void)
{
	static uint8_t send_data[UPC_TOTAL_LEN] = {0};
	send_data[0] = UPC_HEADER;
	send_data[1] = UPC_DATA_LEN;
	send_data[2] = 0;send_data[3] = 0;
	send_data[5] = SEND_ATTITUDE_ID & 0xFF;
	send_data[6] = (SEND_ATTITUDE_ID >> 8) & 0xFF;

	pack_float_to_4bytes(TF_ptr->Chassis_angle.yaw_deg, &send_data[7]);
	pack_float_to_4bytes(TF_ptr->Small_Gimbal_angle.yaw_deg, &send_data[11]);
	pack_float_to_4bytes(TF_ptr->Small_Gimbal_angle.pitch_deg, &send_data[15]);

	Append_CRC8_Check_Sum(send_data, UPC_HEADER_LEN);
	Append_CRC16_Check_Sum(send_data, UPC_TOTAL_LEN);
	HAL_UART_Transmit_DMA(&huart1, send_data, sizeof(send_data));
}

void debug(void)
{
	static uint8_t send_data[10] = {0};
	send_data[0] = rc_ptr->ch0 & 0xFF;
	send_data[1] = (rc_ptr->ch0 >> 8)& 0xFF;
	send_data[2] = rc_ptr->ch1 & 0xFF;
	send_data[3] = (rc_ptr->ch1 >> 8)& 0xFF;
	send_data[4] = rc_ptr->ch2 & 0xFF;
	send_data[5] = (rc_ptr->ch2 >> 8)& 0xFF;
	send_data[6] = rc_ptr->ch3 & 0xFF;
	send_data[7] = (rc_ptr->ch3 >> 8)& 0xFF;
	
	send_data[8] = rc_ptr->s1;
	send_data[9] = rc_ptr->s2;
	HAL_UART_Transmit_DMA(&huart1, send_data, sizeof(send_data));
}
