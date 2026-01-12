#ifndef CON
#define CON

#include "usart.h"
#include "typedef.h"
void RemoteDataProcess(uint8_t *pData);
typedef struct __RC__
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s1;
    uint8_t s2;
    uint8_t sw2;
	
	fp32 last_online_time;
} rc_ctrl_t;

void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
rc_ctrl_t *get_rc_ctrl_data(void);
#endif
