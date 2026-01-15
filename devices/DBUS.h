#ifndef CON
#define CON

#include "usart.h"
#include "typedef.h"
#include "data_transfer.h"

void RemoteDataProcess(uint8_t *pData);

void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);

#endif
