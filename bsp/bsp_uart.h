#ifndef UART_NEW
#define UART_NEW
#include "usart.h"

#define COMM_MESSAGE_LEN 22
#define DEBUG_MESSAGE_LEN 100

#define COMM_HUART huart1
#define DEBUG_HUART huart6 

void uart1_init(void);
void uart6_init(void);

#endif
