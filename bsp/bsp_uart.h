#ifndef BSP_UART_H
#define BSP_UART_H

#define COMM_MSG_LEN 64
#define DEBUG_MSG_LEN 64

#define COMM_BUF_LEN (COMM_MSG_LEN*2)
#define DEBUG_BUF_LEN (DEBUG_MSG_LEN*2)

#define COMM_HUART huart1
#define DEBUG_HUART huart6 

void uart1_init(void);
void uart6_init(void);
void usart1_rec_handler(void);
void usart6_rec_handler(void);

#endif
