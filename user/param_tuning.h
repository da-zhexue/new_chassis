#ifndef PARAM_TUNING_H_
#define PARAM_TUNING_H_

#include "typedef.h"

#define PARAM_TUNING_ENABLE 

#define PARAM_HUART huart6
#define PARAM_HEADER 0xA5
#define PARAM_HEADER_LEN 5
#define RECV_PARAM_NUM 3
#define SEND_PARAM_NUM 3

#define RECV_DATA_LEN (RECV_PARAM_NUM*4)
#define SEND_DATA_LEN (SEND_PARAM_NUM*4)
#define RECV_TOTAL_SIZE (9+RECV_DATA_LEN)
#define SEND_TOTAL_SIZE (9+SEND_DATA_LEN)

void param_tuning_Task(void const *argument);
void param_decode(uint8_t *rx_buf);

#endif
