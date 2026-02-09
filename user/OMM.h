#ifndef OMM_H
#define OMM_H

#include "typedef.h"

typedef enum
{
    RC_ONLINE = 0,
    UPC_ONLINE = 1,
    GIMBAL_S_ONLINE = 2,
    GIMBAL_L_ONLINE = 3,
    M3508_0_ONLINE = 4,
    M3508_1_ONLINE = 5,
    M3508_2_ONLINE = 6,
    M3508_3_ONLINE = 7,
    M9025_ONLINE = 8,
    REFEREE_ONLINE = 9,
    ONLINE_COUNT = 10
} online_online_type;

extern uint16_t on_code;
#define SET_ONLINE(type)  (on_code |= (1 << (type)))
#define SET_OFFLINE(type) (on_code &= ~(1 << (type)))
#define IS_ONLINE(type)   ((on_code & (1 << (type))) ? 1 : 0)

void OMM_update(uint16_t type);
uint8_t OMM_detect(uint16_t type);

#endif
