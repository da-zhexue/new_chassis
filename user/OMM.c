/**
 * @file OMM.c
 * @brief 在线监测模块(Online Monitor Module)
 * 通过将上次接收到数据的时间与当前时间进行对比获得电机、imu、遥控器等的在线情况，并获得错误码。
 * @version 2.0
 * @changelog
 * - 2026-02-09 将上次在线时间由存放在每个数据结构体中改为均存在last_on数组中，简化代码
 */

#include "OMM.h"
#include "bsp_dwt.h"

static uint64_t last_on[ONLINE_COUNT];
uint16_t on_code = 0;

void OMM_update(const uint16_t type)
{
    last_on[type] = DWT_GetTimeline_us();
	SET_ONLINE(type);
}

uint8_t OMM_detect(const uint16_t type)
{
	if (DWT_GetTimeline_us() - last_on[type] > 2000000)
    {
		SET_OFFLINE(type);
        return 0;
    }
	SET_ONLINE(type);
	return 1;
}
