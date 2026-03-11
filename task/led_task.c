/**
 * @file led_task.c
 * @brief LED任务模块
 * 通过LED灯的颜色变化来显示系统状态。
 * @version 1.1
 * @date 2026-02-09
 * @changelog
 * - 2026-02-09 适配新的在线监控模块
 */

#include "led_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "OMM.h"
#include "ulog.h"

#define RGB_FLOW_COLOR_CHANGE_TIME  500
#define RGB_FLOW_COLOR_LENGTH   6
//blue-> green(dark)-> red -> blue(dark) -> green(dark) -> red(dark) -> blue

uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGTH + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};
uint8_t show_online_state(void);

void LedTask(void const * argument)
{
	aRGB_led_init();
	while(1)
	{
		if(show_online_state() == 0)
			for(int i = 0; i < RGB_FLOW_COLOR_LENGTH; i++)
				{
					fp32 alpha = (fp32)((RGB_flow_color[i] & 0xFF000000) >> 24);
					fp32 red = (fp32)((RGB_flow_color[i] & 0x00FF0000) >> 16);
					fp32 green = (fp32)((RGB_flow_color[i] & 0x0000FF00) >> 8);
					fp32 blue = (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

					fp32 delta_alpha = (fp32)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (fp32)((RGB_flow_color[i] & 0xFF000000) >>
						24);
					fp32 delta_red = (fp32)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (fp32)((RGB_flow_color[i] & 0x00FF0000) >>
						16);
					fp32 delta_green = (fp32)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (fp32)((RGB_flow_color[i] & 0x0000FF00) >>
						8);
					fp32 delta_blue = (fp32)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

					delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
					delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
					delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
					delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
					for(int j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
					{
							alpha += delta_alpha;
							red += delta_red;
							green += delta_green;
							blue += delta_blue;

							const uint32_t aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue))
								<< 0;
							aRGB_led_show(aRGB);
							osDelay(3);
					}
			 	}
	}
}

const uint32_t off_led_aRGB[ONLINE_COUNT] = {0xFFFF0000, 0xFFFF0000, 0xFF00FF00,
	0xFF00FFFF, 0xFF00FFFF, 0xFF00FFFF, 0xFF00FFFF, 0xFF00FFFF};
const uint16_t off_led_Hz[ONLINE_COUNT] = {1, 2, 1, 1, 2, 3, 4, 5};
uint8_t show_online_state(void)
{
	uint8_t error_code = 0;
	for(int i = 0; i < ONLINE_COUNT; i++)
	{
		if(!IS_ONLINE(i))
		{
			aRGB_led_shine(off_led_aRGB[i], off_led_Hz[i]);
			LOG_WARN("The device is offline: %d", i);
			osDelay(1000);
			error_code = 1;
		}
	}
	return error_code;
}
