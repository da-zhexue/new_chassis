#include "led_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "Online_Monitor.h"

#define RGB_FLOW_COLOR_CHANGE_TIME  500
#define RGB_FLOW_COLOR_LENGHT   6
//blue-> green(dark)-> red -> blue(dark) -> green(dark) -> red(dark) -> blue

uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};
uint8_t show_online_state(void);

void LedTask(void const * argument)
{
	fp32 delta_alpha, delta_red, delta_green, delta_blue;
	fp32 alpha,red,green,blue;
	uint32_t aRGB;
	aRGB_led_init();
	while(1)
	{
		if(show_online_state() == 0)
			for(int i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
				{
					alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
					red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
					green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
					blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

					delta_alpha = (fp32)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (fp32)((RGB_flow_color[i] & 0xFF000000) >> 24);
					delta_red = (fp32)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (fp32)((RGB_flow_color[i] & 0x00FF0000) >> 16);
					delta_green = (fp32)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (fp32)((RGB_flow_color[i] & 0x0000FF00) >> 8);
					delta_blue = (fp32)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

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

							aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
							aRGB_led_show(aRGB);
							osDelay(3);
					}
			 	}
	}
}

uint8_t show_online_state(void)
{
	uint16_t error_code = error_code_get();
	if(error_code == 0x03FF)
		return 0;
	if(!(error_code & 0x0001)) {aRGB_led_shine(0xFFFF0000, 1); osDelay(1000);}
	if(!(error_code & 0x0002)) {aRGB_led_shine(0xFFFF0000, 2); osDelay(1000);}
	if(!(error_code & 0x0004)) {aRGB_led_shine(0xFFFF0000, 3); osDelay(1000);}
	if(!(error_code & 0x0008)) {aRGB_led_shine(0xFF00FF00, 1); osDelay(1000);}
	if(!(error_code & 0x0010)) {aRGB_led_shine(0xFF00FF00, 2); osDelay(1000);}
	if(!(error_code & 0x0020)) {aRGB_led_shine(0xFF00FFFF, 1); osDelay(1000);}
	if(!(error_code & 0x0040)) {aRGB_led_shine(0xFF00FFFF, 2); osDelay(1000);}
	if(!(error_code & 0x0080)) {aRGB_led_shine(0xFF00FFFF, 3); osDelay(1000);}
	if(!(error_code & 0x0100)) {aRGB_led_shine(0xFF00FFFF, 4); osDelay(1000);}
	if(!(error_code & 0x0200)) {aRGB_led_shine(0xFF00FFFF, 5); osDelay(1000);}
	return 1;
}
