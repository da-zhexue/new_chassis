#include "bsp_led.h"
#include "main.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim5;
/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
	
void aRGB_led_init(void)
{
		HAL_TIM_Base_Start(&htim5);
		HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}
/**
  * @brief          ��ʾRGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' ��͸����,'RR'�Ǻ�ɫ,'GG'����ɫ,'BB'����ɫ
  * @retval         none
  */
void aRGB_led_show(uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}

void aRGB_led_shine(uint32_t aRGB, uint16_t Hz)
{
    for(int i = 0; i < Hz; i++)
    {
        aRGB_led_show(aRGB);
        osDelay(1000 / Hz / 2);
        aRGB_led_show(0x00000000);
        osDelay(1000 / Hz / 2);
    }
}
