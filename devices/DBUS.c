/**
 * @file DBUS.c
 * @brief 遥控器模块
 * 获取遥控器数据。
 * @version 1.0
 * @date 2026-01-17
 */

#include "main.h"
#include "DBUS.h"
#include "DTM.h"
#include "OMM.h"
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01 << 3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01 << 4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01 << 5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01 << 6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01 << 7)
#define RC_FRAME_LENGTH 18u

volatile unsigned char sbus_rx_buffer[RC_FRAME_LENGTH]; // double sbus rx buffer to save data

void RemoteDataProcess(uint8_t *pData)
{
	static rc_t rc_ptr;
    if (pData == NULL)
    {
        return;
    }
    rc_ptr.ch0 = (int16_t)(((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF);
    rc_ptr.ch1 = (int16_t)((((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF);
    rc_ptr.ch2 = (int16_t)((((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF);
    rc_ptr.ch3 = (int16_t)((((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF);
	rc_ptr.ch0 -= RC_CH_VALUE_OFFSET;
	rc_ptr.ch1 -= RC_CH_VALUE_OFFSET;
	rc_ptr.ch2 -= RC_CH_VALUE_OFFSET;
	rc_ptr.ch3 -= RC_CH_VALUE_OFFSET;
	rc_ptr.s1 = ((pData[5] >> 4) & 0x0003);
    rc_ptr.s2 = ((pData[5] >> 4) & 0x000C) >> 2;
     // | ((int16_t)pData[15] << 8);
	DTM_Write(RC_DATA, &rc_ptr, sizeof(rc_t));
	OMM_update(RC_ONLINE);

}

#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart3
uint8_t   dbus_buf[DBUS_BUFLEN];
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
	uint32_t tmp1 = 0;
	tmp1 = huart->RxState;
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}
 
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;
 
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

void dbus_uart_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}

uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  return ((uint16_t)(dma_stream->NDTR));
}
 
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	if (huart == &DBUS_HUART)
	{
		__HAL_DMA_DISABLE(huart->hdmarx);
		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{
            RemoteDataProcess(dbus_buf);
		}
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

void uart_receive_handler(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}
