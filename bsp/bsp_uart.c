#include "bsp_uart.h"
#include "COMM_rec.h"
#include "cmsis_os.h"
#include "DBUS.h"
#include "comm_task.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
uint8_t uart1_rx_buffer[COMM_MESSAGE_LEN];
uint8_t uart6_rx_buffer[DEBUG_MESSAGE_LEN];

void uart1_init(void) // COMMUNICATION_BOARD
{
	HAL_UARTEx_ReceiveToIdle_DMA(&COMM_HUART, uart1_rx_buffer, sizeof(uart1_rx_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}	

void uart6_init(void) // DEBUG
{
	HAL_UARTEx_ReceiveToIdle_DMA(&DEBUG_HUART, uart6_rx_buffer, sizeof(uart6_rx_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) //DMA
{
	if(huart == &COMM_HUART)
	{
		upc_decode(uart1_rx_buffer);
		HAL_UARTEx_ReceiveToIdle_DMA(&COMM_HUART, uart1_rx_buffer, sizeof(uart1_rx_buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
	else if(huart == &DEBUG_HUART)
	{
		temp_imu_handler(uart6_rx_buffer);
		HAL_UARTEx_ReceiveToIdle_DMA(&DEBUG_HUART, uart6_rx_buffer, sizeof(uart6_rx_buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
	}
}
