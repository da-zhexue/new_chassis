#include "bsp_uart.h"
#include "COMM_rec.h"
#include "cmsis_os.h"
#include "DBUS.h"
#include "comm_task.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
static uint8_t uart1_rx_buffer[2][COMM_BUF_LEN];
uint8_t uart6_rx_buffer[DEBUG_MESSAGE_LEN];

void uart1_init(void) // COMMUNICATION_BOARD
{
	//enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(uart1_rx_buffer[0]);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(uart1_rx_buffer[1]);
    //data length
    //数据长度
    hdma_usart1_rx.Instance->NDTR = COMM_BUF_LEN;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
//	HAL_UARTEx_ReceiveToIdle_DMA(&COMM_HUART, uart1_rx_buffer, sizeof(uart1_rx_buffer));
//	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}	

void USART1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = COMM_BUF_LEN - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = COMM_BUF_LEN;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == COMM_MESSAGE_LEN)
            {
                upc_decode(uart1_rx_buffer[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = COMM_BUF_LEN - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = COMM_BUF_LEN;

            //set memory buffer 0
            //设定缓冲区0
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == COMM_MESSAGE_LEN)
            {
                //处理遥控器数据
                upc_decode(uart1_rx_buffer[1]);
            }
        }
    }
}

void uart6_init(void) // DEBUG
{
	HAL_UARTEx_ReceiveToIdle_DMA(&DEBUG_HUART, uart6_rx_buffer, sizeof(uart6_rx_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) //DMA
{
//	if(huart == &COMM_HUART)
//	{
//		upc_decode(uart1_rx_buffer);
//		HAL_UARTEx_ReceiveToIdle_DMA(&COMM_HUART, uart1_rx_buffer, sizeof(uart1_rx_buffer));
//		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
//	}
//	else 
	if(huart == &DEBUG_HUART)
	{
		temp_imu_handler(uart6_rx_buffer);
		HAL_UARTEx_ReceiveToIdle_DMA(&DEBUG_HUART, uart6_rx_buffer, sizeof(uart6_rx_buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
	}
}
