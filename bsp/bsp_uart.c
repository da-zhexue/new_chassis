#include "bsp_uart.h"
#include "COMM_rec.h"
#include "DBUS.h"
#include "param_tuning.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
static uint8_t uart1_rx_buffer[2][COMM_BUF_LEN];
static uint8_t uart6_rx_buffer[2][DEBUG_BUF_LEN];
void uart1_init(void) // COMMUNICATION_BOARD
{
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }
    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(uart1_rx_buffer[0]);
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(uart1_rx_buffer[1]);
    hdma_usart1_rx.Instance->NDTR = COMM_BUF_LEN;
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}	

void usart1_rec_handler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart1_rx);
            this_time_rx_len = COMM_BUF_LEN - hdma_usart1_rx.Instance->NDTR;
            hdma_usart1_rx.Instance->NDTR = COMM_BUF_LEN;
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart1_rx);
            if(this_time_rx_len > 9)
            {
                upc_decode(uart1_rx_buffer[0]);
            }
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart1_rx);
            this_time_rx_len = COMM_BUF_LEN - hdma_usart1_rx.Instance->NDTR;
            hdma_usart1_rx.Instance->NDTR = COMM_BUF_LEN;
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart1_rx);
            if(this_time_rx_len > 9)
            {
                upc_decode(uart1_rx_buffer[1]);
            }
        }
    }
}

void uart6_init(void) // DEBUG
{
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }
    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(uart6_rx_buffer[0]);
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(uart6_rx_buffer[1]);
    hdma_usart6_rx.Instance->NDTR = DEBUG_BUF_LEN;
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}

void usart6_rec_handler(void)
{
    if(huart6.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = DEBUG_BUF_LEN - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = DEBUG_BUF_LEN;
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            if(this_time_rx_len == DEBUG_MSG_LEN)
            {
                param_decode(uart6_rx_buffer[0]);
            }
        }
        else
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = DEBUG_BUF_LEN - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = DEBUG_BUF_LEN;
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if(this_time_rx_len == DEBUG_MSG_LEN)
            {
                param_decode(uart6_rx_buffer[1]);
            }
        }
    }
}
