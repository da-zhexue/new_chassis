#include "main.h"
#include "DBUS.h"
#include "bsp_dwt.h"
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
/* ----------------------- Data Struct ------------------------------------- */

/* ----------------------- Internal Data ----------------------------------- */
volatile unsigned char sbus_rx_buffer[RC_FRAME_LENGTH]; // double sbus rx buffer to save data
rc_ctrl_t RC_CtrlData;
/* ----------------------- Function Implements ---------------------------- */
/******************************************************************************
* @fn RC_Init
*
* @brief configure stm32 usart2 port
* - USART Parameters
* - 100Kbps
* - 8-N-1
* - DMA Mode
*
* @return None.
*
* @note This code is fully tested on STM32F405RGT6 Platform, You can port
it
* to the other platform. Using doube buffer to receive data prevent
losing data.
*/

/******************************************************************************
 * @fn RemoteDataProcess
 *
 * @brief resolution rc protocol data.
 * @pData a point to rc receive buffer.
 * @return None.
 * @note RC_CtrlData is a global variable.you can deal with it in other place.
 */
void RemoteDataProcess(uint8_t *pData)
{
    if (pData == NULL)
    {
        return;
    }

    RC_CtrlData.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    RC_CtrlData.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                          ((int16_t)pData[4] << 10)) &
                         0x07FF;
    RC_CtrlData.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) &
                         0x07FF;
	RC_CtrlData.ch0 -= RC_CH_VALUE_OFFSET;
	RC_CtrlData.ch1 -= RC_CH_VALUE_OFFSET;
	RC_CtrlData.ch2 -= RC_CH_VALUE_OFFSET;
	RC_CtrlData.ch3 -= RC_CH_VALUE_OFFSET;
		RC_CtrlData.s1 = ((pData[5] >> 4) & 0x0003);
    RC_CtrlData.s2 = ((pData[5] >> 4) & 0x000C) >> 2;
     // | ((int16_t)pData[15] << 8);
    RC_CtrlData.last_online_time = DWT_GetTimeline_s();
}
/******************************************************************************
* @fn USART2_IRQHandler
*
* @brief USART2 irq, we are care of ilde interrupt that means receiving the
one frame datas is finished.
*
* @return None.
*
* @note This code is fully tested on STM32F405RGT6 Platform, You can port
it
* to the other platform.
*/


#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart3
uint8_t   dbus_buf[DBUS_BUFLEN];
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{//用于接收数据的函数，使用DMA方式来接收UART数据
  uint32_t tmp1 = 0;
  tmp1 = huart->RxState;
	//创建一个临时变量tmp1，并将UART的接收状态赋值给它
	if (tmp1 == HAL_UART_STATE_READY)//判断UART是否处于就绪状态
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}
 
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;
 
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		//启动DMA接收，源地址为UART数据寄存器，目标地址为接收缓冲区
	
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		//使能UART的DMA接收功能
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

void dbus_uart_init(void)//DBUS串口初始化
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);//用于清除UART的空闲标志，空闲标志指示UART在接收数据时处于空闲状态，通常在接收完成后设置
	//清除这个标志是为了确保后续的接收操作能够正确检测到新的空闲状态
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);//使能UART的空闲中断，当UART处于空闲状态并且接收缓冲区没有数据时，会触发这个中断
	//使能这个中断后，可以在中断服务例程中处理空闲状态
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);//调用之前的函数，用DMA来接收串口数据
}

uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{//返回DMA预定义的缓冲区剩余的长度，方便了解传输过程中还有多少数据尚未传输
  return ((uint16_t)(dma_stream->NDTR));
}
 
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	//清除UART的空闲标志，以便下一次接收时能够正确检测到空闲状态
	
	if (huart == &DBUS_HUART)//确保只处理DBUS串口
	{
		__HAL_DMA_DISABLE(huart->hdmarx);//失能DMA接收，防止下一次接收的数据在上一次数据的尾部，而不是全新的数据
 
		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{//计算当前接收的数据长度，如果接收到的数据长度等于18字节，则调用处理数据函数
            RemoteDataProcess(dbus_buf);    	//处理接收的数据并解码
		}
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);//设置DMA接收预定义的缓冲区的长度，以便为下一次接收做好准备
		__HAL_DMA_ENABLE(huart->hdmarx);//重新启用DMA接收，以便继续接收数据
	}
}

void uart_receive_handler(UART_HandleTypeDef *huart)
{//用于检查UART接收状态并在接收到空闲状态时调用相应的回调函数
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && //检查UART是否设置了空闲标志，表示UART接收完成并进入空闲状态
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))//检查UART空闲中断是否被使能，只有在中断使能的情况下，才会处理空闲状态
	{
		uart_rx_idle_callback(huart);//调用之前定义的函数，处理接收到的数据
	}
}

rc_ctrl_t *get_rc_ctrl_data(void)
{
	return &RC_CtrlData;
}
