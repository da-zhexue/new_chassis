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
rc_ctrl_t* rc_ctrl_ptr;
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

    rc_ctrl_ptr->ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    rc_ctrl_ptr->ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    rc_ctrl_ptr->ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                          ((int16_t)pData[4] << 10)) &
                         0x07FF;
    rc_ctrl_ptr->ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) &
                         0x07FF;
	rc_ctrl_ptr->ch0 -= RC_CH_VALUE_OFFSET;
	rc_ctrl_ptr->ch1 -= RC_CH_VALUE_OFFSET;
	rc_ctrl_ptr->ch2 -= RC_CH_VALUE_OFFSET;
	rc_ctrl_ptr->ch3 -= RC_CH_VALUE_OFFSET;
	rc_ctrl_ptr->s1 = ((pData[5] >> 4) & 0x0003);
    rc_ctrl_ptr->s2 = ((pData[5] >> 4) & 0x000C) >> 2;
     // | ((int16_t)pData[15] << 8);
    rc_ctrl_ptr->last_online_time = DWT_GetTimeline_s();
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
{//���ڽ������ݵĺ�����ʹ��DMA��ʽ������UART����
  uint32_t tmp1 = 0;
  tmp1 = huart->RxState;
	//����һ����ʱ����tmp1������UART�Ľ���״̬��ֵ����
	if (tmp1 == HAL_UART_STATE_READY)//�ж�UART�Ƿ��ھ���״̬
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}
 
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;
 
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		//����DMA���գ�Դ��ַΪUART���ݼĴ�����Ŀ���ַΪ���ջ�����
	
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		//ʹ��UART��DMA���չ���
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

void dbus_uart_init(void)//DBUS���ڳ�ʼ��
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);//�������UART�Ŀ��б�־�����б�־ָʾUART�ڽ�������ʱ���ڿ���״̬��ͨ���ڽ�����ɺ�����
	//��������־��Ϊ��ȷ�������Ľ��ղ����ܹ���ȷ��⵽�µĿ���״̬
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);//ʹ��UART�Ŀ����жϣ���UART���ڿ���״̬���ҽ��ջ�����û������ʱ���ᴥ������ж�
	//ʹ������жϺ󣬿������жϷ��������д�������״̬
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);//����֮ǰ�ĺ�������DMA�����մ�������

	rc_ctrl_ptr = get_rc_ctrl_data();
}

uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{//����DMAԤ����Ļ�����ʣ��ĳ��ȣ������˽⴫������л��ж���������δ����
  return ((uint16_t)(dma_stream->NDTR));
}
 
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	//���UART�Ŀ��б�־���Ա���һ�ν���ʱ�ܹ���ȷ��⵽����״̬
	
	if (huart == &DBUS_HUART)//ȷ��ֻ����DBUS����
	{
		__HAL_DMA_DISABLE(huart->hdmarx);//ʧ��DMA���գ���ֹ��һ�ν��յ���������һ�����ݵ�β����������ȫ�µ�����
 
		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{//���㵱ǰ���յ����ݳ��ȣ�������յ������ݳ��ȵ���18�ֽڣ�����ô������ݺ���
            RemoteDataProcess(dbus_buf);    	//�������յ����ݲ�����
		}
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);//����DMA����Ԥ����Ļ������ĳ��ȣ��Ա�Ϊ��һ�ν�������׼��
		__HAL_DMA_ENABLE(huart->hdmarx);//��������DMA���գ��Ա������������
	}
}

void uart_receive_handler(UART_HandleTypeDef *huart)
{//���ڼ��UART����״̬���ڽ��յ�����״̬ʱ������Ӧ�Ļص�����
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && //���UART�Ƿ������˿��б�־����ʾUART������ɲ��������״̬
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))//���UART�����ж��Ƿ�ʹ�ܣ�ֻ�����ж�ʹ�ܵ�����£��Żᴦ������״̬
	{
		uart_rx_idle_callback(huart);//����֮ǰ����ĺ������������յ�������
	}
}
