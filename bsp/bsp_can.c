#include "bsp_can.h"
#include "can.h"

void can_filter_init(void)
{
  CAN_FilterTypeDef canfilter;

  canfilter.FilterBank = 0;
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
  canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilter.FilterActivation = ENABLE;
  canfilter.SlaveStartFilterBank = 13;

  if (HAL_CAN_ConfigFilter(&hcan1, &canfilter) != HAL_OK)
  {
    Error_Handler();
  }

  // 启动CAN
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
	
	canfilter.SlaveStartFilterBank = 14;
  canfilter.FilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan2, &canfilter) != HAL_OK)
  {
    Error_Handler();
  }
	if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

  // 使能接收中断（可选）
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}
