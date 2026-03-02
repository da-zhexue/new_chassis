/**
 * @file comm_task.c
 * @brief 与通信板通信任务模块
 * 向通信板发送数据，详见通信协议。
 * @version 1.1
 * @changelog
 * - 2026-01-17 适配新的数据中转模块
 */

#include "comm_task.h"


#include "cmsis_os.h"
#include "comm.h"
#include "DTM.h"

void commTask(void const * argument)
{
	static TF_t tf_ptr;
	static upc_t upc_ptr;
    while(1)
    {
    	DTM_Read(TF_DATA, &tf_ptr, sizeof(tf_ptr));
    	DTM_Read(UPC_DATA, &upc_ptr, sizeof(upc_ptr));
		send_attitude_handler(&tf_ptr);
    	send_onlinestate_handler();
    	if (upc_ptr.game_start)
    		send_start_handler();

#ifdef DEBUG_MODE
    	send_power_ctrl_param_handler();
#endif
        osDelay(2);
			
    }
}
