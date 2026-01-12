#include "chassis_task.h"
#include "cmsis_os.h"
#include "DBUS.h"
#include "COMM_rec.h"
#include "motor_ctrl.h"
#include "CAN_rx.h"

void ChassisTask(void const * argument)
{   
	dbus_uart_init();
	chassis_ctrl_init();
	motor_ctrl_init();
	CAN_Receive_Init();

	rc_ctrl_t *rc_ctrl_ptr = get_rc_ctrl_data();
	upc_t *upc_ptr = get_upc_data();
	chassis_ctrl_t *chassis_ctrl_ptr = get_chassis_ctrl_data();

	while(1)
	{
		ctrl_data_update(rc_ctrl_ptr, upc_ptr);
		motor_ctrl_update(chassis_ctrl_ptr);
		osDelay(1);
	}
}
