#include "Online_Monitor.h"
#include "bsp_dwt.h"

online_monitor_t online_monitor_data;

void online_state_set(uint16_t type, uint8_t state);

uint8_t Online_Monitors(fp32 last_online, uint16_t type)
{
	if (DWT_GetTimeline_s() - last_online > 0.5f)
    {
        online_state_set(type, 0);
        return 0;
    }
    online_state_set(type, 1);
	return 1;
}

void online_state_set(uint16_t type, uint8_t state)
{
    switch (type)
    {
        case RC_ONLINE:
            online_monitor_data.rc = state;
            break;
        case UPC_ONLINE:
            online_monitor_data.upc = state;
            break;
        case BIG_GIMBAL_IMU_ONLINE:
            online_monitor_data.big_gimbal_imu = state;
            break;
        case SMALL_GIMBAL_IMU_ONLINE:
            online_monitor_data.small_gimbal_imu = state;
            break;
        case CHASSIS_MOTOR_0_ONLINE:
            online_monitor_data.chassis_motor[0] = state;
            break;
        case CHASSIS_MOTOR_1_ONLINE:
            online_monitor_data.chassis_motor[1] = state;
            break;
        case CHASSIS_MOTOR_2_ONLINE:
            online_monitor_data.chassis_motor[2] = state;
            break;
        case CHASSIS_MOTOR_3_ONLINE:
            online_monitor_data.chassis_motor[3] = state;
            break;
        case GIMBAL_MOTOR_ONLINE:
            online_monitor_data.gimbal_motor = state;
            break;
        case REFEREE_ONLINE:
            online_monitor_data.referee = state;
            break;
        default:
            break;
    }
}

uint16_t error_code_get(void)
{
    uint16_t error_code = 0;
    error_code |= ((online_monitor_data.rc & 0x01) << 0);
    error_code |= ((online_monitor_data.upc & 0x01) << 1);
    error_code |= ((online_monitor_data.referee & 0x01) << 2);
    error_code |= ((online_monitor_data.big_gimbal_imu & 0x01) << 3);
    error_code |= ((online_monitor_data.small_gimbal_imu & 0x01) << 4);
    error_code |= ((online_monitor_data.chassis_motor[0] & 0x01) << 5);
    error_code |= ((online_monitor_data.chassis_motor[1] & 0x01) << 6);
    error_code |= ((online_monitor_data.chassis_motor[2] & 0x01) << 7);
    error_code |= ((online_monitor_data.chassis_motor[3] & 0x01) << 8);
    error_code |= ((online_monitor_data.gimbal_motor & 0x01) << 9);
	return error_code;
}
