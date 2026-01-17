#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H
#include "typedef.h"
#include "data_transfer.h"

#define GIMBAL_ANGLE_DELTA_MAX 0.05f
#define CHASSIS_MAX_V 7000.0f
#define CHASSIS_MAX_W 10.0f

#define CHASSIS_FILTER_VX_BETA 0.3f
#define CHASSIS_FILTER_VY_BETA 0.3f
#define CHASSIS_FILTER_VW_BETA 0.3f
#define CHASSIS_CONTROL_TIME 0.002f

typedef enum
{
    CHASSIS_RC_OFFLINE = 0,
    CHASSIS_RC = 1,
    CHASSIS_UPC = 2,
    GIMBAL_RC = 3
}CHASSIS_CTRL_STATE;

void ctrl_data_update(rc_ctrl_t *rc_ctrl_ptr, upc_t *upc_ptr);
void chassis_ctrl_init(void);

#endif
