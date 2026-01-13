#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H
#include "typedef.h"
#include "DBUS.h"
#include "COMM_rec.h"

#define GIMBAL_ANGLE_DELTA_MAX 0.1f
#define CHASSIS_MAX_V 7000.0f
#define CHASSIS_MAX_W 10.0f

typedef enum
{
    CHASSIS_RC_OFFLINE = 0,
    CHASSIS_RC = 1,
    CHASSIS_UPC = 2,
    CHASSIS_STOP = 3
}CHASSIS_CTRL_STATE;

typedef struct
{
    fp32 given_chassis_v[2];
    fp32 given_chassis_w;
    fp32 given_gimbal_yaw;
    fp32 given_chassis_yaw;

    uint8_t ctrl;
    uint8_t mode;

    uint8_t gimbal_shutdown_flag;
    uint8_t last_gimbal_shutdown_flag;

}chassis_ctrl_t;

void ctrl_data_update(rc_ctrl_t *rc_ctrl_ptr, upc_t *upc_ptr);
void chassis_ctrl_init(void);
chassis_ctrl_t* get_chassis_ctrl_data(void);

#endif
