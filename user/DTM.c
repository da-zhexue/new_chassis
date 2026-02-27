/**
 * @file DTM.c
 * @brief 数据中转模块(Data Transfer Module)
 * 该文件用于保存需要用于多个任务的数据，并实现统一的写入与读取。
 * @version 2.0
 * @changelog
 * - 2026-02-09 统一写入读取
 */

#include "DTM.h"
#include <string.h>
#ifdef DEBUG_MODE
rc_t rc;
TF_t tf;
m9025_t m9025;
m3508_t m3508[4];
fp32 gimbal_s_deg[3];
fp32 gimbal_l_deg[3];
upc_t upc;
fp32 pid_debug[3];
uint8_t debug_ready_flag;
#else
static rc_t rc;
static TF_t tf;
static m9025_t m9025;
static m3508_t m3508[4];
static float gimbal_s_deg[3];
static float gimbal_l_deg[3];
static upc_t upc;
#endif

typedef struct {
    void *ptr;
    size_t size;
} DTM_Info_t;

static const DTM_Info_t DTM_Map[] = {
    [RC_DATA] = {&rc, sizeof(rc)},
    [TF_DATA] = {&tf, sizeof(tf)},
    [M9025_DATA] = {&m9025, sizeof(m9025)},
    [M3508_DATA] = {m3508, sizeof(m3508)},
    [GIMBAL_S_DATA] = {gimbal_s_deg, sizeof(gimbal_s_deg)},
    [GIMBAL_L_DATA] = {gimbal_l_deg, sizeof(gimbal_l_deg)},
    [UPC_DATA] = {&upc, sizeof(upc)},
#ifdef DEBUG_MODE
    [PARAM_DATA] = {pid_debug, sizeof(pid_debug)},
    [DEBUG_READY] = {&debug_ready_flag, sizeof(debug_ready_flag)}
#endif
}; // 在此处存储要用于不同任务的数据

static int DTM_GetInfo(const DTM_DataID_t data_id, void **ptr, size_t *size) {
    const volatile DTM_DataID_t dtm_data_id = data_id;
    if (dtm_data_id < DTM_DATA_COUNT) {
        *ptr = DTM_Map[dtm_data_id].ptr;
        *size = DTM_Map[dtm_data_id].size;
        return 1;
    }
    return 0;
}

DTM_Error_t DTM_Write(const DTM_DataID_t data_id, const void *data, const size_t size) {
    void *var_ptr;
    size_t var_size;
    
    if (!DTM_GetInfo(data_id, &var_ptr, &var_size))
        return DTM_ERR_UNKNOWN_ID;
    if (size != var_size)
        return DTM_ERR_SIZE_MISMATCH;
    
    memcpy(var_ptr, data, size);
    return DTM_SUCCESS;
}

DTM_Error_t DTM_Read(const DTM_DataID_t data_id, void *data, const size_t size) {
    void *var_ptr;
    size_t var_size;
    
    if (!DTM_GetInfo(data_id, &var_ptr, &var_size))
        return DTM_ERR_UNKNOWN_ID;
    if (size != var_size) 
        return DTM_ERR_SIZE_MISMATCH;

    memcpy(data, var_ptr, size);
    return DTM_SUCCESS;
}
