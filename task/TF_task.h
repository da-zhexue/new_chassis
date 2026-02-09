#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "typedef.h"
#include "BMI088driver.h"

#define X 0
#define Y 1
#define Z 2
#define MF9025_ECD_IN_ZERO 0x1280

//#define AUTO_CORRECTION_ENABLE  // 9025编码器有问题，不能保证校准零位准确，暂时选择手动校准零位

void TF_Task(void const * argurment);
#endif
