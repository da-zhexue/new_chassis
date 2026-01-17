#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "typedef.h"
#include "BMI088driver.h"

#define X 0
#define Y 1
#define Z 2
#define MF9025_ECD_IN_ZERO 0x128D

void TF_Task(void const * argurment);
#endif
