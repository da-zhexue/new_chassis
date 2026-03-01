#ifndef NEW_CHASSIS_BSP_RNG_H
#define NEW_CHASSIS_BSP_RNG_H
#include "typedef.h"

#define RANDOM_FLITER_TOP_BETA 0.9f
#define RANDROM_CONTROL_TIME 0.02f

void rng_init(float beta);
uint32_t rng_smooth_rand();

#endif //NEW_CHASSIS_BSP_RNG_H