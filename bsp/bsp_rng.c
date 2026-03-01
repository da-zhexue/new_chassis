#include "bsp_rng.h"
#include "rng.h"
#include "user_lib.h"

static first_order_filter_type_t random_filter;
static float filter_top_num[1] = {RANDOM_FLITER_TOP_BETA};
static uint32_t randbuf;

void rng_init(const float beta)
{
    HAL_RNG_Init(&hrng);
    //HAL_RNG_GenerateRandomNumber_IT(&hrng);
    if (beta > 0.0f && beta < 1.0f)
        filter_top_num[0] = beta;
    first_order_filter_init(&random_filter, RANDROM_CONTROL_TIME, filter_top_num);
}

uint32_t rng_smooth_rand()
{
    randbuf = HAL_RNG_GetRandomNumber(&hrng);
    first_order_filter_cali(&random_filter, (fp32)randbuf);
    return (uint32_t)random_filter.out;
}