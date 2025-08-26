
#include "math.h"
#include "park.h"

/**
 * @brief Park 变换
 */
void park_cal(foc_param_t *foc_param)
{
    // Park 变换
    float cos_theta = cosf(foc_param->pos_radian);
    float sin_theta = sinf(foc_param->pos_radian);

    foc_param->d = foc_param->alpha * cos_theta + foc_param->beta * sin_theta;
    foc_param->q = -foc_param->alpha * sin_theta + foc_param->beta * cos_theta;
}

/**
 * @brief 逆 Park 变换
 */
void inv_park_cal(foc_param_t *foc_param)
{
    // iPark 变换
    float cos_theta = cosf(foc_param->pos_radian);
    float sin_theta = sinf(foc_param->pos_radian);

    foc_param->alpha = foc_param->d * cos_theta - foc_param->q * sin_theta;
    foc_param->beta  = foc_param->d * sin_theta + foc_param->q * cos_theta;
}
