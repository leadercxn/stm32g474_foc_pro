
#include "math.h"
#include "clark.h"

/**
 * @brief Clark 等幅变换
 */
void clark_cal(foc_param_t *foc_param)
{
    // Clark 变换
    foc_param->alpha = foc_param->u;
    foc_param->beta = (2 * foc_param->v + foc_param->u) * SQRT_3_DIV_3;          // 除以√3 = 乘以√3/3  乘法比除法块
}

/**
 * @brief 逆 Clark 等幅变换
 */
void inv_clark_cal(foc_param_t *foc_param)
{
    // iClark 变换
    foc_param->u = foc_param->alpha;
    foc_param->v = (-foc_param->alpha + SQRT_3 * foc_param->beta) * 0.5f;  //乘法比除法块
    foc_param->w = (-foc_param->alpha - SQRT_3 * foc_param->beta) * 0.5f;  //乘法比除法块
}

void clark_cal_param(float u, float v, float w, float *alpha, float *beta)
{
    // Clark 变换
    *alpha = u;
    *beta = (2 * v + u) * SQRT_3_DIV_3;          // 除以√3 = 乘以√3/3  乘法比除法块
}

void inv_clark_cal_param(float alpha, float beta, float *u, float *v, float *w)
{
    // iClark 变换
    *u = alpha;
    *v = (-alpha + SQRT_3 * beta) * 0.5f;  //乘法比除法块
    *w = (-alpha - SQRT_3 * beta) * 0.5f;  //乘法比除法块
}