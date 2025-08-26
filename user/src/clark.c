
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
