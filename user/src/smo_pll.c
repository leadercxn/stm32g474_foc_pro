#include "string.h"

#include "boards.h"
#include "util.h"
#include "math.h"
#include "trace.h"
#include "foc.h"
#include "smo_pll.h"
#include "parameters.h"
#include "clark.h"
#include "park.h"

#include "sys.h"

#include "trace.h"

//低通滤波器
float foc_low_pass_filter(lp_filter_t *lpf, float input)
{
    float output = lpf->pre_output_ratio * lpf->pre_output + (1.0f - lpf->pre_output_ratio) * input;

    lpf->pre_output = output;

    return output;
}

/**
 * 根据实际电流值计算得到 i_d 和 i_q
 */
_RAM_FUNC void iq_id_cal(current_foc_t *current_foc, float i_u, float i_v, float i_w, float pos_radian)
{
    // Clark 变换
    clark_cal_param(i_u, i_v, i_w, &current_foc->i_alpha, &current_foc->i_beta);

    // Park 变换
    park_cal_param(current_foc->i_alpha, current_foc->i_beta, pos_radian, &current_foc->i_d, &current_foc->i_q);

#if 0
    trace_debug("i_u %.2f, i_v %.2f, i_w %.2f| i_a %.2f, i_b %.2f| i_d %.2f, i_q %.2f, uq %.2f, rad %.4f\r\n", \
    i_u, i_v, i_w, current_foc->i_alpha, current_foc->i_beta, current_foc->i_d, current_foc->i_q, current_foc->uq, pos_radian);
#endif

}

/**
 * @brief PLL锁相环 计算, 解析反电动势中的角度
 * 
 * 参考灯哥FOC做的PLL锁相环
 */
_RAM_FUNC void pll_cal(smo_pll_t *p_smo_pll)
{
    if(p_smo_pll == NULL)
    {
        return;
    }

    float cos_theta = cosf(p_smo_pll->pll_est_theta);   //计算估算角度θ 的正弦，余弦值
    float sin_theta = sinf(p_smo_pll->pll_est_theta);

    /**
     * 反电动势 经 Park 变换后，得到的 d轴 q轴分量
     * 只列举 d轴上的电动势
     * Ed = Eα * cosθ + Eβ * sinθ
     * 
     * 因为Id = 0, 所以 Ed = 0
     * => Eα * cosθ + Eβ * sinθ = 0
     * 
     * 所以 实际值 Ed_real - 估算值 Ed_est = 误差 = 0 - (Eα_est * cosθ + Eβ_est * sinθ)
     * 
     */

    //求d轴上 反电动势的 误差 error = 实际值 Ed_real - 估算值 Ed_est, 也是电机转子加速度
    p_smo_pll->pll_theta_err = (-1 * p_smo_pll->est_vemf_alpha  * cos_theta) + (p_smo_pll->est_vemf_beta * sin_theta * -1);

    //求转子速度 i_err = ∑ Ts * Ki * error
    p_smo_pll->i_err += p_smo_pll->ts_dt * p_smo_pll->pll_ki * p_smo_pll->pll_theta_err;
    //速度 θ = Kp * error + i_err
    p_smo_pll->est_speed = p_smo_pll->pll_kp * p_smo_pll->pll_theta_err + p_smo_pll->i_err;

    //速度滤波 θ_filt
    p_smo_pll->est_speed_flt = p_smo_pll->est_speed_flt * 0.9f + p_smo_pll->est_speed * 0.1f;

    // 对电角速度进行积分获得电角度 θ = ∑ Ts * θ_filt
    p_smo_pll->pll_est_theta += p_smo_pll->ts_dt * p_smo_pll->est_speed_flt;

    //对电角度进行归一化
    p_smo_pll->pll_est_theta = radian_normalize(p_smo_pll->pll_est_theta);
}

/**
  * 滑模符号函数(饱和函数)
*/
_RAM_FUNC float smo_sat(float error, float limit)
{
    if(error > limit)
    {
        return 1;
    }
    else if(error < -limit)
    {
        return -1;
    }
    
    return error / limit;
}

static float  smo_limit_sat( float uint,float u_max, float u_min) //限制赋值函数
{
	float uout;
 	if(uint <= u_min)
		uout = u_min;
	else if( uint >= u_max)
		uout = u_max;
	else
	  uout = uint;
  return  uout;
}

/**
 * 参考灯哥FOC做的滑膜观测器
 * 
 * // 滑膜观察， 计算反电动势
 */
_RAM_FUNC void smo_position_estimate(smo_pll_t *p_smo_pll)
{
    if(p_smo_pll == NULL)
    {
        return;
    }

    //观察电流,  计算估算的电流
    p_smo_pll->est_i_alpha += p_smo_pll->ts_dt * \
    ( (-500.0f * p_smo_pll->est_i_alpha) + 1250.0f * (p_smo_pll->u_alpha - p_smo_pll->est_vemf_alpha) );
//    ( (-p_smo_pll->rs / p_smo_pll->ls * p_smo_pll->est_i_alpha) + (1 / p_smo_pll->ls) * (p_smo_pll->u_alpha - p_smo_pll->est_vemf_alpha));

    p_smo_pll->est_i_beta += p_smo_pll->ts_dt * \
    ( (-500.0f * p_smo_pll->est_i_beta) + 1250.0f * (p_smo_pll->u_beta - p_smo_pll->est_vemf_beta) );
//    ( (-p_smo_pll->rs / p_smo_pll->ls * p_smo_pll->est_i_beta) + (1 / p_smo_pll->ls) * (p_smo_pll->u_beta - p_smo_pll->est_vemf_beta));

#if 0
    // 顺序 ts_dt, uα, est_vemf_alpha, est_i_alpha, uβ, est_vemf_beta, est_i_beta
    VOFA_PRINTF("%.3f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f \n", p_smo_pll->ts_dt, p_smo_pll->u_alpha, p_smo_pll->est_vemf_alpha, p_smo_pll->est_i_alpha, \
    p_smo_pll->u_beta, p_smo_pll->est_vemf_beta, p_smo_pll->est_i_beta);
#endif

    //电流误差(构造滑膜面) 估算值 - 实际值 = 误差
    p_smo_pll->i_alpha_err = p_smo_pll->est_i_alpha - p_smo_pll->i_alpha;
    p_smo_pll->i_beta_err  = p_smo_pll->est_i_beta  - p_smo_pll->i_beta;

    //设计滑膜面趋近律 -- 等速趋近律， 推算出估算的反电动势
    p_smo_pll->est_vemf_alpha = p_smo_pll->h * smo_sat(p_smo_pll->i_alpha_err, 0.5f);   //0.5A 电流误差限幅
    p_smo_pll->est_vemf_beta  = p_smo_pll->h * smo_sat(p_smo_pll->i_beta_err, 0.5f);    //0.5A 电流误差限幅

    //低通滤波
    p_smo_pll->est_vemf_alpha_flt = 0.1 * p_smo_pll->est_vemf_alpha_flt + 0.9f * p_smo_pll->est_vemf_alpha;
    p_smo_pll->est_vemf_beta_flt  = 0.1 * p_smo_pll->est_vemf_beta_flt  + 0.9f * p_smo_pll->est_vemf_beta;
}

/**
 * 滑膜观测器 -- 参考硕历达模型
 */
_RAM_FUNC void smo_angle_cal(smo_pll_t *p_smo_pll)
{
    if(p_smo_pll == NULL)
    {
        return;
    }

    float I_alpha_err_limit = 0, I_beta_err_limit = 0;

    p_smo_pll->est_i_alpha += g_app_param.smo_f * p_smo_pll->est_i_alpha + \
                             g_app_param.smo_g * (p_smo_pll->u_alpha - p_smo_pll->est_vemf_alpha - p_smo_pll->z_alpha);

    p_smo_pll->est_i_beta += g_app_param.smo_f * p_smo_pll->est_i_beta + \
                            g_app_param.smo_g * (p_smo_pll->u_beta - p_smo_pll->est_vemf_beta - p_smo_pll->z_beta);

    //电流误差(构造滑膜面) 估算值 - 实际值 = 误差
    p_smo_pll->i_alpha_err = p_smo_pll->est_i_alpha - p_smo_pll->i_alpha;
    p_smo_pll->i_beta_err  = p_smo_pll->est_i_beta  - p_smo_pll->i_beta;

    // 限制赋值
    I_alpha_err_limit = smo_limit_sat(p_smo_pll->i_alpha_err, g_app_param.smo_i_err_max, (-g_app_param.smo_i_err_max));
    I_beta_err_limit  = smo_limit_sat(p_smo_pll->i_beta_err, g_app_param.smo_i_err_max, (-g_app_param.smo_i_err_max));

    p_smo_pll->z_alpha = I_alpha_err_limit * g_app_param.smo_k_slide;
	p_smo_pll->z_beta  = I_beta_err_limit  * g_app_param.smo_k_slide;

    /**/
    p_smo_pll->est_vemf_alpha = p_smo_pll->est_vemf_alpha + (p_smo_pll->z_alpha - p_smo_pll->est_vemf_alpha) * g_app_param.smo_k_slf;
    p_smo_pll->est_vemf_beta  = p_smo_pll->est_vemf_beta  + (p_smo_pll->z_beta  - p_smo_pll->est_vemf_beta)  * g_app_param.smo_k_slf;
}

/**
 * 反正切 -- 参考硕历达
 */
_RAM_FUNC void ftan_angle_cal(smo_pll_t *p_smo_pll)
{
    if(p_smo_pll == NULL)
    {
        return;
    }

    static float pll_est_theta_last = 0;

    if(p_smo_pll->est_vemf_beta == 0)
    {
        p_smo_pll->pll_est_theta = 0.0f;
    }
    else
    {
        p_smo_pll->pll_est_theta += -atanf(p_smo_pll->est_vemf_alpha / p_smo_pll->est_vemf_beta);
    }
    
//    p_smo_pll->pll_est_theta = pll_est_theta_last * 0.5f + p_smo_pll->pll_est_theta * 0.5f;

    //对电角度进行归一化
    p_smo_pll->pll_est_theta = radian_normalize(p_smo_pll->pll_est_theta);

    pll_est_theta_last = p_smo_pll->pll_est_theta;
}

//滑膜,锁相闭环
_RAM_FUNC void smo_pll_close_loop(smo_pll_t *p_smo_pll, current_foc_t *p_current_foc)
{
    if(p_smo_pll == NULL || p_current_foc == NULL)
    {
        return;
    }

    float cos_theta = cosf(p_smo_pll->pll_est_theta);   //计算估算角度θ 的正弦，余弦值
    float sin_theta = sinf(p_smo_pll->pll_est_theta);

    p_smo_pll->i_alpha = p_current_foc->i_alpha;   //赋值实际Iα
    p_smo_pll->i_beta  = p_current_foc->i_beta;    //赋值实际Iβ

//    p_smo_pll->u_alpha = cos_theta * p_current_foc->ud - sin_theta * p_current_foc->uq;
//    p_smo_pll->u_beta  = sin_theta * p_current_foc->ud + cos_theta * p_current_foc->uq;

    // 逆 park 变换
    p_smo_pll->u_alpha = -sin_theta * p_current_foc->uq;    //赋值估算的Uα, 因为角度是估算的
    p_smo_pll->u_beta  = cos_theta * p_current_foc->uq;     //赋值估算的Uβ

    // 灯哥 smo 滑膜模型
    smo_position_estimate(p_smo_pll);

    //硕历达 smo 滑膜模型
//    smo_angle_cal(p_smo_pll);

    // 反正切
//    ftan_angle_cal(p_smo_pll);

    //PLL锁相环计算
    pll_cal(p_smo_pll);

#if 0
    // 顺序 Uα, pll_theta_err, i_err, est_speed, pll_est_theta, iα, est_iα, i_alpha_err, iβ, est_iβ, i_beta_err, 
    VOFA_PRINTF("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f \n", p_smo_pll->u_alpha, p_smo_pll->pll_theta_err, p_smo_pll->i_err, \
    p_smo_pll->est_speed, p_smo_pll->pll_est_theta, p_smo_pll->i_alpha, p_smo_pll->est_i_alpha, p_smo_pll->i_alpha_err, \
    p_smo_pll->i_beta, p_smo_pll->est_i_beta, p_smo_pll->i_beta_err);
#endif

#if 0
    // 顺序 Uα, pll_theta_err, i_err, est_speed, pll_est_theta, iα, est_iα, i_alpha_err, iβ, est_iβ, i_beta_err, 
    VOFA_PRINTF("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", p_smo_pll->i_alpha, p_smo_pll->est_i_alpha, p_smo_pll->i_alpha_err, \
    p_smo_pll->i_beta, p_smo_pll->est_i_beta, p_smo_pll->i_beta_err);
#endif

}


















