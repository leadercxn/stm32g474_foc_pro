#ifndef SMO_PLL_H__
#define SMO_PLL_H__

#include "foc.h"

typedef struct {
    float pre_output_ratio;     //上一次输出占比
    float pre_output;           //上一次输出值
}lp_filter_t;

typedef struct 
{
    float rs;               //电机相电阻
    float ls;               //电机相电感

    float u_alpha;          //Uα alpha轴电压
    float u_beta;           //Uβ beta轴电压

    float i_alpha;          //Iα alpha轴实际电流
    float i_beta;           //Iβ beta轴实际电流

    float est_i_alpha;      //估算的Iα
    float est_i_beta;       //估算的Iβ

    float i_alpha_err;      //Iα电流误差
    float i_beta_err;       //Iβ电流误差

    float h;                    //滑膜增益

    float est_vemf_alpha;       //估算的反电动势Uα
    float est_vemf_beta;        //估算的反电动势Uβ

    float est_vemf_alpha_flt;   //滤波后的估算的反电动势Uα
    float est_vemf_beta_flt;    //滤波后的估算的反电动势Uβ


    float pll_theta_err;        //传入PLL角度误差

    float pll_est_theta;        //PLL估算角度
    float pll_err_last;         //PLL角度误差上次值

    float est_speed;            //估算速度
    float est_speed_flt;        //滤波后的估算速度

    float pll_kp;               //PLL比例参数
    float pll_ki;               //PLL积分参数

    float i_err;                 //电流误差积分
    float last_i_err;            //上次电流误差积分

    uint32_t last_tick;          //上次观察的时间
    float    ts_dt;              //观察的时间差，单位 s
} smo_pll_t;

typedef struct
{
  float i_d;
  float i_q;
  float i_alpha;
  float i_beta;
  float ud;
  float uq;         //pid 运算后得到的 uq值
} current_foc_t;

float foc_low_pass_filter(lp_filter_t *lpf, float input);
void iq_id_cal(current_foc_t *current_foc, float i_u, float i_v, float i_w, float pos_radian);
void smo_pll_close_loop(smo_pll_t *p_smo_pll, current_foc_t *p_current_foc);


#endif
