#include "string.h"
#include "boards.h"
#include "foc.h"
#include "adc.h"
#include "timer.h"
#include "util.h"
#include "math.h"

#include "parameters.h"
#include "sensors_task.h"

#include "trace.h"

/**
 * @brief 弧度归一化 [0 , 2PI]
 */
float radian_normalize(float radian)
{
    float a;

	a = fmod(radian, DOUBLE_PI);
	
	return (a >= 0) ? a : (a + DOUBLE_PI);
}


/**
 * @brief 设置力矩 , 传入需要进行 iPark 变换的参数
 * 
 * @param uq q轴电压，单位：V
 * @param ud d轴电压，单位：V， 常规为 0
 * @param pos_radian 位置角度，单位：弧度
 */
void torque_set(float uq, float ud, float pos_radian)
{
    foc_param_t foc_u;

    foc_u.q = uq;  // q轴电压
    foc_u.d = ud;  // d轴电压

    // 记录下当前的uq值到 g_current_foc
    g_current_foc.uq = uq;
    g_current_foc.ud = ud;

# if 0
    foc_u.pos_radian = radian_normalize(pos_radian);  // 归一化位置角度

    inv_park_cal(&foc_u);    // 逆 Park 变换

//假如用第二种方法 N = 4C + 2B + A 的公式来求扇区的话，就要用 iclark 变换
#if 0
    inv_clark_cal(&foc_u);   // 逆 Clark 变换
#endif

    //第一种方式，根据弧度制判断 分区
    int sector = (int)(foc_u.pos_radian / PI_DIV_3) + 1;  // 计算扇区

    float t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;
    float ts = PWM_PERIOD;                                      // 因为中心对齐，20KHz PWM 的周期是 50us
    float k = ts / VBUS_VLOT;  // 计算比例系数 k = TS / Udc

    uint32_t tu = 0, tv = 0, tw = 0;  // 三相 PWM 占空比

    float t_div_mom = 0.5;

    // 根据分区设置时间
    if(sector == 1)
    {
        t4 = SQRT_3 * k * (SQRT_3_DIV_2 * foc_u.alpha - 0.5f * foc_u.beta);
        t6 = SQRT_3 * k * foc_u.beta;

        tu = (uint32_t) ((ts - t4 - t6) * t_div_mom);  // u相占空比
        tv = (uint32_t) ((ts + t4 - t6) * t_div_mom);  // v相占空比
        tw = (uint32_t) ((ts + t4 + t6) * t_div_mom);  // w相占空比
    }
    else if(sector == 2)
    {
        t2 = SQRT_3 * k * (-SQRT_3_DIV_2 * foc_u.alpha + 0.5f * foc_u.beta);
        t6 = SQRT_3 * k * (SQRT_3_DIV_2 * foc_u.alpha + 0.5f * foc_u.beta);

        tu = (uint32_t) ((ts + t2 - t6) * t_div_mom);  // u相占空比
        tv = (uint32_t) ((ts - t2 - t6) * t_div_mom);  // v相占空比
        tw = (uint32_t) ((ts + t2 + t6) * t_div_mom);  // w相占空比
    }
    else if(sector == 3)
    {
        t2 = SQRT_3 * k * foc_u.beta;
        t3 = -SQRT_3 * k * (SQRT_3_DIV_2 * foc_u.alpha + 0.5f * foc_u.beta);

        tu = (uint32_t) ((ts + t2 + t3) * t_div_mom);  // u相占空比
        tv = (uint32_t) ((ts - t2 - t3) * t_div_mom);  // v相占空比
        tw = (uint32_t) ((ts + t2 - t3) * t_div_mom);  // w相占空比
    }
    else if(sector == 4)
    {
        t1 = -SQRT_3 * k * foc_u.beta;
        t3 = SQRT_3 * k * (-SQRT_3_DIV_2 * foc_u.alpha + 0.5f * foc_u.beta);

        tu = (uint32_t) ((ts + t1 + t3) * t_div_mom);  // u相占空比
        tv = (uint32_t) ((ts + t1 - t3) * t_div_mom);  // v相占空比
        tw = (uint32_t) ((ts - t1 - t3) * t_div_mom);  // w相占空比
    }
    else if(sector == 5)
    {
        t1 = -SQRT_3 * k * (SQRT_3_DIV_2 * foc_u.alpha + 0.5f * foc_u.beta);
        t5 = -SQRT_3 * k * (-SQRT_3_DIV_2 * foc_u.alpha + 0.5f * foc_u.beta);

        tu = (uint32_t) ((ts + t1 - t5) * t_div_mom);  // u相占空比
        tv = (uint32_t) ((ts + t1 + t5) * t_div_mom);  // v相占空比
        tw = (uint32_t) ((ts - t1 - t5) * t_div_mom);  // w相占空比
    }
    else if(sector == 6)
    {
        t4 = SQRT_3 * k  * (SQRT_3_DIV_2 * foc_u.alpha + 0.5f * foc_u.beta);
        t5 = -SQRT_3 * k * foc_u.beta;

        tu = (uint32_t) ((ts - t4 - t5) * t_div_mom);  // u相占空比
        tv = (uint32_t) ((ts + t4 + t5) * t_div_mom);  // v相占空比
        tw = (uint32_t) ((ts + t4 - t5) * t_div_mom);  // w相占空比
    }

    trace_debug("sector %d, uq %f, radian %f, alpha %f, beta %f\r\n", sector, uq, foc_u.pos_radian, foc_u.alpha, foc_u.beta);
    trace_debug("t1 %f, t2 %f, t3 %f, t4 %f, t5 %f, t6 %f\r\n", t1, t2, t3, t4, t5, t6);
    trace_debug("tu %lu, tv %lu, tw %lu\r\n", tu, tv, tw);
#endif


/**
 * 参考灯哥FOC
 * 验证波形正确
 */
#if 1

    float u_out = 0;

    if(ud)
    {
        u_out = sqrtf(uq * uq + ud * ud) / VBUS_VLOT;  // 计算输出电压
        foc_u.pos_radian = radian_normalize(pos_radian + atan2f(uq, ud));               // 归一化位置角度
    }
    else
    {
        u_out = uq / VBUS_VLOT;    // 计算输出电压
        foc_u.pos_radian = radian_normalize(pos_radian);            // 归一化位置角度
    }

    int sector = (int)(foc_u.pos_radian / PI_DIV_3) + 1;            // 计算扇区
    float t0 = 0.0f, t1 = 0.0f, t2 = 0.0f;
    float ts = PWM_PERIOD;                                          // 因为中心对齐，20KHz PWM 的周期是 50us

    uint32_t tu = 0, tv = 0, tw = 0;                                // 三相 PWM 占空比

    t1 = SQRT_3 * u_out  * sin(sector * PI_DIV_3 - foc_u.pos_radian);
    t2 = SQRT_3 * u_out  * sin(foc_u.pos_radian - (sector - 1)  * PI_DIV_3);
    t0 = 1 - t1 - t2;

    // 根据分区设置时间
    if(sector == 1)
    {
        tu = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // u相占空比
        tv = (uint32_t) ((t2 + t0 * 0.5f) * ts);       // v相占空比
        tw = (uint32_t) ((t0 * 0.5) * ts);             // w相占空比
    }
    else if(sector == 2)
    {
        tu = (uint32_t) ((t1 + t0 * 0.5f) * ts);       // u相占空比
        tv = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // v相占空比
        tw = (uint32_t) ((t0 * 0.5) * ts);             // w相占空比
    }
    else if(sector == 3)
    {
        tu = (uint32_t) ((t0 * 0.5f) * ts);            // u相占空比
        tv = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // v相占空比
        tw = (uint32_t) ((t2 + t0 * 0.5f) * ts);       // w相占空比
    }
    else if(sector == 4)
    {
        tu = (uint32_t) ((t0 * 0.5f) * ts);            // u相占空比
        tv = (uint32_t) ((t1 + t0 * 0.5f) * ts);       // v相占空比
        tw = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // w相占空比
    }
    else if(sector == 5)
    {

        tu = (uint32_t) ((t2 + t0 * 0.5f) * ts);       // u相占空比
        tv = (uint32_t) ((t0 * 0.5f) * ts);            // v相占空比
        tw = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // w相占空比
    }
    else if(sector == 6)
    {

        tu = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // u相占空比
        tv = (uint32_t) ((t0 * 0.5f) * ts);            // v相占空比
        tw = (uint32_t) ((t1 + t0 * 0.5f) * ts);       // w相占空比
    }

#if 0
    trace_debug("sector %d, uq %f, radian %f\r\n", sector, uq, foc_u.pos_radian);
    trace_debug("t0 %f->%f, t1 %f->%f, t2 %f->%f\r\n", t0, t0 * 50.f, t1, t1 * 50.0f ,t2, t2 * 50.0f);
    trace_debug("tu %lu, tv %lu, tw %lu\r\n", tu, tv, tw);
#endif

#endif

    phase_pwm_set(tu, tv, tw);
}


/**
 * @brief 设置uvw三相的PWMs输出电压, 在 iClark 变换后使用
 * 
 * @param u 期望U相电压，单位：V
 * @param v 期望V相电压，单位：V
 * @param w 期望W相电压，单位：V
 */
void svpwm_set(float u, float v, float w)
{
    //限幅
    float u_value = CONSTRAIN( u, 0, VBUS_VLOT );
    float v_value = CONSTRAIN( v, 0, VBUS_VLOT );
    float w_value = CONSTRAIN( w, 0, VBUS_VLOT );

    //计算各通道占空比
    float u_pwm_per = u_value / VBUS_VLOT;
    float v_pwm_per = v_value / VBUS_VLOT;
    float w_pwm_per = w_value / VBUS_VLOT;

    // 将电压转换为PWM占空比
    uint32_t u_pwm = (uint32_t)(u_pwm_per * PWM_PERIOD);
    uint32_t v_pwm = (uint32_t)(v_pwm_per * PWM_PERIOD);
    uint32_t w_pwm = (uint32_t)(w_pwm_per * PWM_PERIOD);

    phase_pwm_set(u_pwm, v_pwm, w_pwm);
}