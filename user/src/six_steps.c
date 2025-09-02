/**
 * @file        six_steps.c
 * @brief       无感六步换相驱动，改自正点原子的六步驱动程序
 */

#include "six_steps.h"

#include "boards.h"
#include "parameters.h"
#include "timer.h"
#include "gpio.h"

vvvf_start  m_vvvf_sp;                          /* 无感开环结构体 */
uint16_t    m_delay_temp = 0;                   /* 延迟变量 */
uint8_t     m_zero_ctr_status = 0;              /* 无感状态标志位 */
/**
 * @brief       无感控制逻辑
 * @note        无感开环六步换相步骤：先将转子固定至某一相-->从无感6步换相顺序依次导通，即可实现
 * @param       无
 * @retval      无
 */
void zero_ctr_loop(void)
{
    if(g_bldc_motor.run_flag == 1)
    {
        switch(m_zero_ctr_status)
        {
            case 0:                             /* 定位：固定一相*/
            {
                m_vvvf_sp.voilage_ref = (uint16_t)(MAX_PWM_DUTY / 20);
                g_bldc_motor.pwm_duty = m_vvvf_sp.voilage_ref;
                motor_uhvl();                      /* U+V-*/
                m_delay_temp = 0;
                m_vvvf_sp.freq_t_ref = 1100;    /* 初始延迟时间为：55*1100us，不同电机此参数不同 */
                m_zero_ctr_status = 1;          /* 切换至延迟计数 */
                m_vvvf_sp.count = 0;            /* 换相电压的换相计数 */
            }
            break;

            case 1:                             /* 定时延迟 */
            {
                m_delay_temp++;
                if(m_delay_temp >= m_vvvf_sp.freq_t_ref)
                {
                    m_delay_temp = 0;
                    m_zero_ctr_status = 2;      /* 切换至换相操作 */
                }
            }
            break;

            case 2:                             /* 加速换相（提高换向频率）*/
            {
                m_vvvf_sp.freq_t_ref -= m_vvvf_sp.freq_t_ref / 12 + 1;  /* 改变换向频率 */
                m_vvvf_sp.count++;
                change_voltage();               /* 改变换向所需电压 */

                if(m_vvvf_sp.freq_t_ref < 180)
                {
                    m_vvvf_sp.freq_t_ref = 180; /* 固定次频率（延迟时间），不同电机此参数有差异4对级的可设置150 */
                    m_zero_ctr_status = 1;      /* 切换至延迟计数 */
                }
                else
                {
                    m_zero_ctr_status = 1;      /* 切换至延迟计数 */
                }
                
                m_vvvf_sp.vvvf_count++;         /* 六步换相的换向计数 */

                if(m_vvvf_sp.vvvf_count == 6)
                {
                    m_vvvf_sp.vvvf_count = 0;
                }
                anwerfen_sw();

            }
            break;
            default:    break;
        }
    }
    else if(g_bldc_motor.run_flag == 0)
    {
        zero_ctr_init();                        /* 停止状态下执行无感初始化 */
    }
}

/**
 * @brief       无感控制初始化，将无感状态标志清0
 * @param       无
 * @retval      无
 */
void zero_ctr_init(void)
{
    m_zero_ctr_status = 0;                      /* 无感状态标志位清0 */
    m_delay_temp = 0;                           /* 延迟时间清0 */
    m_vvvf_sp.count = 0;                        /* 换相电压的换相计数清0 */
    m_vvvf_sp.vvvf_count = 0;                   /* 6步换相的换相计数清0 */
}

/**
 * @brief       无感开环6步换相顺序
 * @param       无
 * @retval      无
 */
void anwerfen_sw(void)
{
    if(g_bldc_motor.dir == MOTOR_DIR_CCW)                /* 逆时针旋转*/
    {
        switch(m_vvvf_sp.vvvf_count)            /* 换向次数 */
        {
            /*六步换向顺序:(U+V-)-> (U+W-)-> (V+W-)-> (V+U-)-> (W+U-)->(W+V-)*/
            case  0x00:  motor_uhvl(); break;      /* U+V-* 对应初始固定相 */
            case  0x01:  motor_uhwl(); break;
            case  0x02:  motor_vhwl(); break;
            case  0x03:  motor_vhul(); break;
            case  0x04:  motor_whul(); break;
            case  0x05:  motor_whvl(); break;
            default:break;
        }
    }
    else
    {
        /*正转顺序*/
        switch(m_vvvf_sp.vvvf_count)
        {
            case  0x00:  motor_uhvl(); break;      /* U+V-* 对应初始固定相 */
            case  0x01:  motor_whvl(); break;
            case  0x02:  motor_whul(); break;
            case  0x03:  motor_vhul(); break;
            case  0x04:  motor_vhwl(); break;
            case  0x05:  motor_uhwl(); break;
            default:break;
        }
    }
}
#define FSCA    2
/**
 * @brief       换相电压修改
 * @param       无
 * @retval      无
 */
void change_voltage(void)
{
    switch(m_vvvf_sp.count)
    {
        case  1:  g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 10; break;
        case  2:  g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 9;  break;
        case  3:  g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 8;  break;
        case  4:  g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 7;  break;
        case  5:  g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 7;  break;
        case  6:  g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 6;  break;
        case  7:  g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 6;  break;
        case  8:  g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 5;  break;
        case  9:  g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 5;  break;
        case  10: g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 4;  break;
        case  11: g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 4;  break;
        case  12: g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 4;  break;
        case  13: g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 4;  break;
        case  14: g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 4;  break;
        case  15: g_bldc_motor.pwm_duty = MAX_PWM_DUTY / FSCA / 4;  break;
        default:    break;   
    }
}


