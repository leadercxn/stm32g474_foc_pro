#include <math.h>
#include "string.h"

#include "boards.h"
#include "adc.h"
#include "sys.h"
#include "util.h"
#include "gpio.h"
#include "parameters.h"
#include "motor_ctrl_task.h"
#include "uart.h"
#include "timer.h"
#include "foc.h"
#include "sensors_task.h"

#include "trace.h"

#define UQ_INIT             0.5f    //无感启动初始q轴电压
#define UQ_ACC_STEP         0.036f  //无感启动q轴电压每次增加步进
#define FIRST_STEPS_TIMES   60      //无感启动初始固定相时间间隔

typedef struct
{
    uint8_t     cmd;            //命令字 0:停止 1:启动
    uint8_t     dir;            //方向   0:正转 1:反转
    uint16_t    speed;          //速度  0 ~ 4000
    uint8_t     uq;             //q轴电压 0 ~ 255       1 = 0.1 V
    uint16_t    iq;             //q轴电流 0 ~ 19800     1 = 1 mA
} __attribute__((__packed__ )) uart_cmd_t;

typedef enum
{
    MOTOR_START_STA_ACC,        //加速中
    MOTOR_START_STA_ACC_END,    //加速完成
    MOTOR_START_STA_CONST,      //恒速转动
} motor_start_sta_e;

typedef struct
{
    uint32_t last_ticks;        //上次时间戳
    float    pre_output;        //上次输出值
} lp_flt_param_t;

lp_flt_param_t m_iq_flt = {0};   //Iq滤波器

static float iq_lowpass_filter(lp_flt_param_t *p_lp_flt, float input, float tf)
{
    if(p_lp_flt == NULL)
    {
        return 0.0f;
    }

    uint32_t temp_dt = sys_time_ms_get() - p_lp_flt->last_ticks;
    if(temp_dt >= 10)                                       // 因为第一次运行，这个时间会比较大，不合理，需要限幅
    {
        p_lp_flt->last_ticks = sys_time_ms_get();
        p_lp_flt->pre_output = input;
        return input;
    }

    float alpha = tf / (tf + temp_dt * 0.001f);   //计算滤波系数

    float out = alpha * p_lp_flt->pre_output + (1.0f - alpha) * input;

    p_lp_flt->pre_output = out;
    p_lp_flt->last_ticks = sys_time_ms_get();

    return out;
}

/**
 * @brief 速度、电流环双环PID计算力矩Uq
 */
static void vel_current_loop_pid_handler(void)
{
    float vel_pid_out_iq     = 0.0f;    //速度环输出, q轴电流
    float current_pid_out_uq = 0.0f;    //电流环输出, q轴电压

    static float speed_flt = 0.0f;

    speed_flt = speed_flt * 0.998f + g_app_param.motor_speed_set * 0.002f;  //速度滤波

    // 速度环
    vel_pid_out_iq = pid_cal(&g_moter_vel_loop_pid, g_smo_pll.est_speed * 0.2, speed_flt);

    // 电流环
    current_pid_out_uq = pid_cal(&g_moter_i_loop_pid, g_current_foc.i_q, vel_pid_out_iq);

    //设置力矩
    torque_set(current_pid_out_uq, 0, g_smo_pll.pll_est_theta);
}


/**
 * @brief 电流环PID计算力矩Uq
 */
static void current_loop_pid_handler(void)
{
    float current_pid_out_uq = 0.0f;    //电流环输出, q轴电压

    static float target_iq_flt = 0.0f;
    float  feb_iq = 0.0f;

    target_iq_flt = target_iq_flt * 0.5f + g_app_param.iq * 0.5f;       //速度滤波

    feb_iq = iq_lowpass_filter(&m_iq_flt, g_current_foc.i_q, 0.002f);   //Iq低通滤波

    // 电流环
    current_pid_out_uq = pid_cal(&g_moter_i_loop_pid, feb_iq, target_iq_flt);

    VOFA_PRINTF("%.4f, %.4f, %.4f\n", feb_iq, target_iq_flt, current_pid_out_uq);   // pid input_iq, set_iq, pid output_uq

    current_pid_out_uq = CONSTRAIN(current_pid_out_uq, 0.1f, 8.0f);

    //设置力矩
    torque_set(current_pid_out_uq, 0, g_smo_pll.pll_est_theta);
}

/**
 * @brief 力矩环
 */
static void smo_torque_handler(void)
{
    static float target_uq_flt = 0.0f;

    target_uq_flt = target_uq_flt * 0.99f + g_app_param.uq * 0.01f;       //目标力矩滤波

//    VOFA_PRINTF("%.4f, %.4f, %.4f\n", target_uq_flt, g_app_param.uq, g_smo_pll.pll_est_theta);

    torque_set(target_uq_flt, 0, g_smo_pll.pll_est_theta);
}

/**
 * @brief 运行无感滑膜观测、PLL锁相、FOC等处理
 */
static void motor_smo_pll_foc_run(void)
{
    // 根据三相电流, 计算出clark变换后，实际的i_alpha, i_beta, 以及park变换后估算的 i_d, i_q（因为角度是估算的），存放到 g_current_foc
    iq_id_cal(&g_current_foc, adc_sample_physical_value_get(ADC_CH_U_I), adc_sample_physical_value_get(ADC_CH_V_I), \
                              adc_sample_physical_value_get(ADC_CH_W_I), g_smo_pll.pll_est_theta);

    smo_pll_close_loop(&g_smo_pll, &g_current_foc);                          
}



/**
 * 电机控制逻辑任务
 */
int motor_ctrl_task(void)
{

    static uint8_t uart1_rx_uq = 0;

    static uint32_t             motor_starting_ticks = 0;               //电机启动时间戳
    static float                shaft_angle = PI_DIV_4;                     //电机轴角度
    static motor_start_sta_e    motor_acc_stats = MOTOR_START_STA_ACC;  //加速状态 0：加速中 1：加速完成 2：恒速转动
    static uint16_t             motor_acc_cnt = 0;                      //电机加速次数计数
    static uint16_t             motor_acc_ticks_dt = 0;                 //累计每一次电机调整角度时间间隔
    static uint16_t             per_acc_hold_ticks = FIRST_STEPS_TIMES; //每一次调整角度的时间间隔长度

    uint32_t motor_starting_ticks_dt = 0;                               //电机启动时间间隔
    float uq = 0.0f;                                                    //q轴电压, 力矩

    static uint16_t motor_start_const_cnt = 0;                          //启动进入恒速转动计数, 用来切入到滑膜观测
    static uint32_t motor_running_ticks = 0;                            //电机运行时间戳

    // 串口控制电机
#if 1
    uart_cmd_t usart1_rx_data;
    uint8_t usart1_rx_len = 0;

    usart1_rx_len = usart1_rx( (uint8_t *)&usart1_rx_data );
    if(usart1_rx_len > 0)
    {
        trace_debug("u1 rx %d data:\r\n", usart1_rx_len);
        trace_dump((uint8_t *)&usart1_rx_data, usart1_rx_len);

        if(usart1_rx_data.cmd == 0)  //停止
        {
            g_app_param.motor_sta = MOTOR_STA_STOPPING;

#ifdef SIX_STEPS_ENABLE     //六步换相驱动
            g_bldc_motor.run_flag = 0;
            motor_stop();
#endif

            trace_debug("motor stop\r\n");
        }
        else if(usart1_rx_data.cmd == 1)  //启动
        {
            g_app_param.motor_sta = MOTOR_STA_STARTING;

#ifdef SIX_STEPS_ENABLE     //六步换相驱动
            g_bldc_motor.run_flag = 1;
            motor_start();
#endif

            trace_debug("motor start\r\n");
        }

        if(usart1_rx_data.dir == 0)
        {
            g_app_param.motor_dir = MOTOR_DIR_CW;

#ifdef SIX_STEPS_ENABLE     //六步换相驱动
            g_bldc_motor.dir = MOTOR_DIR_CW;
#endif

            trace_debug("motor dir MOTOR_DIR_CW\r\n");
        }
        else if(usart1_rx_data.dir == 1)
        {
            g_app_param.motor_dir = MOTOR_DIR_CCW;

#ifdef SIX_STEPS_ENABLE     //六步换相驱动
            g_bldc_motor.dir = MOTOR_DIR_CCW;
#endif

            trace_debug("motor dir MOTOR_DIR_CCW\r\n");
        }

        //speed
        if((usart1_rx_data.speed >= MOTOR_SPEED_MIN_RPM) && (usart1_rx_data.speed <= MOTOR_SPEED_MAX_RPM))
        {
            g_app_param.motor_speed_set = usart1_rx_data.speed;

            trace_debug("montor speed %d\r\n", usart1_rx_data.speed);
        }

        //uq
        if(usart1_rx_data.uq <= MOTOR_UQ_MAX)
        {
            uart1_rx_uq = usart1_rx_data.uq;        //q轴电压

            g_app_param.uq = uart1_rx_uq * 0.1f;    //转换为实际电压值

            trace_debug("montor uq %.2f\r\n", g_app_param.uq);
        }

        //iq
        if(usart1_rx_data.iq <= (MOTOR_I_MAX * 1000))
        {
            g_app_param.iq = usart1_rx_data.iq * 0.001f;    //转换为实际电流值

            trace_debug("montor iq %.2f\r\n", g_app_param.iq);
        }
    }
#endif

    // 电机状态机
    switch(g_app_param.motor_sta)
    {
        case MOTOR_STA_STOP:
            motor_starting_ticks = sys_time_ms_get();

            break;

        case MOTOR_STA_STOPPING:
            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 0);

            torque_set(0.0f, 0.0f, 0); // 停止时，电压为0

            motor_starting_ticks   = sys_time_ms_get();
            motor_acc_stats        = MOTOR_START_STA_ACC;
            motor_acc_ticks_dt     = 0;
            motor_acc_cnt          = 0;
            per_acc_hold_ticks     = FIRST_STEPS_TIMES;
            shaft_angle            = PI_DIV_4;
            motor_start_const_cnt  = 0;

            g_app_param.motor_sta  = MOTOR_STA_STOP;    //  切回到停止状态
            break;

        case MOTOR_STA_RUNNING:
            if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), motor_running_ticks, 1))      //执行间隔
            {
                motor_smo_pll_foc_run();                        // 运行无感滑膜观测、PLL锁相、FOC等处理
                //vel_current_loop_pid_handler();               // 速度、电流环PID计算
                //current_loop_pid_handler();                   // 仅电流环PID计算
                smo_torque_handler();                           // 力矩环

                motor_running_ticks = sys_time_ms_get();
            }

            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 1);
            break;

        case MOTOR_STA_STARTING:
/**
 * 参考 正点原子 六步换相 时间间隔进行加速
 */
#if 1
            if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), motor_starting_ticks,  1))      //执行间隔
            {
                motor_starting_ticks_dt = sys_time_ms_get() - motor_starting_ticks;      //计算时间间隔
                motor_starting_ticks = sys_time_ms_get();

                if(motor_acc_stats == MOTOR_START_STA_ACC)      //加速未完成
                {
                    motor_acc_ticks_dt += motor_starting_ticks_dt;
                    if(motor_acc_ticks_dt > per_acc_hold_ticks)                          //进行一次加速
                    {
                        motor_acc_cnt++;

                        per_acc_hold_ticks -= per_acc_hold_ticks / 12;                      //加速时间间隔逐渐变短
                        if(per_acc_hold_ticks < 13)
                        {
                            per_acc_hold_ticks = 12;                                        //加速时间间隔下限

                            motor_acc_stats = MOTOR_START_STA_ACC_END;                      //加速完成
                        }

                        motor_acc_ticks_dt = 0;                                          //清零时间间隔计数

                        shaft_angle += PI_DIV_3;                                            //每次加速，电机转动60°
                        shaft_angle = radian_normalize(shaft_angle);
                    }

                    uq = UQ_INIT + motor_acc_cnt * UQ_ACC_STEP;
                    uq = CONSTRAIN(uq, 0.1f, 3.0f);

                    torque_set(uq, 0, shaft_angle);
                }
                else if(motor_acc_stats == MOTOR_START_STA_ACC_END)                         //加速已完成，切换到恒速
                {
                    uq = UQ_INIT + motor_acc_cnt * UQ_ACC_STEP;
                    uq = CONSTRAIN(uq, 0.1f, 3.0f);

                    trace_debug("motor acc done, total cnts %d, uq = %.3f\r\n", motor_acc_cnt, uq);

                    motor_acc_stats = MOTOR_START_STA_CONST;
                }
                else if(motor_acc_stats == MOTOR_START_STA_CONST)                            //恒速运行
                {
                    motor_acc_ticks_dt += motor_starting_ticks_dt;
                    if(motor_acc_ticks_dt > per_acc_hold_ticks)                          //进行一次角度调整
                    {
                        motor_acc_ticks_dt = 0;

                        shaft_angle += PI_DIV_3;
                        shaft_angle = radian_normalize(shaft_angle);
                    }

                    uq = UQ_INIT + motor_acc_cnt * UQ_ACC_STEP;
                    uq = CONSTRAIN(uq, 0.1f, 3.0f);

                    torque_set(uq, 0, shaft_angle);

                    g_app_param.motor_speed_real = (uint16_t)((60.0f * 1000.0f * PI_DIV_3) / (per_acc_hold_ticks * DOUBLE_PI));     //计算当前速度，单位RPM

// 跳转到滑膜观测器 & 锁相
#if 1
                    motor_start_const_cnt++;
                    if(motor_start_const_cnt >= 3000)   //恒速一定时间后，切入到运行态，进行滑膜观察
                    {
                        motor_start_const_cnt = 0;

                        g_app_param.motor_sta  = MOTOR_STA_RUNNING;    //切换到运行态
                        motor_running_ticks = sys_time_ms_get();

                        trace_debug("motor go to MOTOR_STA_RUNNING\r\n");
                    }
#endif
                }
            }
#endif

            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 1);
            break;

        case MOTOR_STA_ERROR:

            break;
    }


    return 0;
}



