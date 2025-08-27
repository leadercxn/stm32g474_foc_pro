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

#include "trace.h"

#define MOTOR_VELOCITY_ACC  0.01f    //电机速度加速度
#define MOTOR_VELOCITY_MAX  250.0f  //电机速度上限

#define FIRST_STEPS_TIMES   60    //无感启动初始固定相时间

typedef struct
{
    uint8_t     cmd;            //命令字 0:停止 1:启动
    uint8_t     dir;            //方向   0:正转 1:反转
    uint16_t    speed;          //速度  0 ~ 4000

    uint8_t     uq;             //q轴电压 0 ~ 255   1 = 0.1 V
} __attribute__((__packed__ )) uart_cmd_t;


/**
 * 电机控制逻辑任务
 */
int motor_ctrl_task(void)
{

    static uint8_t uart1_rx_uq = 0;

    //恒速启动变量
    static uint32_t motor_starting_ticks = 0;               //电机启动时间戳
    uint32_t motor_starting_ticks_delta = 0;                //电机启动时间间隔
    static float shaft_angle = 0.0f;                        //电机轴角度

    // 固定加速度应用变量
    static uint64_t per_tim8_50us_ticks = 0;    //系统50us计时的时间戳
    float           tim8_50us_delta = 0;
    static float    motor_vel = 0.0f;           //电机速度
    float           uq = 0.0f;                  //q轴电压
    static uint8_t  motor_acc_stats = 0;        //加速状态 0：加速中 1：加速完成 2：恒速转动
    static uint32_t motor_acc_total_times = 0;  //加速总时间 1 = 50us

    //参考六步换相时间
    static uint32_t motor_acc_cnt = 0;                          //电机加速次数计数
    static uint32_t motor_acc_ticks_delta = 0;                  //电机加速时间间隔
    static uint32_t per_acc_hold_ticks = FIRST_STEPS_TIMES;     //电机加速时间间隔时间戳

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

            g_bldc_motor.dir = MOTOR_DIR_CW;
        }
        else if(usart1_rx_data.dir == 1)
        {
            g_app_param.motor_dir = MOTOR_DIR_CCW;

            g_bldc_motor.dir = MOTOR_DIR_CCW;
        }

        if((usart1_rx_data.speed >= MOTOR_SPEED_MIN_RPM) && (usart1_rx_data.speed <= MOTOR_SPEED_MAX_RPM))
        {
            g_app_param.motor_speed_set = usart1_rx_data.speed;

            trace_debug("montor speed %d\r\n", usart1_rx_data.speed);
        }

        if(usart1_rx_data.uq <= MOTOR_UQ_MAX)
        {
            uart1_rx_uq = usart1_rx_data.uq;        //q轴电压

            g_app_param.uq = uart1_rx_uq * 0.1f;    //转换为实际电压值

            trace_debug("montor uq %d\r\n", uart1_rx_uq);
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

            motor_starting_ticks = sys_time_ms_get();
            motor_acc_stats = 0;
            motor_acc_ticks_delta = 0;
            motor_acc_cnt = 0;
            per_acc_hold_ticks = FIRST_STEPS_TIMES;
            shaft_angle = 0;
            break;

        case MOTOR_STA_RUNNING:

            break;

        case MOTOR_STA_STARTING:

// 以固定加速度进行启动
#if 0
            if(motor_acc_stats == 0)      //加速未完成
            {
                tim8_50us_delta = (g_tim8_50us_ticks - per_tim8_50us_ticks);  //计算与上次的时间间隔
                motor_acc_total_times += tim8_50us_delta;

                tim8_50us_delta = tim8_50us_delta * 1e-6f * 50;             //50us定时器，转换为秒
                if(tim8_50us_delta <= 0 || tim8_50us_delta > 0.01f)
                {
                    tim8_50us_delta = 1e-3f;
                }
                
                if(motor_vel < MOTOR_VELOCITY_MAX)                         //速度上限
                {
                    motor_vel = motor_vel + MOTOR_VELOCITY_ACC;             //加速度
                }
                else
                {
                    motor_vel = MOTOR_VELOCITY_MAX ;

                    motor_acc_stats = 1;                                    //加速完成
                }

                shaft_angle = radian_normalize(shaft_angle + motor_vel * tim8_50us_delta);

                uq = 0.3f + motor_vel * 0.001;

                uq = CONSTRAIN(uq, 0.1f, 4.0f);

                torque_set(uq, 0, shaft_angle);

                per_tim8_50us_ticks = g_tim8_50us_ticks;               //更新50us定时器时间戳
            }
            else if(motor_acc_stats == 1)   //加速启动完成, 以恒定速度运转
            {
                uq = 0.6f + motor_vel * 0.006;

                trace_debug("motor acc done, total times %lu, uq = %.3f\r\n", motor_acc_total_times, uq);

                motor_acc_stats = 2;
            }
            else if(motor_acc_stats == 2)  //恒速运转
            {
                tim8_50us_delta = (g_tim8_50us_ticks - per_tim8_50us_ticks);  //计算与上次的时间间隔
                motor_acc_total_times += tim8_50us_delta;

                tim8_50us_delta = tim8_50us_delta * 1e-6f * 50;             //50us定时器，转换为秒
                if(tim8_50us_delta <= 0 || tim8_50us_delta > 0.01f)
                {
                    tim8_50us_delta = 1e-3f;
                }

                shaft_angle = radian_normalize(shaft_angle + motor_vel * tim8_50us_delta);

                uq = 0.3f + motor_vel * 0.001;

                uq = CONSTRAIN(uq, 0.3f, 4.0f);

                torque_set(uq, 0, shaft_angle);

                per_tim8_50us_ticks = g_tim8_50us_ticks;               //更新50us定时器时间戳
            }
#endif

/**
 * 以固定速度启动电机
 */
#if 0
            if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), motor_starting_ticks,  1))      //间隔
            {
                motor_starting_ticks_delta = sys_time_ms_get() - motor_starting_ticks;  //计算时间间隔

                shaft_angle += (g_app_param.motor_speed_set / 60.0f) * DOUBLE_PI * 0.001f * motor_starting_ticks_delta; //每1ms转动的角度
                shaft_angle = radian_normalize(shaft_angle);
                

                g_app_param.uq = CONSTRAIN(g_app_param.uq, 5.0f, 12.0f);            //q轴电压限幅

                torque_set(g_app_param.uq, 0, shaft_angle);

                motor_starting_ticks = sys_time_ms_get();
            }
#endif

/**
 * 参考六步换相时间进行加速
 */
#if 1
            if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), motor_starting_ticks,  1))      //间隔
            {
                motor_starting_ticks_delta = sys_time_ms_get() - motor_starting_ticks;      //计算时间间隔
                motor_starting_ticks = sys_time_ms_get();

                if(motor_acc_stats == 0)      //加速未完成
                {
                    motor_acc_ticks_delta += motor_starting_ticks_delta;
                    if(motor_acc_ticks_delta > per_acc_hold_ticks)                          //进行一次加速
                    {
                        motor_acc_cnt++;

                        per_acc_hold_ticks -= per_acc_hold_ticks / 12;                      //加速时间间隔逐渐变短
                        if(per_acc_hold_ticks < 13)
                        {
                            per_acc_hold_ticks = 12;                                         //加速时间间隔下限

                            motor_acc_stats = 1;                                             //加速完成
                        }

                        motor_acc_ticks_delta = 0;

                        shaft_angle += PI_DIV_3;                                            //每次加速，电机转动60度
                        shaft_angle = radian_normalize(shaft_angle);
                    }

                    uq = 0.5f + motor_acc_cnt * 0.036f;
                    uq = CONSTRAIN(uq, 0.1f, 3.0f);

                    torque_set(uq, 0, shaft_angle);
                }
                else if(motor_acc_stats == 1) //加速已完成，切换到恒速
                {
                    uq = 0.5f + motor_acc_cnt * 0.036f;
                    uq = CONSTRAIN(uq, 0.1f, 3.0f);

                    trace_debug("motor acc done, total cnts %lu, uq = %.3f\r\n", motor_acc_cnt, uq);

                    motor_acc_stats = 2;
                }
                if(motor_acc_stats == 2)      //恒速运行
                {
                    motor_acc_ticks_delta += motor_starting_ticks_delta;
                    if(motor_acc_ticks_delta > per_acc_hold_ticks)      //进行一次角度调整
                    {
                        motor_acc_ticks_delta = 0;

                        shaft_angle += PI_DIV_3;                        // 1ms 转 6°
                        shaft_angle = radian_normalize(shaft_angle);
                    }

                    uq = 0.5f + motor_acc_cnt * 0.036f;
                    uq = CONSTRAIN(uq, 0.1f, 3.0f);

                    torque_set(uq, 0, shaft_angle);
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



