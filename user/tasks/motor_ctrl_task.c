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
#include "vofa.h"
#include "sensors_task.h"
//#include "ekf.h"

#include "app_timer.h"
#include "trace.h"

TIMER_DEF(m_speed_pid_timer);           //速度环定时器

#define UQ_INIT             0.5f        //无感启动初始q轴电压
#define UQ_ACC_STEP         0.036f      //无感启动q轴电压每次增加步进
#define FIRST_STEPS_TIMES   (60 * 20)   //无感启动初始固定相时间间隔

float   vofa_param[12] = {0.0f};

typedef struct
{
    uint8_t     cmd;            //命令字 0:停止 1:启动
    uint8_t     dir;            //方向   0:正转 1:反转
    uint16_t    speed;          //目标速度  0 ~ 4000
    uint8_t     uq;             //目标q轴电压 0 ~ 255       1 = 0.1 V
    uint16_t    iq;             //目标q轴电流 0 ~ 19800     1 = 1 mA
} __attribute__((__packed__ )) uart_cmd_t;


/**
 * 电机加速过程, 10K的执行频率
 */
static void motor_acc_start_handle(void)
{
    static float                shaft_angle = PI_DIV_4;                 //电机轴角度
    static uint16_t             motor_acc_cnt = 0;                      //电机加速次数计数
    static uint16_t             motor_acc_ticks_dt = 0;                 //累计每一次电机调整角度时间间隔
    static uint16_t             per_acc_hold_ticks = FIRST_STEPS_TIMES; //每一次调整角度的时间间隔长度

    float uq = 0.0f;                                                    //q轴电压, 力矩

    static uint16_t motor_start_const_cnt = 0;                          //启动进入恒速转动计数, 用来切入到滑膜观测

    // 电机状态机
    switch(g_app_param.motor_sta)
    {
        case MOTOR_STA_STOP:
            break;

        case MOTOR_STA_STOPPING:
            torque_set(0.0f, 0.0f, 0);                                  // 停止时，电压为0

            g_app_param.motor_start_acc_sta     = MOTOR_START_STA_ACC;
            motor_acc_ticks_dt                  = 0;
            motor_acc_cnt                       = 0;
            per_acc_hold_ticks                  = FIRST_STEPS_TIMES;
            shaft_angle                         = PI_DIV_4;
            motor_start_const_cnt               = 0;
            g_app_param.motor_sta               = MOTOR_STA_STOP;       //  切回到停止状态
            break;

        case MOTOR_STA_RUNNING:
            break;

        case MOTOR_STA_STARTING:
/**
 * 参考 正点原子 六步换相 时间间隔进行加速
 */
#if 1
                if(g_app_param.motor_start_acc_sta == MOTOR_START_STA_ACC)                  //加速未完成
                {
                    motor_acc_ticks_dt ++;
                    if(motor_acc_ticks_dt > per_acc_hold_ticks)                             //进行一次加速
                    {
                        motor_acc_cnt++;

                        per_acc_hold_ticks -= per_acc_hold_ticks / 12;                     //加速时间间隔逐渐变短
                        if(per_acc_hold_ticks < 130)
                        {
                            per_acc_hold_ticks = 100;                                       //加速时间间隔下限

                            g_app_param.motor_start_acc_sta = MOTOR_START_STA_ACC_END;      //加速完成
                        }

                        motor_acc_ticks_dt = 0;                                             //清零时间间隔计数

                        shaft_angle += PI_DIV_4;                                            //每次加速，电机转动60°
                        shaft_angle = radian_normalize(shaft_angle);
                    }

                    uq = UQ_INIT + motor_acc_cnt * UQ_ACC_STEP;
                    uq = CONSTRAIN(uq, 0.1f, 3.0f);

                    torque_set(uq, 0, shaft_angle);
                }
                else if(g_app_param.motor_start_acc_sta == MOTOR_START_STA_ACC_END)         //加速已完成，切换到恒速
                {
                    g_app_param.motor_start_acc_sta = MOTOR_START_STA_CONST;
                }
                else if(g_app_param.motor_start_acc_sta == MOTOR_START_STA_CONST)           //恒速运行
                {
                    motor_acc_ticks_dt ++;
                    if(motor_acc_ticks_dt > per_acc_hold_ticks)                             //进行一次角度调整
                    {
                        motor_acc_ticks_dt = 0;

                        shaft_angle += PI_DIV_4;
                        shaft_angle = radian_normalize(shaft_angle);

                        uq = UQ_INIT + motor_acc_cnt * UQ_ACC_STEP;
                        uq = CONSTRAIN(uq, 0.1f, 3.0f);

                        torque_set(uq, 0, shaft_angle);
                    }

// 跳转到滑膜观测器 & 锁相
#if 0
                    motor_start_const_cnt++;
                    if(motor_start_const_cnt >= 30000)                                       //恒速一定时间后，切入到运行态，进行滑膜观察
                    {
                        motor_start_const_cnt = 0;

                        g_app_param.motor_sta  = MOTOR_STA_RUNNING;                         //切换到运行态
                        g_app_param.curr_uq    = 0.0f;                                      //切换到运行态后， uq 重新重头开始
                    }
#endif
                }

            if(g_app_param.motor_sta != MOTOR_STA_RUNNING)
            {
                g_app_param.curr_uq     = uq;
                g_app_param.curr_theta  = shaft_angle;                              //接着强拖后的角度
            }
#endif
            break;

        case MOTOR_STA_ERROR:

            break;
    }
}

/**
 * 电机算法运行过程
 */
static void motor_algorithm_handle(void)
{
    float cos_theta;   //计算估算角度θ 的正弦，余弦值
    float sin_theta;

    /**
     * 强拉之后，开始进入 FOC 控制
     */
        
    if((g_app_param.motor_sta == MOTOR_STA_STARTING) || (g_app_param.motor_sta == MOTOR_STA_RUNNING))
    {

#if 0
        iq_id_cal(&g_current_foc, adc_sample_physical_value_get(ADC_CH_U_I), adc_sample_physical_value_get(ADC_CH_V_I), \
                    adc_sample_physical_value_get(ADC_CH_W_I), g_app_param.ekf_theta);

        cos_theta = cosf(g_app_param.ekf_theta);
        sin_theta = sinf(g_app_param.ekf_theta);
#endif

//        g_app_param.ekf_u_alpha = cos_theta * g_current_foc.ud - sin_theta * g_current_foc.uq;
//        g_app_param.ekf_u_beta  = sin_theta * g_current_foc.ud + cos_theta * g_current_foc.uq;

//      g_app_param.ekf_u_alpha = - sin_theta * g_current_foc.uq;
//      g_app_param.ekf_u_beta  = + cos_theta * g_current_foc.uq;

        //经pid计算后得到的 ualpha,ubeta
//      g_app_param.ekf_u_alpha = cos_theta * g_id_pi.out - sin_theta * g_iq_pi.out;
//      g_app_param.ekf_u_beta  = sin_theta * g_id_pi.out + cos_theta * g_iq_pi.out;

#if 0
        if(g_app_param.motor_sta == MOTOR_STA_RUNNING)
        {
            pi_cal(&g_iq_pi, g_app_param.target_iq - g_current_foc.i_q);
            pi_cal(&g_id_pi, 0 - g_current_foc.i_d);

            g_app_param.ekf_u_alpha = cos_theta * g_id_pi.out - sin_theta * g_iq_pi.out;
            g_app_param.ekf_u_beta  = sin_theta * g_id_pi.out + cos_theta * g_iq_pi.out;
        }
        else
        {
            g_app_param.ekf_u_alpha = cos_theta * g_current_foc.ud - sin_theta * g_current_foc.uq;
            g_app_param.ekf_u_beta  = sin_theta * g_current_foc.ud + cos_theta * g_current_foc.uq;
        }
#endif

#if 0
        if(g_app_param.motor_sta == MOTOR_STA_RUNNING)
        {
            ekf_torque_handler();                           // 力矩环
        }
#endif

#if 0
        g_ekf_data.ekf_input[0] = g_app_param.ekf_u_alpha;
        g_ekf_data.ekf_input[1] = g_app_param.ekf_u_beta;
        g_ekf_data.ekf_input[2] = g_current_foc.i_alpha;
        g_ekf_data.ekf_input[3] = g_current_foc.i_beta;
        g_ekf_data.ekf_input[4] = MOTOR_PHASE_RES;                //电阻
        g_ekf_data.ekf_input[5] = MOTOR_PHASE_LS;                 //电感
        g_ekf_data.ekf_input[6] = MOTOR_FLUXLINK;                 //磁链

        apt_ekf_update(&g_ekf_data.ekf_input[0], &g_ekf_data.ekf_states[0]);

        g_app_param.ekf_theta       = g_ekf_data.ekf_states[3];  //取出估计角度
        g_app_param.ekf_angle_speed = g_ekf_data.ekf_states[2];
#endif


#if 0
        if(g_app_param.motor_sta == MOTOR_STA_RUNNING)
        {
            g_app_param.curr_uq = g_iq_pi.out;
            torque_set(g_app_param.curr_uq, g_id_pi.out, g_app_param.ekf_theta);
        }
#endif
        }
}

uint16_t tim8_irq_cnt = 0;
/**
 * timer8 CCH4 中断回调函数 10KHz的执行频率
 */
static void timer8_irq_cb_handler(void)
{
    tim8_irq_cnt++;

    gpio_output_set(TEST1_IO_PORT, TEST1_IO_PIN, 1);
    adc_inj_start();        //每一次中断触发一次电流采集 10K 的执行频率

/**
 * 原来cxn的启动 和 EKF 观测
 */
#if 0
        motor_acc_start_handle();
        motor_algorithm_handle();
#endif
}

/**
 * 电机运行
 */
void motor_run(void)
{
    // 电机状态机
    switch(g_app_param.motor_sta)
    {
        case MOTOR_STA_STOPPING:
            break;

        case MOTOR_STA_RUNNING:
            break;

        case MOTOR_STA_STARTING:
            if(g_app_param.motor_start_acc_sta == MOTOR_START_STA_ACC)              //加速未完成
            {
                if(g_app_param.iq_acc_dir == ACC_START)                             //Iq发生改变，开始调整Iq
                {
                    if(g_app_param.curr_iq < g_app_param.target_iq)
                    {
                        g_app_param.iq_acc_dir = ACC_UP;
                    }
                    else
                    {
                        g_app_param.iq_acc_dir = ACC_DOWN;
                    }
                }

                if(g_app_param.iq_acc_dir == ACC_UP)    //iq 加速
                {
                    g_app_param.curr_iq += 0.001f;  //步进

                    if(g_app_param.curr_iq > g_app_param.target_iq)
                    {
                        g_app_param.iq_acc_dir = ACC_DONE;
                        g_app_param.curr_iq = g_app_param.target_iq;
                    }
                }
                else if(g_app_param.iq_acc_dir == ACC_DOWN) //iq 减速
                {
                    g_app_param.curr_iq -= 0.001f;  //步进

                    if(g_app_param.curr_iq < g_app_param.target_iq)
                    {
                        g_app_param.iq_acc_dir = ACC_DONE;
                        g_app_param.curr_iq = g_app_param.target_iq;
                    }
                }

                if( !g_app_param.is_speed_ring_start )                  //速度闭环未开始
                {
                    g_FOC_Input.Iq_ref = g_app_param.curr_iq;           //速度还没有闭环之前, 使用受限Iq，避免跑飞
                    g_Speed_Pid.I_Sum  = g_app_param.curr_iq;

                    if(g_FOC_Output.EKF[2] > SPEED_LOOP_CLOSE_RAD_S)    //检测速度是否达标速度闭环
                    {
                        g_app_param.is_speed_ring_start = true;
                    }
                }
                else                                                    //开始速度闭环
                {
                    g_Speed_Fdk         = g_FOC_Output.EKF[2];          //使用卡尔曼估算的角速度
                    g_FOC_Input.Iq_ref  = g_Speed_Pid_Out;              //使用速度环的输出值作为目标Iq
                }

                g_FOC_Input.theta = g_FOC_Output.EKF[3];    //因为没有使用高频注入--所以没有角度切换--直接一开始就是用卡尔曼估算角度
                g_Speed_Fdk       = g_FOC_Output.EKF[2];

                g_FOC_Input.Udc     = adc_sample_physical_value_get(ADC_CH_VBUS);
                g_FOC_Input.ia      = adc_sample_physical_value_get(ADC_CH_U_I);
                g_FOC_Input.ib      = adc_sample_physical_value_get(ADC_CH_V_I);
                g_FOC_Input.ic      = adc_sample_physical_value_get(ADC_CH_W_I);
                g_FOC_Input.Id_ref  = 0.0f;

                //计算好后赋值到PWM_CCRX比较寄存器通道
   	            foc_algorithm_step();

                TIM8->CCR1 = (uint16_t)(g_FOC_Output.Tcmp1);     
	            TIM8->CCR2 = (uint16_t)(g_FOC_Output.Tcmp2);
	            TIM8->CCR3 = (uint16_t)(g_FOC_Output.Tcmp3);
            }
            else if(g_app_param.motor_start_acc_sta == MOTOR_START_STA_ACC_END)     //加速已完成，切换到恒速
            {

            }
            else if(g_app_param.motor_start_acc_sta == MOTOR_START_STA_CONST)       //恒速运行
            {
                
            }
            break;

        case MOTOR_STA_ERROR:
            break;
    }
}


/**
 *发送串口数据到vofa显示
 */
static void vofa_send(void)
{

#if 0
    vofa_param[0] = adc_sample_physical_value_get(ADC_CH_U_I);
    vofa_param[1] = adc_sample_physical_value_get(ADC_CH_V_I);
    vofa_param[2] = g_app_param.curr_uq;
    vofa_param[3] = g_app_param.curr_theta;
    vofa_param[4] = g_app_param.ekf_theta;
    vofa_param[5] = g_app_param.ekf_angle_speed;

    VOFA_PRINTF("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", \
                        vofa_param[0], \
                        vofa_param[1], \
                        vofa_param[2], \
                        vofa_param[3], \
                        vofa_param[4], \
                        vofa_param[5], \
                        vofa_param[6], \
                        vofa_param[7]);
#endif

    static uint8_t tx_idx = 1;

    switch (tx_idx)
    {
        case 0:
            justfloat_update(g_FOC_Output.EKF[3], 0);   //卡尔曼估算角度
	        justfloat_update(PLL_def.theta, 0);         //SMO估算角度
	        justfloat_update(g_FOC_Output.EKF[2], 0);   //卡尔曼估算速度
	        justfloat_update(PLL_def.we, 1);            //SMO角速度
        break;
    
        case 1:
            justfloat_update(g_FOC_Input.Iq_ref,  0);
            justfloat_update(g_FOC_Output.EKF[3], 0);   //卡尔曼估算角度
            justfloat_update(g_FOC_Output.EKF[2], 0);   //卡尔曼估算速度
//            justfloat_update(PLL_def.theta, 0);         //SMO估算角度
//            justfloat_update(PLL_def.we,    0);            //SMO角速度

            justfloat_update(adc_sample_physical_value_get(ADC_CH_U_I), 0);
	        justfloat_update(adc_sample_physical_value_get(ADC_CH_V_I), 0);
	        justfloat_update(adc_sample_physical_value_get(ADC_CH_W_I), 1);

        break;

        default:
            break;
    }
    
//    tx_idx++;
    if(tx_idx > 1)
    {
        tx_idx = 0;
    }
}

static void speed_pid_timer_handler(void *p_data)
{
    //速度环执行
    Speed_Pid_Calc(g_Speed_Ref, g_Speed_Fdk, &g_Speed_Pid_Out, &g_Speed_Pid);
}

/**
 * 电机控制逻辑任务
 */
int motor_ctrl_task(void)
{
    static bool init_done = false;

    if(!init_done)
    {
        init_done = true;
        timer8_irq_cb_register(timer8_irq_cb_handler);      //回调函数注册到 timer8 的中断函数里面

        TIMER_CREATE(&m_speed_pid_timer, false, true, speed_pid_timer_handler);     //循环定时器，立马执行
        TIMER_START(m_speed_pid_timer, 1);                                          // 1Kz的执行频率
    }

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

            trace_debug("motor stop\r\n");
        }
        else if(usart1_rx_data.cmd == 1)  //启动
        {
            g_app_param.motor_sta   = MOTOR_STA_STARTING;
            g_app_param.iq_acc_dir  = ACC_START;

            trace_debug("motor start\r\n");
        }

        if(usart1_rx_data.dir == 0)
        {
            g_app_param.motor_dir = MOTOR_DIR_CW;

            trace_debug("motor dir MOTOR_DIR_CW\r\n");
        }
        else if(usart1_rx_data.dir == 1)
        {
            g_app_param.motor_dir = MOTOR_DIR_CCW;

            trace_debug("motor dir MOTOR_DIR_CCW\r\n");
        }

        //speed
        if((usart1_rx_data.speed >= MOTOR_SPEED_MIN_RPM) && (usart1_rx_data.speed <= MOTOR_SPEED_MAX_RPM))
        {
            g_app_param.motor_speed_set = usart1_rx_data.speed;

            g_Speed_Ref = usart1_rx_data.speed;

            trace_debug("montor speed %d\r\n", usart1_rx_data.speed);
        }

        //uq
        if(usart1_rx_data.uq <= MOTOR_UQ_MAX)
        {
            g_app_param.target_uq = usart1_rx_data.uq * 0.1f;    //q轴电压 转换为实际电压值

            trace_debug("montor uq %.2f\r\n", g_app_param.target_uq);
        }

        //iq
        if(usart1_rx_data.iq <= (MOTOR_I_MAX * 1000))
        {
            g_app_param.target_iq = usart1_rx_data.iq * 0.001f;    //转换为实际电流值

            if((g_app_param.target_iq > 3.0f) || (g_app_param.target_iq < -3.0f))
            {
                g_app_param.target_iq = 0.0f;
            }

            trace_debug("montor iq %.2f\r\n", g_app_param.target_iq);
        }
    }
#endif

    // 电机状态机
    switch(g_app_param.motor_sta)
    {
        case MOTOR_STA_STOP:
            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 0);
            break;

        case MOTOR_STA_STOPPING:
            if(g_app_param.motor_sta != g_app_param.pre_motor_sta)  //开始停机
            {
                phase_pwm_stop();

                g_app_param.is_speed_ring_start = false;            //参数恢复
                g_app_param.curr_iq = 0.0f;
            }

            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 0);

            g_app_param.motor_sta = MOTOR_STA_STOP;
            break;

        case MOTOR_STA_RUNNING:
            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 1);
            break;

        case MOTOR_STA_STARTING:
            if(g_app_param.motor_sta != g_app_param.pre_motor_sta)  //每一次启动都要foc参数初始化
            {
                phase_pwm_start();

                foc_algorithm_initialize();   //FOC 算法参数初始化
            }

            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 1);
            break;

        case MOTOR_STA_ERROR:
            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 0);
            break;
    }

#ifndef TRACE_ENABLE
        vofa_send();    //vofa 显示
#endif

    if(g_app_param.motor_sta != g_app_param.pre_motor_sta)
    {
        g_app_param.pre_motor_sta = g_app_param.motor_sta;
    }

#if 0
    if(tim8_irq_cnt >= 10000)
    {
        tim8_irq_cnt = 0;

        trace_debug("sys time ms %lu\r\n", sys_time_ms_get());

    }
#endif

    

    return 0;
}



