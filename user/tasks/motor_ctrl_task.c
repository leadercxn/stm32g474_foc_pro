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

TIMER_DEF(m_speed_pid_timer);                   //速度环定时器

static float m_curr_speed_ring_s = 0.0f;        //目标速度 圈/秒, 用于目标速度递进

typedef enum
{
    CMD_SW = 1,             //开关
    CMD_TARGET_SPEED,       //目标速度
    CMD_TARGET_IQ,          //目标d轴电流
    CMD_TARGET_UQ,          //目标q轴电压
    CMD_VF_STEP_RAD_S,      //目标步进幅度
    CMD_DIR,                //方向
} uart_cmd_e;

typedef union
{
  	float       fdate;
	uint32_t    udata;
} float_uint32_u;

typedef struct
{
    uart_cmd_e      cmd;
    float_uint32_u  data;
} __attribute__((__packed__ )) uart_cmd_t;

float   vofa_param[12] = {0.0f};

uint16_t tim8_irq_cnt = 0;
/**
 * timer8 CCH4 中断回调函数 10KHz的执行频率
 */
static void timer8_irq_cb_handler(void)
{
    tim8_irq_cnt++;

    gpio_output_set(TEST1_IO_PORT, TEST1_IO_PIN, 1);
    adc_inj_start();        //每一次中断触发一次电流采集 10K 的执行频率
}

/**
 * 电机运行
 */
void motor_run(void)
{
    // 电机状态机
    switch(g_app_param.motor_sta)
    {
        case MOTOR_STA_STOP:
            break;

        case MOTOR_STA_STARTING:
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
                    g_foc_input.iq_ref = g_app_param.curr_iq;           //速度还没有闭环之前, 使用受限Iq，避免跑飞
                    g_speed_pid.i_sum  = g_app_param.curr_iq;

                    if(g_foc_output.ekf[2] > SPEED_LOOP_CLOSE_RAD_S)    //检测速度是否达标速度闭环
                    {
                        g_app_param.is_speed_ring_start = true;
                    }
                }
                else                                                    //开始速度闭环
                {
                    g_app_param.curr_speed_rad_s         = g_foc_output.ekf[2];          //使用卡尔曼估算的角速度
                    g_foc_input.iq_ref  = g_speed_pid_out;              //使用速度环的输出值作为目标Iq
                }

                g_foc_input.theta = g_foc_output.ekf[3];    //因为没有使用高频注入--所以没有角度切换--直接一开始就是用卡尔曼估算角度
                g_app_param.curr_speed_rad_s       = g_foc_output.ekf[2];

                g_foc_input.udc     = adc_sample_physical_value_get(ADC_CH_VBUS);
                g_foc_input.ia      = adc_sample_physical_value_get(ADC_CH_U_I);
                g_foc_input.ib      = adc_sample_physical_value_get(ADC_CH_V_I);
                g_foc_input.ic      = adc_sample_physical_value_get(ADC_CH_W_I);
                g_foc_input.id_ref  = 0.0f;

                //计算好后赋值到PWM_CCRX比较寄存器通道
   	            foc_algorithm_step();

                TIM8->CCR1 = (uint16_t)(g_foc_output.tcmp1);     
	            TIM8->CCR2 = (uint16_t)(g_foc_output.tcmp2);
	            TIM8->CCR3 = (uint16_t)(g_foc_output.tcmp3);
            break;

        case MOTOR_STA_ERROR:
            break;
    }
}

/**
 * 电机vf运行
 */
void motor_vf_run(void)
{
    static uint16_t vf_start_cnt = 0;
    // 电机状态机
    switch(g_app_param.motor_sta)
    {
        case MOTOR_STA_STOP:
            break;

        case MOTOR_STA_STARTING:
                if(g_app_param.iq_acc_dir == ACC_START)                             //Iq发生改变，开始调整Iq
                {
                    if(g_app_param.vf_curr_uq < g_app_param.vf_target_uq)
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
                    g_app_param.vf_curr_uq += 0.001f;  //步进

                    if(g_app_param.vf_curr_uq > g_app_param.vf_target_uq)
                    {
                        g_app_param.iq_acc_dir = ACC_DONE;
                        g_app_param.vf_curr_uq = g_app_param.vf_target_uq;
                    }
                }
                else if(g_app_param.iq_acc_dir == ACC_DOWN) //iq 减速
                {
                    g_app_param.vf_curr_uq -= 0.001f;  //步进

                    if(g_app_param.vf_curr_uq < g_app_param.vf_target_uq)
                    {
                        g_app_param.iq_acc_dir = ACC_DONE;
                        g_app_param.vf_curr_uq = g_app_param.vf_target_uq;
                    }
                }

                if( !g_app_param.is_speed_ring_start )                  //速度闭环未开始
                {
                    g_foc_input.theta  = g_app_param.vf_curr_theta;
                    g_foc_input.iq_ref = g_app_param.vf_curr_uq;

                    //速度稳定后切入到速度环
                    if( (g_foc_output.ekf[2] > 40.0f) || (g_foc_output.ekf[2] < -40.0f) )    //检测速度是否达标速度闭环
                    {
                        vf_start_cnt++;
                        if(vf_start_cnt > 100)                         //速度环达标超 0.1 * 100 ms 后，转到速度闭环
                        {
                            vf_start_cnt = 0;
                            g_app_param.is_speed_ring_start = true;

                            TIMER_START(m_speed_pid_timer, 1);         //1K的执行频率
                        }
                    }
                    else
                    {
                        vf_start_cnt = 0;
                    }
                }
                else
                {
// 使用卡尔曼
#if 1
                    g_app_param.curr_speed_rad_s = g_foc_output.ekf[2];          //使用卡尔曼估算的角速度
                    g_foc_input.theta            = g_foc_output.ekf[3];          //使用卡尔曼估算角度
#endif

// 使用PLL
#if 0
                    g_app_param.curr_speed_rad_s         = g_pll.we;          //使用 PLL 估算的角速度
                    g_foc_input.theta   = g_pll.theta;       //使用 PLL 估算角度
#endif
                    g_foc_input.iq_ref  = g_speed_pid_out;              //使用速度环的输出值作为目标Iq
                }

                g_foc_input.udc     = adc_sample_physical_value_get(ADC_CH_VBUS);
                g_foc_input.ia      = adc_sample_physical_value_get(ADC_CH_U_I);
                g_foc_input.ib      = adc_sample_physical_value_get(ADC_CH_V_I);
                g_foc_input.ic      = adc_sample_physical_value_get(ADC_CH_W_I);
                g_foc_input.id_ref  = 0.0f;

                //计算好后赋值到PWM_CCRX比较寄存器通道
   	            foc_algorithm_step();

                TIM8->CCR1 = (uint16_t)(g_foc_output.tcmp1);     
	            TIM8->CCR2 = (uint16_t)(g_foc_output.tcmp2);
	            TIM8->CCR3 = (uint16_t)(g_foc_output.tcmp3);
            break;

        case MOTOR_STA_ERROR:
            break;
    }
}

/**
 * 电机if运行
 */
void motor_if_run(void)
{
    // 电机状态机
    switch(g_app_param.motor_sta)
    {
        case MOTOR_STA_STOP:
            break;

        case MOTOR_STA_STARTING:
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
                    g_if_start_def.if_abs_time++;
                    if_start_algorithm(&g_foc_input.iq_ref, &g_foc_input.theta, &g_if_start_def);
                    g_speed_pid.i_sum = g_app_param.curr_iq;;


                    if(g_foc_output.ekf[2] > SPEED_LOOP_CLOSE_RAD_S)    //检测速度是否达标速度闭环
                    {
                        g_app_param.is_speed_ring_start = true;
                    }
                }
                else                                                    //开始速度闭环
                {
                    g_foc_input.theta   = g_foc_output.ekf[3];          //使用卡尔曼估算角度
                    g_app_param.curr_speed_rad_s         = g_foc_output.ekf[2];          //使用卡尔曼估算的角速度
                    g_foc_input.iq_ref  = g_speed_pid_out;              //使用速度环的输出值作为目标Iq
                }

                g_foc_input.udc     = adc_sample_physical_value_get(ADC_CH_VBUS);
                g_foc_input.ia      = adc_sample_physical_value_get(ADC_CH_U_I);
                g_foc_input.ib      = adc_sample_physical_value_get(ADC_CH_V_I);
                g_foc_input.ic      = adc_sample_physical_value_get(ADC_CH_W_I);
                g_foc_input.id_ref  = 0.0f;

                //计算好后赋值到PWM_CCRX比较寄存器通道
   	            foc_algorithm_step();

                TIM8->CCR1 = (uint16_t)(g_foc_output.tcmp1);     
	            TIM8->CCR2 = (uint16_t)(g_foc_output.tcmp2);
	            TIM8->CCR3 = (uint16_t)(g_foc_output.tcmp3);
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
    justfloat_update(g_foc_output.ekf[3], 0);       //卡尔曼估算角度 -- 0
    justfloat_update(g_foc_output.ekf[2], 0);       //卡尔曼估算速度 -- 1
    justfloat_update(g_pll.theta, 0);               //SMO估算角度   -- 2
    justfloat_update(g_pll.we,    0);               //SMO角速度     -- 3
    justfloat_update(g_current_dq.iq,    0);        //当前Iq        -- 4
    justfloat_update(g_foc_input.iq_ref,    0);     //目标Iq        -- 5
    justfloat_update(g_voltage_dq.vq,    0);        //实际的Vq      -- 6
    justfloat_update(g_app_param.target_speed_ring_s, 0);  //目标speed     -- 7
    justfloat_update(g_app_param.vf_curr_theta,  1);   //强拖的角度     -- 8
}

/**
 * 速度环回调函数
 */
static void speed_pid_timer_handler(void *p_data)
{
    //速度环执行
    speed_pid_cal(g_app_param.target_speed_ring_s, g_app_param.curr_speed_rad_s, &g_speed_pid_out, &g_speed_pid);
}

static void usart_ctrl_cmd_handler(void)
{
    //串口控制命令处理
    uart_cmd_t  usart1_rx_data;
    uint8_t     usart1_rx_len = 0;

    usart1_rx_len = usart1_rx( (uint8_t *)&usart1_rx_data );
    if(usart1_rx_len > 0)
    {
        trace_debug("u1 rx %d data:\r\n", usart1_rx_len);
        trace_dump((uint8_t *)&usart1_rx_data, usart1_rx_len);

        if(usart1_rx_len == 5)      //目前VOFA个人设置只发送5字节数据
        {
            switch (usart1_rx_data.cmd)
            {
                case CMD_SW:
                    if(usart1_rx_data.data.udata == 0x0)                //关机控件
                    {
                        g_app_param.motor_cmd   = MOTOR_CMD_STOP;
                        trace_debug("motor stop\r\n");
                    }
                    else if(usart1_rx_data.data.udata == 0x3F800000)    //开机控件
                    {
                        g_app_param.motor_cmd   = MOTOR_CMD_STARTUP;
                        trace_debug("motor start\r\n");
                    }
                    break;

                case CMD_TARGET_SPEED:
                        if(g_app_param.motor_dir == MOTOR_DIR_CCW)  //逆
                        {
                            if(usart1_rx_data.data.fdate < 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }
                        else                                        //顺
                        {
                            if(usart1_rx_data.data.fdate > 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }

                        g_app_param.target_speed_ring_s = usart1_rx_data.data.fdate;

                        trace_debug("target speed %.4f\r\n", usart1_rx_data.data.fdate);
                    break;

                case CMD_TARGET_IQ:
                        if(g_app_param.motor_dir == MOTOR_DIR_CCW)  //逆
                        {
                            if(usart1_rx_data.data.fdate < 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }
                        else                                        //顺
                        {
                            if(usart1_rx_data.data.fdate > 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }

                        g_app_param.target_iq = usart1_rx_data.data.fdate;
                        if((g_app_param.target_iq > 6.0f) || (g_app_param.target_iq < -6.0f))
                        {
                            g_app_param.target_iq = 0.0f;
                        }

                        trace_debug("target Iq %.4f\r\n", usart1_rx_data.data.fdate);
                    break;

                case CMD_TARGET_UQ:
                        g_app_param.vf_target_uq  = usart1_rx_data.data.fdate;
                        g_app_param.iq_acc_dir = ACC_START;

                        trace_debug("target Uq %.4f\r\n", usart1_rx_data.data.fdate);
                    break;

                case CMD_VF_STEP_RAD_S:
                        if(g_app_param.motor_dir == MOTOR_DIR_CCW)  //逆
                        {
                            if(usart1_rx_data.data.fdate < 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }
                        else                                        //顺
                        {
                            if(usart1_rx_data.data.fdate > 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }

                        g_app_param.vf_step_rad = usart1_rx_data.data.fdate;

                        trace_debug("vf step rad_s %.4f\r\n", usart1_rx_data.data.fdate);
                    break;

                case CMD_DIR:
                    if(usart1_rx_data.data.udata == 0x0)                //控件数据
                    {
                        g_app_param.motor_dir = MOTOR_DIR_CCW;
                        trace_debug("dir cw\r\n");

                        //改变了方向，数据的正负极性也要修改
                        if(g_app_param.target_speed_ring_s < 0.0f)
                        {
                            g_app_param.target_speed_ring_s = -g_app_param.target_speed_ring_s;
                        }

                        if(g_app_param.target_iq < 0.0f)
                        {
                            g_app_param.target_iq = -g_app_param.target_iq;
                        }

                        if(g_app_param.vf_step_rad < 0.0f)
                        {
                            g_app_param.vf_step_rad = -g_app_param.vf_step_rad;
                        }
                    }
                    else if(usart1_rx_data.data.udata == 0x3F800000)    //控件数据
                    {
                        g_app_param.motor_dir = MOTOR_DIR_CW;
                        trace_debug("dir ccw\r\n");

                        //改变了方向，数据的正负极性也要修改
                        if(g_app_param.target_speed_ring_s > 0.0f)
                        {
                            g_app_param.target_speed_ring_s = -g_app_param.target_speed_ring_s;
                        }

                        if(g_app_param.target_iq > 0.0f)
                        {
                            g_app_param.target_iq = -g_app_param.target_iq;
                        }

                        if(g_app_param.vf_step_rad > 0.0f)
                        {
                            g_app_param.vf_step_rad = -g_app_param.vf_step_rad;
                        }
                    }
                    break;
                
                default:
                    break;
            }
        }
    }
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
    }

    usart_ctrl_cmd_handler();    //串口控制命令处理

    if(g_app_param.motor_cmd != g_app_param.old_motor_cmd)
    {
        if(g_app_param.motor_cmd == MOTOR_CMD_STARTUP)
        {
            g_app_param.motor_sta   = MOTOR_STA_STARTING;
            g_app_param.iq_acc_dir  = ACC_START;
        }
        else if(g_app_param.motor_cmd == MOTOR_CMD_STOP)
        {
            g_app_param.motor_sta = MOTOR_STA_STOP;
        }

        g_app_param.old_motor_cmd = g_app_param.motor_cmd;
    }

    // 电机状态机
    switch(g_app_param.motor_sta)
    {
        case MOTOR_STA_STOP:
            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 0);

            if(g_app_param.motor_sta != g_app_param.pre_motor_sta)  //开始停机
            {
                phase_pwm_stop();

                g_app_param.is_speed_ring_start = false;            //参数恢复
                g_app_param.curr_iq = 0.0f;
                g_app_param.vf_curr_uq = 0.0f;
                g_app_param.vf_curr_theta = 0.0f;
                g_app_param.iq_acc_dir = ACC_DONE;

                foc_algorithm_init();                               //FOC 算法参数初始化

                TIMER_STOP(m_speed_pid_timer);
            }
            break;

        case MOTOR_STA_STARTING:
            if(g_app_param.motor_sta != g_app_param.pre_motor_sta)  //每一次启动都要foc参数初始化
            {
                if_start_param_init();                              //IF启动参数初始化

                foc_algorithm_init();                               //FOC 算法参数初始化

                phase_pwm_start();
            }

            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 1);
            break;

        case MOTOR_STA_ERROR:
            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 0);
            break;

        default:
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
