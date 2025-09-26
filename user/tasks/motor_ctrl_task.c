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

typedef enum
{
    CMD_SW = 1,             //开关
    CMD_TARGET_SPEED,       //目标速度
    CMD_TARGET_IQ,          //目标d轴电流
    CMD_TARGET_UQ,          //目标q轴电压
    CMD_TARGET_STEP_ANGLE,  //目标步进幅度
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
 * 电机vf运行
 */
void motor_vf_run(void)
{
    static uint16_t vf_start_cnt = 0;
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
                    if(g_app_param.curr_uq < g_app_param.target_uq)
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
                    g_app_param.curr_uq += 0.001f;  //步进

                    if(g_app_param.curr_uq > g_app_param.target_uq)
                    {
                        g_app_param.iq_acc_dir = ACC_DONE;
                        g_app_param.curr_uq = g_app_param.target_uq;
                    }
                }
                else if(g_app_param.iq_acc_dir == ACC_DOWN) //iq 减速
                {
                    g_app_param.curr_uq -= 0.001f;  //步进

                    if(g_app_param.curr_uq < g_app_param.target_uq)
                    {
                        g_app_param.iq_acc_dir = ACC_DONE;
                        g_app_param.curr_uq = g_app_param.target_uq;
                    }
                }

                if( !g_app_param.is_speed_ring_start )                  //速度闭环未开始
                {
                    g_FOC_Input.theta = g_app_param.curr_theta;
                    g_FOC_Input.Iq_ref = g_app_param.curr_uq;

//速度稳定后切入到速度环
#if 1
                    if( (g_FOC_Output.EKF[2] > 60.0f) || (g_FOC_Output.EKF[2] < -60.0f) )    //检测速度是否达标速度闭环
                    {
                        vf_start_cnt++;
                        if(vf_start_cnt > 40000)                       //速度环达标超4S后，转到速度闭环
                        {
                            vf_start_cnt = 0;
                            g_app_param.is_speed_ring_start = true;

                            TIMER_START(m_speed_pid_timer, 1);          //1K的执行频率
                        }
                    }
                    else
                    {
                        vf_start_cnt = 0;
                    }
#endif

                }
                else
                {
                    g_Speed_Fdk         = g_FOC_Output.EKF[2];          //使用卡尔曼估算的角速度
                    g_FOC_Input.theta   = g_FOC_Output.EKF[3];          //使用卡尔曼估算角度
                    g_FOC_Input.Iq_ref  = g_Speed_Pid_Out;              //使用速度环的输出值作为目标Iq
                }
                

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
 * 电机if运行
 */
void motor_if_run(void)
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
                    g_IF_start_def.IF_abs_time++;
                    IF_start_Algorithm(&g_FOC_Input.Iq_ref, &g_FOC_Input.theta, &g_IF_start_def);
                    g_Speed_Pid.I_Sum = g_app_param.curr_iq;;


                    if(g_FOC_Output.EKF[2] > SPEED_LOOP_CLOSE_RAD_S)    //检测速度是否达标速度闭环
                    {
                        g_app_param.is_speed_ring_start = true;
                    }
                }
                else                                                    //开始速度闭环
                {
                    g_FOC_Input.theta   = g_FOC_Output.EKF[3];          //使用卡尔曼估算角度
                    g_Speed_Fdk         = g_FOC_Output.EKF[2];          //使用卡尔曼估算的角速度
                    g_FOC_Input.Iq_ref  = g_Speed_Pid_Out;              //使用速度环的输出值作为目标Iq
                }

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
#if 0
            justfloat_update(g_FOC_Input.Iq_ref,  0);
            justfloat_update(g_FOC_Output.EKF[3], 0);   //卡尔曼估算角度
            justfloat_update(g_FOC_Output.EKF[2], 0);   //卡尔曼估算速度
            justfloat_update(PLL_def.theta, 0);         //SMO估算角度
            justfloat_update(PLL_def.we,    0);         //SMO角速度

//          justfloat_update(adc_sample_physical_value_get(ADC_CH_U_I), 0);
//	        justfloat_update(adc_sample_physical_value_get(ADC_CH_V_I), 0);
	        justfloat_update(adc_sample_physical_value_get(ADC_CH_W_I), 1);
#endif

//VF 运行显示
#if 1
            //justfloat_update(g_app_param.curr_uq,  0);
            //justfloat_update(g_app_param.target_uq,  0);
            justfloat_update(g_FOC_Output.EKF[3], 0);   //卡尔曼估算角度
            justfloat_update(g_FOC_Output.EKF[2], 0);   //卡尔曼估算速度
            justfloat_update(PLL_def.theta, 0);         //SMO估算角度
            justfloat_update(PLL_def.we,    0);         //SMO角速度
            justfloat_update(g_app_param.curr_theta,  1);
#endif

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

/**
 * 速度环回调函数
 */
static void speed_pid_timer_handler(void *p_data)
{
    //速度环执行
    Speed_Pid_Calc(g_Speed_Ref, g_Speed_Fdk, &g_Speed_Pid_Out, &g_Speed_Pid);
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
                        g_app_param.motor_sta = MOTOR_STA_STOPPING;
                        trace_debug("motor stop\r\n");
                    }
                    else if(usart1_rx_data.data.udata == 0x3F800000)    //开机控件
                    {
                        g_app_param.motor_sta   = MOTOR_STA_STARTING;
                        g_app_param.iq_acc_dir  = ACC_START;

                        g_Speed_Ref = g_app_param.motor_speed_set;
                        trace_debug("motor start\r\n");
                    }
                    break;

                case CMD_TARGET_SPEED:
                        trace_debug("target speed %.4f\r\n", usart1_rx_data.data.fdate);

                        g_app_param.motor_speed_set = (uint16_t)usart1_rx_data.data.fdate;

                        g_Speed_Ref = usart1_rx_data.data.fdate;
                    break;

                case CMD_TARGET_IQ:
                        trace_debug("target Iq %.4f\r\n", usart1_rx_data.data.fdate);

                        g_app_param.target_iq = usart1_rx_data.data.fdate;
                        if((g_app_param.target_iq > 6.0f) || (g_app_param.target_iq < -6.0f))
                        {
                            g_app_param.target_iq = 0.0f;
                        }
                    break;

                case CMD_TARGET_UQ:
                        trace_debug("target Uq %.4f\r\n", usart1_rx_data.data.fdate);

                        g_app_param.target_uq  = usart1_rx_data.data.fdate;
                        g_app_param.iq_acc_dir = ACC_START;
                    break;

                case CMD_TARGET_STEP_ANGLE:
                        trace_debug("target step angle %.4f\r\n", usart1_rx_data.data.fdate);

                        g_app_param.target_step_angle = usart1_rx_data.data.fdate;
                    break;

                case CMD_DIR:
                    if(usart1_rx_data.data.udata == 0x0)                //控件数据
                    {
                        g_app_param.motor_dir = MOTOR_DIR_CW;
                        trace_debug("dir cw\r\n");
                    }
                    else if(usart1_rx_data.data.udata == 0x3F800000)    //控件数据
                    {
                        g_app_param.motor_dir = MOTOR_DIR_CCW;
                        trace_debug("dir ccw\r\n");
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
                g_app_param.curr_uq = 0.0f;
                g_app_param.curr_theta = 0.0f;
                g_app_param.iq_acc_dir = ACC_DONE;

                g_app_param.is_param_init_done = false;
            }

            TIMER_STOP(m_speed_pid_timer);
            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 0);

            g_app_param.motor_sta = MOTOR_STA_STOP;
            break;

        case MOTOR_STA_RUNNING:
            gpio_output_set(PWM_EN_PORT, PWM_EN_PIN, 1);
            break;

        case MOTOR_STA_STARTING:
            if(g_app_param.motor_sta != g_app_param.pre_motor_sta)  //每一次启动都要foc参数初始化
            {
                IF_Start_Init();                //IF启动参数初始化

                foc_algorithm_initialize();     //FOC 算法参数初始化

                g_app_param.is_param_init_done = true;

                phase_pwm_start();
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



