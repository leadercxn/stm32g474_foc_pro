#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"

#include "boards.h"
#include "sys.h"
#include "delay.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "adc.h"
#include "parameters.h"
#include "trace.h"
#include "app_timer.h"
#include "mid_timer.h"

#include "sensors_task.h"
#include "motor_ctrl_task.h"
#include "mb_slaver_task.h"
#include "monitor_task.h"

#include "foc.h"
//#include "ekf.h"

static void param_init(void)
{
    g_foc_input.tpwm  = PWM_TIM_PULSE_TPWM;

    g_mb_ctrl_param.speed_pid_p     = SPEED_PI_P;
    g_mb_ctrl_param.speed_pid_i     = SPEED_PI_I;
    g_mb_ctrl_param.speed_pid_kb    = SPEED_PI_KB;
    g_mb_ctrl_param.speed_pid_limit = SPEED_PI_UP_LIMIT;

    g_mb_ctrl_param.i_pid_p      = Q_PI_P;
    g_mb_ctrl_param.i_pid_i      = Q_PI_I;
    g_mb_ctrl_param.i_pid_kb     = Q_PI_KB;
    g_mb_ctrl_param.i_pid_limit  = Q_PI_UP_LIMIT;

    g_mb_ctrl_param.phase_rs     = MOTOR_PHASE_RES;   //相电阻
    g_mb_ctrl_param.phase_ls     = MOTOR_PHASE_LS;    //相电感
    g_mb_ctrl_param.flux_link    = MOTOR_FLUXLINK;    //磁链

    g_mb_ctrl_param.speed_max    = MOTOR_SPEED_MAX_RPM;
    g_mb_ctrl_param.speed_min    = MOTOR_SPEED_MIN_RPM;

    g_mb_ctrl_param.i_err_th          = 10.0f;            //过流阈值
    g_mb_ctrl_param.v_err_th          = VBUS_VLOT * 1.2f; //过压阈值

    g_mb_ctrl_param.pll_p = 600.0f;
    g_mb_ctrl_param.pll_i = 1500.0f;

    g_mb_ctrl_param.motor_pole_pairs  = MOTOR_POLE_PAIRS;

    foc_algorithm_init();
}

/**
 * 测试sinf, arm_sin计算的速度
 */
void sin_cal_speed_compare(void)
{
    trace_debug("sinf %.4f, arm_sin f %.4f, cosf %.4f, arm_cos f %.4f\r\n", \
    sinf(2.22f), arm_sin_f32(2.22f), cosf(1.56f), arm_cos_f32(1.56f));

    float temp = 0;
    uint32_t ticks = 0;
    uint32_t ticks_delta = 0;

    ticks = sys_time_ms_get();
    for(uint32_t i = 0; i < 100000; i++)
    {
      temp = sinf(2.22f);     //对比好像这个运行更快
    }
    ticks_delta = sys_time_ms_get() - ticks;
    trace_debug("sinf %lu\r\n", ticks_delta);

    ticks = sys_time_ms_get();
    for(uint32_t i = 0; i < 100000; i++)
    {
      temp = arm_sin_f32(2.22f);
    }
    ticks_delta = sys_time_ms_get() - ticks;
    trace_debug("arm_sin_f32 %lu\r\n", ticks_delta);
}

int main(void)
{
  bool led_stat = false;
  uint32_t test_inter_ticks = 0;

  float angle = PI * 0.2f;

#if 0
  volt_dq_t           t_vdq;
  transf_cos_sin_t    t_cos_sin;
  float               t_theta = PI / 5;
  volt_alpha_beta_t   t_v_alpha_beta;

  t_vdq.Vd = 0;
  t_vdq.Vq = 3.0f;
#endif

  HAL_Init();
//sys_stm32_clock_init(85, 2, 2, 4, 8);       /* 设置时钟,170Mhz  正点原子*/
  sys_stm32_clock_init(85, 3, 2, 2, 2);       /* 设置时钟,170Mhz */
  delay_init(170);                            /* 延时初始化 */

  //外设初始化
  bsp_gpio_init();  //普通型IO初始化
  usart1_init();    //usart1 初始化, 用于串口打印调试信息
  usart3_init();    //usart3 初始化, 用于 modbus 数据交互

  timer8_init();    //用于生成PWM，
  adc_init();       //adc2 用于采样电流、电压、温度

  TIMER_INIT();     // 调度定时器初始化，用于简单的ms级定时器调度

  trace_info("\r\n STM32G474 FOC Test Start \r\n\r\n")

  phase_pwm_start();

  param_init();     //参数初始化
//  apt_ekf_init();

  while (1)
  {
      if(sys_time_ms_get() - test_inter_ticks >= 1000)
      {
        test_inter_ticks = sys_time_ms_get();

        if(led_stat)
        {
            led_stat = false;
        }
        else
        {
            led_stat = true;
        }

        gpio_output_set(LED_STAT_PORT, LED_STAT_PIN, led_stat);
        
#if 0
        angle_to_cos_sin(t_theta, &t_cos_sin);
        rev_park_transf(t_vdq, t_cos_sin, &t_v_alpha_beta);
        svpwm_calc(t_v_alpha_beta, 24.0, g_foc_input.tpwm);

        TIM8->CCR1 = (uint16_t)(g_foc_output.tcmp1);     
	      TIM8->CCR2 = (uint16_t)(g_foc_output.tcmp2);
	      TIM8->CCR3 = (uint16_t)(g_foc_output.tcmp3);

        t_theta += PI / 3;

        t_theta = radian_normalize(t_theta);

        trace_debug("t_theta %.3f, alpha %.3f, beta %.3f, CCR1 %lu, CCR2 %lu, CCR3 %lu\r\n", \
          t_theta, t_v_alpha_beta.Valpha, t_v_alpha_beta.Vbeta, TIM8->CCR1, TIM8->CCR2, TIM8->CCR3);
#endif

//        sin_cal_speed_compare();

#if 0
        trace_debug("UV %.2fV, VV %.2fV, WV %.2fV, UI %.4fA, VI %.4fA, WI %.4fA, VBUS %.1fV, T %.1fC \r\n", \
        adc_sample_physical_value_get(ADC_CH_U_VOLT), adc_sample_physical_value_get(ADC_CH_V_VOLT), \
        adc_sample_physical_value_get(ADC_CH_W_VOLT), adc_sample_physical_value_get(ADC_CH_U_I), \
        adc_sample_physical_value_get(ADC_CH_V_I), adc_sample_physical_value_get(ADC_CH_W_I),   \
        adc_sample_physical_value_get(ADC_CH_VBUS), adc_sample_physical_value_get(ADC_CH_TEMP));

#endif

// 测试svpwm
#if 0
        angle += PI / 3;

        angle = radian_normalize(angle);

        torque_set(2.0f, 0, angle);

        trace_debug("angle %f\r\n", angle);
#endif

        trace_debug("evt code %#llx\r\n", g_app_param.evt_code);
      }

      sensors_task();         //传感器任务

      motor_ctrl_task();      //电机控制任务

      mb_slaver_task();       //modbus 从机任务

      monitor_task();         //监控任务

      mid_timer_loop_task();  //调度定时器的循环执行
  }
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
