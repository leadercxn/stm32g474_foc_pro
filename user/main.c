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

#include "foc.h"
#include "ekf.h"

static void param_init(void)
{
    g_app_param.foc_ts    = (float) (1.0f / PWM_FREQ);
    g_app_param.smo_f     = expf( (- MOTOR_PHASE_RES / MOTOR_PHASE_LS) * g_app_param.foc_ts);
    g_app_param.smo_g     = (1.0f - g_app_param.smo_f) / MOTOR_PHASE_RES;
    g_app_param.smo_g    *= (MOTOR_RATED_V * SQRT_3_DIV_3 / MOTOR_RATED_I);   // 额定电压除以额定电流，得到阻抗值, 参考 硕历达的程序

    g_app_param.smo_k_slide   = 0.168f;
    g_app_param.smo_k_slf     = 0.058;
    g_app_param.smo_i_err_max = 0.5f;

    g_app_param.vel_coeff = (float) ( 1.0f / (g_app_param.foc_ts * VEL_LOOP_EXEC_FREQ) * 60.0f / (MOTOR_POLE_PAIRS * DOUBLE_PI) ); // 速度系数，RPM 转换为 电角度/50us

    g_app_param.ekf_theta = 0.0f;

    trace_debug("foc ts %.5f, smo f %.5f, smo g %.5f, vel_coeff %.4f\r\n", \
    g_app_param.foc_ts, g_app_param.smo_f, g_app_param.smo_g, g_app_param.vel_coeff);

    g_smo_pll.ts_dt = g_app_param.foc_ts;

    g_iq_pi.kp       = 1.2f;
    g_iq_pi.ki       = 0.3f;
    g_iq_pi.out_max  = 4.0f;
    g_iq_pi.iout_max = 10.0f;

    g_id_pi.kp       = 0.6f;
    g_id_pi.ki       = 0.1f;
    g_id_pi.out_max  = 4.0f;
    g_id_pi.iout_max = 10.0f;

}

int main(void)
{
  bool led_stat = false;
  uint32_t test_inter_ticks = 0;

  float angle = PI * 0.2f;

  HAL_Init();
//sys_stm32_clock_init(85, 2, 2, 4, 8);       /* 设置时钟,170Mhz  正点原子*/
  sys_stm32_clock_init(85, 3, 2, 2, 2);       /* 设置时钟,170Mhz */
  delay_init(170);                            /* 延时初始化 */

  //外设初始化
  bsp_gpio_init();  //普通型IO初始化
  usart1_init();    //usart1 初始化, 用于 modbus 数据交互

  timer8_init();    //用于生成PWM，
  adc_init();       //adc2 用于采样电流、电压、温度

  TIMER_INIT();     // 调度定时器初始化，用于简单的ms级定时器调度

  trace_info("STM32G474 FOC Test Start \r\n\r\n")

  param_init();     //参数初始化
  apt_ekf_init();

  phase_pwm_start();

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
        gpio_output_set(TEST_IO_PORT, TEST_IO_PIN, led_stat);

#if 0
        trace_debug("sys time ms %lu\r\n", sys_time_ms_get());

        trace_debug("adc 1-%d, 2-%d, 3-%d, 4-%d, 5-%d, 6-%d, 7-%d, 8-%d\r\n", \
        adc_sample_data_get(ADC_CH_U_VOLT), adc_sample_data_get(ADC_CH_V_VOLT), adc_sample_data_get(ADC_CH_W_VOLT), \
        adc_sample_data_get(ADC_CH_U_I), adc_sample_data_get(ADC_CH_V_I), adc_sample_data_get(ADC_CH_W_I), \
        adc_sample_data_get(ADC_CH_VBUS), adc_sample_data_get(ADC_CH_TEMP));
#endif

#if 0
        trace_debug("UV %.2fV, VV %.2fV, WV %.2fV, UI %.4fA, VI %.4fA, WI %.4fA, VBUS %.1fV, T %.1fC \r\n", \
        adc_sample_physical_value_get(ADC_CH_U_VOLT), adc_sample_physical_value_get(ADC_CH_V_VOLT), \
        adc_sample_physical_value_get(ADC_CH_W_VOLT), adc_sample_physical_value_get(ADC_CH_U_I), \
        adc_sample_physical_value_get(ADC_CH_V_I), adc_sample_physical_value_get(ADC_CH_W_I),   \
        adc_sample_physical_value_get(ADC_CH_VBUS), adc_sample_physical_value_get(ADC_CH_TEMP));

        trace_debug("motor real speed = %d\r\n", g_app_param.motor_speed_real);
#endif

// 测试svpwm
#if 0
        angle += PI / 3;

        angle = radian_normalize(angle);

        torque_set(2.0f, 0, angle);

        trace_debug("angle %f\r\n", angle);
#endif
      }

//vofa 打印三相电压，电流
#if 0
      uint32_t vofa_send_ticks = 0;

      if(sys_time_ms_get() - vofa_send_ticks >= 10)
      {
          vofa_send_ticks = sys_time_ms_get();

          VOFA_PRINTF("%.2f, %.2f, %.2f, %.4f, %.4f, %.4f\n", adc_sample_physical_value_get(ADC_CH_U_VOLT), \
                 adc_sample_physical_value_get(ADC_CH_V_VOLT), adc_sample_physical_value_get(ADC_CH_W_VOLT), \
                 adc_sample_physical_value_get(ADC_CH_U_I), adc_sample_physical_value_get(ADC_CH_V_I),     \
                 adc_sample_physical_value_get(ADC_CH_W_I));
      }
#endif

#if 0
      uint64_t old_50us_ticks = 0;

      if(g_tim8_50us_ticks - old_50us_ticks > 1000)   //50 ms定时器任务
      {
          old_50us_ticks = g_tim8_50us_ticks;

          trace_debug("50us ticks %llu, sys ticks %lu\r\n", g_tim8_50us_ticks, sys_time_ms_get());

      }
#endif

      sensors_task();         //传感器任务

      motor_ctrl_task();      //电机控制任务

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
