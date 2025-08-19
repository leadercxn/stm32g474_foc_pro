#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#include "boards.h"
#include "sys.h"
#include "delay.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "adc.h"
#include "parameters.h"
#include "trace.h"

int main(void)
{
  bool led_stat = false;
  uint32_t test_inter_ticks = 0;

  HAL_Init();
//sys_stm32_clock_init(85, 2, 2, 4, 8);       /* 设置时钟,170Mhz  正点原子*/
  sys_stm32_clock_init(85, 3, 2, 2, 2);       /* 设置时钟,170Mhz */
  delay_init(170);                            /* 延时初始化 */

  //外设初始化
  bsp_gpio_init();
  usart1_init();

  trace_info("STM32G474 Pro Start \r\n\r\n\r\n")

  timer8_init();    //用于生成PWM
  adc_init();

  phase_pwm_start();
  phase_pwm_set( (PWM_PERIOD / 3), (PWM_PERIOD / 4), (PWM_PERIOD / 5));

  adc_start();

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

      trace_debug("sys time ms %lu\r\n", sys_time_ms_get());

      trace_debug("adc 1-%d, 2-%d, 3-%d, 4-%d, 5-%d, 6-%d, 7-%d, 8-%d\r\n", \
      adc_sample_data_get(ADC_CH_U_VOLT), adc_sample_data_get(ADC_CH_V_VOLT), adc_sample_data_get(ADC_CH_W_VOLT), \
      adc_sample_data_get(ADC_CH_U_I), adc_sample_data_get(ADC_CH_V_I), adc_sample_data_get(ADC_CH_W_I), \
      adc_sample_data_get(ADC_CH_VBUS), adc_sample_data_get(ADC_CH_TEMP));

      if(sys_time_ms_get() > 6000)
      {
        phase_pwm_stop();
      }
    }
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
