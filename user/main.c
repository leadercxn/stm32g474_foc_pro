#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#include "boards.h"
#include "sys.h"
#include "delay.h"
#include "uart.h"
#include "gpio.h"
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
