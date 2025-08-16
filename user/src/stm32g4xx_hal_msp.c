#include "main.h"

#include "boards.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_PWREx_DisableUCPDDeadBattery();
}

/**
  * @brief 定时器恢复默认状态
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }

}

/**
  * @brief 定时器恢复默认状态
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  }

}


/**
 * 串口硬件IO初始化，内部调用
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef gpio_init_struct = {0};

  if(huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio_init_struct.Pin        = USART1_TX_PIN | USART1_RX_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull       = GPIO_NOPULL;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_LOW;
    gpio_init_struct.Alternate  = USART1_TX_AF;
  
    HAL_GPIO_Init(USART1_TX_PORT, &gpio_init_struct);

    HAL_NVIC_EnableIRQ(USART1_IRQn);                      /* 使能USART1中断通道 */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);              /* 抢占优先级3，子优先级3 */
  }

}

/**
 * 串口硬件IO恢复默认值，内部调用
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();

    HAL_GPIO_DeInit(USART1_TX_PORT, USART1_TX_PIN | USART1_RX_PIN);
  }

}

