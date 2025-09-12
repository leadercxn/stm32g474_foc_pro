#include "gpio.h"

#include "boards.h"


int bsp_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = { 0 };

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio_init_struct.Pin     = LED_STAT_PIN | TEST0_IO_PIN | TEST1_IO_PIN | TEST2_IO_PIN ;
    gpio_init_struct.Mode    = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull    = GPIO_NOPULL;
    gpio_init_struct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_STAT_PORT, &gpio_init_struct);

    gpio_init_struct.Pin     = PWM_EN_PIN;
    HAL_GPIO_Init(PWM_EN_PORT, &gpio_init_struct);

    return 0;
}

void gpio_output_set(uint32_t gpio_port, uint32_t gpio_pin, uint8_t value)
{
    if(value)
    {
        HAL_GPIO_WritePin((GPIO_TypeDef *)gpio_port, gpio_pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin((GPIO_TypeDef *)gpio_port, gpio_pin, GPIO_PIN_RESET);
    }
}

int gpio_input_get(uint32_t gpio_port, uint32_t gpio_pin)
{
    
    return 0;
}

