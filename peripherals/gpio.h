#ifndef GPIO_H__
#define GPIO_H__


#include "boards.h"

int bsp_gpio_init(void);

void gpio_output_set(uint32_t gpio_port, uint32_t gpio_pin, uint8_t value);
int  gpio_input_get(uint32_t gpio_port, uint32_t gpio_pin);

#endif
