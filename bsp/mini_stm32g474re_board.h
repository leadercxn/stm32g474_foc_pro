#ifndef BOARD_MINI_STM32G474RE_H__
#define BOARD_MINI_STM32G474RE_H__

#include "stdint.h"
#include "stdbool.h"
#include "stm32g4xx_hal.h"

#define MACHINE_NAME                "MINI_STM32G474RE"

#define USART1_TX_PORT              GPIOA
#define USART1_TX_PIN               GPIO_PIN_9
#define USART1_TX_AF                GPIO_AF7_USART1

#define USART1_RX_PORT              GPIOA
#define USART1_RX_PIN               GPIO_PIN_10
#define USART1_RX_AF                GPIO_AF7_USART1

#define USART1_BAUDRATE             115200

#define LED_STAT_PORT               GPIOC
#define LED_STAT_PIN                GPIO_PIN_13

#define TEST_IO_PORT                GPIOC
#define TEST_IO_PIN                 GPIO_PIN_1

#endif

