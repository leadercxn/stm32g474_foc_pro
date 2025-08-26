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

#define PWM_UH_PORT                 GPIOC
#define PWM_UH_PIN                  GPIO_PIN_6  //UH
#define PWM_UL_PORT                 GPIOC
#define PWM_UL_PIN                  GPIO_PIN_10 //UL

#define PWM_VH_PORT                 GPIOC
#define PWM_VH_PIN                  GPIO_PIN_7  //VH
#define PWM_VL_PORT                 GPIOC
#define PWM_VL_PIN                  GPIO_PIN_11 //VL

#define PWM_WH_PORT                 GPIOC
#define PWM_WH_PIN                  GPIO_PIN_8  //WH
#define PWM_WL_PORT                 GPIOC
#define PWM_WL_PIN                  GPIO_PIN_12 //WL

#define PWM_CH4_PORT                GPIOC
#define PWM_CH4_PIN                 GPIO_PIN_9

#define ADC_U_BEMF_PORT             GPIOA
#define ADC_U_BEMF_PIN              GPIO_PIN_6  //adc2-IN3
#define ADC_V_BEMF_PORT             GPIOA
#define ADC_V_BEMF_PIN              GPIO_PIN_7  //adc2-IN4
#define ADC_W_BEMF_PORT             GPIOC
#define ADC_W_BEMF_PIN              GPIO_PIN_4  //adc2-IN5
#define ADC_U_I_PORT                GPIOC
#define ADC_U_I_PIN                 GPIO_PIN_5  //adc2-IN11
#define ADC_V_I_PORT                GPIOB
#define ADC_V_I_PIN                 GPIO_PIN_2  //adc2-IN12
#define ADC_W_I_PORT                GPIOA
#define ADC_W_I_PIN                 GPIO_PIN_5  //adc2-IN13

#define ADC_VBUS_PORT               GPIOB
#define ADC_VBUS_PIN                GPIO_PIN_15 //adc2-IN15
#define ADC_TEMP_PORT               GPIOA
#define ADC_TEMP_PIN                GPIO_PIN_4  //adc2-IN17

#define PWM_EN_PORT                 GPIOA
#define PWM_EN_PIN                  GPIO_PIN_3

#endif

