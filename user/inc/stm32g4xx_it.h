#ifndef __STM32G4xx_IT_H
#define __STM32G4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"

typedef void (*timer_handler_t) (void);

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void systick_timer_handler_register(timer_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif /* __STM32G4xx_IT_H */
