#ifndef TIMER_H__
#define TIMER_H__

int timer8_init(void);

void phase_pwm_set(uint32_t u, uint32_t v, uint32_t w);
void phase_pwm_start(void);
void phase_pwm_stop(void);
#endif


