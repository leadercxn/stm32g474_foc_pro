#ifndef TIMER_H__
#define TIMER_H__

extern uint64_t g_tim8_50us_ticks;

typedef void (*timer8_irq_cb_t)(void);

int timer8_init(void);

void phase_pwm_set(uint32_t u, uint32_t v, uint32_t w);
void phase_pwm_start(void);
void phase_pwm_stop(void);

void timer8_irq_cb_register(timer8_irq_cb_t cb);

/**
 * 六步换相API
 * 
 */
void motor_stop(void);
void motor_start(void);

void motor_uhvl(void);
void motor_uhwl(void);
void motor_vhwl(void);
void motor_vhul(void);
void motor_whul(void);
void motor_whvl(void);


#endif


