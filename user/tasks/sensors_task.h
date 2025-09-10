#ifndef SENSORS_TASK_H__
#define SENSORS_TASK_H__

uint16_t adc_reg_sample_data_get(uint8_t ch);
uint16_t adc_inj_sample_data_get(uint8_t ch);
float adc_sample_physical_value_get(adc_channel_e ch);

int sensors_task(void);

#endif
