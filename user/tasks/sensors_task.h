#ifndef SENSORS_TASK_H__
#define SENSORS_TASK_H__

uint16_t adc_sample_data_get(adc_channel_e ch);
float adc_sample_physical_value_get(adc_channel_e ch);

int sensors_task(void);

#endif
