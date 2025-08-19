#ifndef ADC_H__
#define ADC_H__

typedef enum
{
    ADC_CH_U_VOLT,
    ADC_CH_V_VOLT,
    ADC_CH_W_VOLT,
    ADC_CH_U_I,
    ADC_CH_V_I,
    ADC_CH_W_I,
    ADC_CH_VBUS,
    ADC_CH_TEMP,
} adc_channel_e;

int  adc_init(void);
void adc_start(void);
uint16_t adc_sample_data_get(adc_channel_e ch);

#endif