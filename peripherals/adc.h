#ifndef ADC_H__
#define ADC_H__

#define ADC_CHAN_NUM            8                                   // ADC通道数
#define ADC_SAMPLE_NUM          40                                  // ADC采样扫描次数
#define ADC_TOTAL_COLLECT_NUM   (ADC_CHAN_NUM * ADC_SAMPLE_NUM)

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
void adc_start(uint32_t *p_dma_adc_rx_data);
void adc_stop(void);

#endif