#ifndef ADC_H__
#define ADC_H__

#define ADC_REG_CHAN_NUM        5       // ADC规则通道数
#define ADC_SAMPLE_NUM          10      // ADC采样扫描次数
#define ADC_TOTAL_COLLECT_NUM   (ADC_REG_CHAN_NUM * ADC_SAMPLE_NUM)

#define ADC_INJ_CHAN_NUM        3       // 注入通道数量

extern ADC_HandleTypeDef m_adc2_handle;

typedef enum
{
    // 规则通道
    ADC_CH_U_VOLT,
    ADC_CH_V_VOLT,
    ADC_CH_W_VOLT,
    ADC_CH_VBUS,
    ADC_CH_TEMP,

    // 注入通道
    ADC_CH_U_I,
    ADC_CH_V_I,
    ADC_CH_W_I,
} adc_channel_e;

int  adc_init(void);

void adc_reg_start(uint32_t *p_dma_adc_rx_data);
void adc_reg_stop(void);
void adc_inj_start(void);
void adc_inj_stop(void);

#endif