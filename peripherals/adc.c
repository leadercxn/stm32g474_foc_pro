#include "string.h"
#include "boards.h"
#include "adc.h"
#include "math.h"
#include "parameters.h"

#include "trace.h"

extern void Error_Handler(void);

ADC_HandleTypeDef m_adc2_handle;
static DMA_HandleTypeDef m_dma_adc2_handle;

static bool m_calibration_done = false;
/**
 * adc2初始化函数
 */
int adc_init(void)
{
    ADC_ChannelConfTypeDef      channle_cfg = {0};  //规则通道

    m_adc2_handle.Instance                      = ADC2;
    m_adc2_handle.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    m_adc2_handle.Init.Resolution               = ADC_RESOLUTION_12B;
    m_adc2_handle.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
    m_adc2_handle.Init.GainCompensation         = 0;
    m_adc2_handle.Init.ScanConvMode             = ADC_SCAN_ENABLE;
    m_adc2_handle.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    m_adc2_handle.Init.LowPowerAutoWait         = DISABLE;
    m_adc2_handle.Init.ContinuousConvMode       = ENABLE;
    m_adc2_handle.Init.NbrOfConversion          = ADC_REG_CHAN_NUM;
    m_adc2_handle.Init.DiscontinuousConvMode    = DISABLE;
    m_adc2_handle.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    m_adc2_handle.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    m_adc2_handle.Init.DMAContinuousRequests    = ENABLE;
    m_adc2_handle.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    m_adc2_handle.Init.OversamplingMode         = DISABLE;

    if (HAL_ADC_Init(&m_adc2_handle) != HAL_OK)
    {
        Error_Handler();
    }

    //规则通道
    channle_cfg.Channel      = ADC_CHANNEL_3;
    channle_cfg.Rank         = ADC_REGULAR_RANK_1;
    channle_cfg.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    channle_cfg.SingleDiff   = ADC_SINGLE_ENDED;
    channle_cfg.OffsetNumber = ADC_OFFSET_NONE;
    channle_cfg.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&m_adc2_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    channle_cfg.Channel = ADC_CHANNEL_4;
    channle_cfg.Rank    = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&m_adc2_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    channle_cfg.Channel = ADC_CHANNEL_5;
    channle_cfg.Rank    = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&m_adc2_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    channle_cfg.Channel = ADC_CHANNEL_15;
    channle_cfg.Rank    = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&m_adc2_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    channle_cfg.Channel = ADC_CHANNEL_17;
    channle_cfg.Rank    = ADC_REGULAR_RANK_5;
    if (HAL_ADC_ConfigChannel(&m_adc2_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    // 配置注入通道
    ADC_InjectionConfTypeDef injection_ch_cfg = {0};

    injection_ch_cfg.InjectedChannel                = ADC_CHANNEL_11;
    injection_ch_cfg.InjectedRank                   = ADC_INJECTED_RANK_1;
    injection_ch_cfg.InjectedSamplingTime           = ADC_SAMPLETIME_2CYCLES_5;
    injection_ch_cfg.InjectedSingleDiff             = ADC_SINGLE_ENDED;
    injection_ch_cfg.InjectedOffsetNumber           = ADC_OFFSET_NONE;
    injection_ch_cfg.InjectedOffset                 = 0;
    injection_ch_cfg.InjectedNbrOfConversion        = ADC_INJ_CHAN_NUM;
    injection_ch_cfg.InjectedDiscontinuousConvMode  = DISABLE;
    injection_ch_cfg.AutoInjectedConv               = DISABLE;
    injection_ch_cfg.QueueInjectedContext           = DISABLE;
    injection_ch_cfg.ExternalTrigInjecConv          = ADC_INJECTED_SOFTWARE_START;
    injection_ch_cfg.ExternalTrigInjecConvEdge      = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
//    injection_ch_cfg.ExternalTrigInjecConv          = ADC_EXTERNALTRIGINJEC_T8_CC4;          // 不知为何，T8 CH4总是触发不了ADC 注入通道的启动
//    injection_ch_cfg.ExternalTrigInjecConvEdge      = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    injection_ch_cfg.InjecOversamplingMode          = DISABLE;
    if (HAL_ADCEx_InjectedConfigChannel(&m_adc2_handle, &injection_ch_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    injection_ch_cfg.InjectedChannel  = ADC_CHANNEL_12;
    injection_ch_cfg.InjectedRank     = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&m_adc2_handle, &injection_ch_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    injection_ch_cfg.InjectedChannel  = ADC_CHANNEL_13;
    injection_ch_cfg.InjectedRank     = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&m_adc2_handle, &injection_ch_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    return 0;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  GPIO_InitTypeDef          gpio_init_struct = {0};
  RCC_PeriphCLKInitTypeDef  periph_clk_init = {0};
  if(adcHandle->Instance == ADC2)
  {
    periph_clk_init.PeriphClockSelection    = RCC_PERIPHCLK_ADC12;
    periph_clk_init.Adc12ClockSelection     = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio_init_struct.Pin  = ADC_U_BEMF_PIN | ADC_V_BEMF_PIN | ADC_W_I_PIN | ADC_TEMP_PIN;
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    gpio_init_struct.Pin = ADC_V_I_PIN | ADC_VBUS_PIN;
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    gpio_init_struct.Pin = ADC_W_BEMF_PIN | ADC_U_I_PIN;
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);

    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* ADC2 DMA Init */
    m_dma_adc2_handle.Instance                  = DMA1_Channel1;
    m_dma_adc2_handle.Init.Request              = DMA_REQUEST_ADC2;
    m_dma_adc2_handle.Init.Direction            = DMA_PERIPH_TO_MEMORY;
    m_dma_adc2_handle.Init.PeriphInc            = DMA_PINC_DISABLE;
    m_dma_adc2_handle.Init.MemInc               = DMA_MINC_ENABLE;
    m_dma_adc2_handle.Init.PeriphDataAlignment  = DMA_PDATAALIGN_HALFWORD;
    m_dma_adc2_handle.Init.MemDataAlignment     = DMA_MDATAALIGN_HALFWORD;
    m_dma_adc2_handle.Init.Mode                 = DMA_CIRCULAR;
    m_dma_adc2_handle.Init.Priority             = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&m_dma_adc2_handle) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle, DMA_Handle, m_dma_adc2_handle);

    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 1);                        /* 设置DMA中断优先级为2，子优先级为0 */
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);    
  }
}

/**
 * @brief       ADC DMA采集中断服务函数
 * @param       无
 * @retval      无
 */
void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&m_dma_adc2_handle);
}

void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&m_adc2_handle);
}

void adc_reg_start(uint32_t *p_dma_adc_rx_data)
{
    int err_code = 0;
    
    if(m_calibration_done == false)
    {
        err_code = HAL_ADCEx_Calibration_Start(&m_adc2_handle, ADC_SINGLE_ENDED);   //ADC 启动前自校准
        if(err_code == 0)
        {
            m_calibration_done = true;
        }
    }

    HAL_ADC_Start_DMA(&m_adc2_handle, p_dma_adc_rx_data, ADC_TOTAL_COLLECT_NUM);      //用于ADC1规则通道DMA传输
}

void adc_reg_stop(void)
{
    HAL_ADC_Stop_DMA(&m_adc2_handle);      //停止DMA传输
}

void adc_inj_start(void)
{
    int err_code = 0;
    
    if(m_calibration_done == false)
    {
        err_code = HAL_ADCEx_Calibration_Start(&m_adc2_handle, ADC_SINGLE_ENDED);   //ADC 启动前自校准
        if(err_code == 0)
        {
            m_calibration_done = true;
        }
    }

    HAL_ADCEx_InjectedStart_IT(&m_adc2_handle);//启动ADC1注入通道转换
}

void adc_inj_stop(void)
{
    HAL_ADCEx_InjectedStop_IT(&m_adc2_handle);
}