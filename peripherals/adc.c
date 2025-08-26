#include "string.h"
#include "boards.h"
#include "adc.h"
#include "math.h"
#include "parameters.h"

#include "trace.h"

extern void Error_Handler(void);

static ADC_HandleTypeDef m_adc2_handle;
static DMA_HandleTypeDef m_dma_adc2_handle;

/**
 * adc2初始化函数
 */
int adc_init(void)
{
    ADC_ChannelConfTypeDef      channle_cfg = {0};

    m_adc2_handle.Instance                      = ADC2;
    m_adc2_handle.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    m_adc2_handle.Init.Resolution               = ADC_RESOLUTION_12B;
    m_adc2_handle.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
    m_adc2_handle.Init.GainCompensation         = 0;
    m_adc2_handle.Init.ScanConvMode             = ADC_SCAN_ENABLE;
    m_adc2_handle.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    m_adc2_handle.Init.LowPowerAutoWait         = DISABLE;
    m_adc2_handle.Init.ContinuousConvMode       = ENABLE;
    m_adc2_handle.Init.NbrOfConversion          = 8;
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

#if 0
    ADC_MultiModeTypeDef        multimode = {0};

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&m_adc2_handle, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    trace_debug("multimode init done\r\n");
#endif

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

    channle_cfg.Channel = ADC_CHANNEL_11;
    channle_cfg.Rank    = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&m_adc2_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    channle_cfg.Channel = ADC_CHANNEL_12;
    channle_cfg.Rank    = ADC_REGULAR_RANK_5;
    if (HAL_ADC_ConfigChannel(&m_adc2_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    channle_cfg.Channel = ADC_CHANNEL_13;
    channle_cfg.Rank    = ADC_REGULAR_RANK_6;
    if (HAL_ADC_ConfigChannel(&m_adc2_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    channle_cfg.Channel = ADC_CHANNEL_15;
    channle_cfg.Rank    = ADC_REGULAR_RANK_7;
    if (HAL_ADC_ConfigChannel(&m_adc2_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    channle_cfg.Channel = ADC_CHANNEL_17;
    channle_cfg.Rank    = ADC_REGULAR_RANK_8;
    if (HAL_ADC_ConfigChannel(&m_adc2_handle, &channle_cfg) != HAL_OK)
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

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);                        /* 设置DMA中断优先级为2，子优先级为0 */
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

void adc_start(uint32_t *p_dma_adc_rx_data)
{
    static bool calibration_done = false;
    int err_code = 0;
    
    if(calibration_done == false)
    {
        err_code = HAL_ADCEx_Calibration_Start(&m_adc2_handle, ADC_SINGLE_ENDED);   //ADC 启动前自校准
        if(err_code == 0)
        {
            calibration_done = true;
        }
    }

    HAL_ADC_Start_DMA(&m_adc2_handle, p_dma_adc_rx_data, ADC_TOTAL_COLLECT_NUM);      //用于ADC1规则通道DMA传输
}

void adc_stop(void)
{
    HAL_ADC_Stop_DMA(&m_adc2_handle);      //停止DMA传输
}


