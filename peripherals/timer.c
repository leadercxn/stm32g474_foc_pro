#include "boards.h"
#include "timer.h"
#include "parameters.h"

#include "gpio.h"
#include "trace.h"

extern void Error_Handler(void);

static TIM_HandleTypeDef m_timer8_handle;

uint64_t g_tim8_50us_ticks = 0;  //50us定时器计数 1= 50us

static timer8_irq_cb_t m_timer8_cb_handler = NULL;

static void pwm_io_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = { 0 };

    __HAL_RCC_GPIOC_CLK_ENABLE();

    /** TIM8 GPIO Configuration
     * 
        PC6  ---- TIM8_CH1  AF4 ---- UH
        PC10 ---- TIM8_CH1N AF4 ---- UL
        PC7  ---- TIM8_CH2  AF4 ---- VH
        PC11 ---- TIM8_CH2N AF4 ---- VL
        PC8  ---- TIM8_CH3  AF4 ---- WH
        PC12 ---- TIM8_CH3N AF4 ---- WL
        PC9  ---- TIM8_CH4  AF4
     *
    */

    gpio_init_struct.Pin       = PWM_UH_PIN | PWM_UL_PIN | PWM_VH_PIN | PWM_VL_PIN | PWM_WH_PIN | PWM_WL_PIN | PWM_CH4_PIN;
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull      = GPIO_NOPULL;
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio_init_struct.Alternate = GPIO_AF4_TIM8;

    HAL_GPIO_Init(PWM_UH_PORT, &gpio_init_struct);

}

int timer8_init(void)
{
    TIM_ClockConfigTypeDef          clk_cfg = {0};
    TIM_MasterConfigTypeDef         master_cfg = {0};
    TIM_OC_InitTypeDef              oc_cfg = {0};
    TIM_BreakDeadTimeConfigTypeDef  break_deadtime_cfg = {0};

    //timer base cfg
    m_timer8_handle.Instance                = TIM8;
    m_timer8_handle.Init.Prescaler          = 0;                                //不分频
    m_timer8_handle.Init.CounterMode        = TIM_COUNTERMODE_CENTERALIGNED2;   //中心对齐模式2
    m_timer8_handle.Init.Period             = PWM_PERIOD;                       //20K
    m_timer8_handle.Init.ClockDivision      = TIM_CLOCKDIVISION_DIV1;
    m_timer8_handle.Init.RepetitionCounter  = 0;
    m_timer8_handle.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&m_timer8_handle) != HAL_OK)
    {
        Error_Handler();
    }

    clk_cfg.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&m_timer8_handle, &clk_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&m_timer8_handle) != HAL_OK)
    {
        Error_Handler();
    }

    //master cfg
    master_cfg.MasterOutputTrigger  = TIM_TRGO_OC4REF;  //  OC4REF信号被用于作为触发输出(TRGO)
    master_cfg.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master_cfg.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&m_timer8_handle, &master_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    //OC cfg
    oc_cfg.OCMode       = TIM_OCMODE_PWM1;
    oc_cfg.Pulse        = 0;
    oc_cfg.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc_cfg.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc_cfg.OCFastMode   = TIM_OCFAST_DISABLE;
    oc_cfg.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc_cfg.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&m_timer8_handle, &oc_cfg, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&m_timer8_handle, &oc_cfg, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&m_timer8_handle, &oc_cfg, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    oc_cfg.Pulse = (PWM_PERIOD - 10); //用来触发 adc 采集
    if (HAL_TIM_PWM_ConfigChannel(&m_timer8_handle, &oc_cfg, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }

    //deadtime
    break_deadtime_cfg.OffStateRunMode  = TIM_OSSR_DISABLE;
    break_deadtime_cfg.OffStateIDLEMode = TIM_OSSI_DISABLE;
    break_deadtime_cfg.LockLevel        = TIM_LOCKLEVEL_OFF;
    break_deadtime_cfg.DeadTime         = 10;                  //死区延时
    break_deadtime_cfg.BreakState       = TIM_BREAK_DISABLE;
    break_deadtime_cfg.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    break_deadtime_cfg.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&m_timer8_handle, &break_deadtime_cfg) != HAL_OK)
    {
        Error_Handler();
    }
    
    //pwm gpio
    pwm_io_init();

    return 0;
}


/**
  * @brief 定时器恢复默认状态
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance == TIM8)
  {
    __HAL_RCC_TIM8_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM8_UP_IRQn, 2, 0);   //对应PWM1模式上升中断
    HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
  }
}

/**
  * @brief 定时器恢复默认状态
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{


}

void TIM8_UP_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&m_timer8_handle);

    //自定义
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM8)  // 20K 的中断频率
    {
        g_tim8_50us_ticks++;

        if(m_timer8_cb_handler != NULL)
        {
            m_timer8_cb_handler();
        }
    }
}

void timer8_irq_cb_register(timer8_irq_cb_t cb)
{
    if(cb == NULL)
    {
        return;
    }

    m_timer8_cb_handler = cb;
}

/**
 * @brief 设置uvw三相PWM占空比
 * 
 * @param u 0 ~ PWM_PERIOD
 * @param v 0 ~ PWM_PERIOD
 * @param w 0 ~ PWM_PERIOD
 */
void phase_pwm_set(uint32_t u, uint32_t v, uint32_t w)
{
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_1, u);
	__HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_2, v);
	__HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_3, w);
}

void phase_pwm_start(void)
{
    HAL_TIM_Base_Start_IT(&m_timer8_handle);    //开启定时器以及中断
//    HAL_TIM_Base_Start(&m_timer8_handle);     //开启定时器不开中断

    HAL_TIM_PWM_Start(&m_timer8_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&m_timer8_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&m_timer8_handle, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&m_timer8_handle, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&m_timer8_handle, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&m_timer8_handle, TIM_CHANNEL_3);
    

    HAL_TIM_PWM_Start(&m_timer8_handle, TIM_CHANNEL_4);
}

void phase_pwm_stop(void)
{
    HAL_TIM_Base_Stop(&m_timer8_handle);

    HAL_TIM_PWM_Stop(&m_timer8_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&m_timer8_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&m_timer8_handle, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&m_timer8_handle, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&m_timer8_handle, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&m_timer8_handle, TIM_CHANNEL_3); 
}
