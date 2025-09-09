#include "boards.h"
#include "timer.h"
#include "parameters.h"

#include "gpio.h"
#include "trace.h"
#include "six_steps.h"

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
#ifdef SIX_STEPS_ENABLE     //六步换相驱动

    gpio_init_struct.Pin       = PWM_UH_PIN  | PWM_VH_PIN  | PWM_WH_PIN | PWM_CH4_PIN;
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull      = GPIO_NOPULL;
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio_init_struct.Alternate = GPIO_AF4_TIM8;

    HAL_GPIO_Init(PWM_UH_PORT, &gpio_init_struct);

    gpio_init_struct.Pin       = PWM_UL_PIN | PWM_VL_PIN |  PWM_WL_PIN ;
    gpio_init_struct.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull      = GPIO_NOPULL;
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_MEDIUM;

    HAL_GPIO_Init(PWM_UL_PORT, &gpio_init_struct);
#else

    gpio_init_struct.Pin       = PWM_UH_PIN | PWM_UL_PIN | PWM_VH_PIN | PWM_VL_PIN | PWM_WH_PIN | PWM_WL_PIN | PWM_CH4_PIN;
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull      = GPIO_NOPULL;
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio_init_struct.Alternate = GPIO_AF4_TIM8;

    HAL_GPIO_Init(PWM_UH_PORT, &gpio_init_struct);
#endif

}

int timer8_init(void)
{
    TIM_ClockConfigTypeDef          clk_cfg = {0};
    TIM_MasterConfigTypeDef         master_cfg = {0};
    TIM_OC_InitTypeDef              oc_cfg = {0};
    TIM_BreakDeadTimeConfigTypeDef  break_deadtime_cfg = {0};

#ifdef SIX_STEPS_ENABLE     //六步换相驱动
    //timer base cfg
    m_timer8_handle.Instance                = TIM8;
    m_timer8_handle.Init.Prescaler          = 0;                                //不分频
    m_timer8_handle.Init.CounterMode        = TIM_COUNTERMODE_UP;               //向上计数模式
    m_timer8_handle.Init.Period             = PWM_PERIOD;                       //20K
    m_timer8_handle.Init.ClockDivision      = TIM_CLOCKDIVISION_DIV1;
    m_timer8_handle.Init.RepetitionCounter  = 0;
    m_timer8_handle.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_DISABLE;   //统计好下次的时间后，再加载CCR
#else
    //timer base cfg
    m_timer8_handle.Instance                = TIM8;
    m_timer8_handle.Init.Prescaler          = 0;                                //不分频
    m_timer8_handle.Init.CounterMode        = TIM_COUNTERMODE_CENTERALIGNED2;   //中心对齐模式2
    m_timer8_handle.Init.Period             = PWM_PERIOD;                       //20K
    m_timer8_handle.Init.ClockDivision      = TIM_CLOCKDIVISION_DIV1;
    m_timer8_handle.Init.RepetitionCounter  = 0;
    m_timer8_handle.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;
#endif

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

    oc_cfg.Pulse        = PWM_PERIOD * 2;    // ch4 是作为触发adc采集的信号， 10K的采集频率
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

    HAL_TIM_PWM_Start(&m_timer8_handle, TIM_CHANNEL_4);         //开始adc

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

    HAL_NVIC_SetPriority(TIM8_UP_IRQn, 0, 0);   //对应PWM1模式上升中断
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

#ifdef SIX_STEPS_ENABLE     //六步换相驱动
        zero_ctr_loop();        //无感控制逻辑
#endif

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

#ifdef SIX_STEPS_ENABLE     //六步换相驱动
    HAL_TIM_PWM_Start(&m_timer8_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&m_timer8_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&m_timer8_handle, TIM_CHANNEL_3);
#else
    HAL_TIM_PWM_Start(&m_timer8_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&m_timer8_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&m_timer8_handle, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&m_timer8_handle, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&m_timer8_handle, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&m_timer8_handle, TIM_CHANNEL_3);
#endif
    
}

void phase_pwm_stop(void)
{
    HAL_TIM_Base_Stop(&m_timer8_handle);

#ifdef SIX_STEPS_ENABLE     //六步换相驱动
    HAL_TIM_PWM_Stop(&m_timer8_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&m_timer8_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&m_timer8_handle, TIM_CHANNEL_3);
#else
    HAL_TIM_PWM_Stop(&m_timer8_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&m_timer8_handle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&m_timer8_handle, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&m_timer8_handle, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&m_timer8_handle, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&m_timer8_handle, TIM_CHANNEL_3); 
#endif
    
}


/**
 * 六步换相API
 * 
 */
void motor_stop(void)
{
    phase_pwm_stop();

    phase_pwm_set(0, 0, 0);

    gpio_output_set(PWM_UL_PORT, PWM_UL_PIN, 0);
    gpio_output_set(PWM_VL_PORT, PWM_VL_PIN, 0);
    gpio_output_set(PWM_WL_PORT, PWM_WL_PIN, 0);
}

void motor_start(void)
{
    phase_pwm_start();
}

/**
  * @brief  U相上桥臂导通，V相下桥臂导通
  * @param  无
  * @retval 无
  */
void motor_uhvl(void)
{
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_1, g_bldc_motor.pwm_duty);
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_3, 0);
    
    HAL_GPIO_WritePin(PWM_VL_PORT, PWM_VL_PIN, GPIO_PIN_SET);     /* V相下桥臂导通 */
    HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_RESET);   /* U相下桥臂关闭 */
    HAL_GPIO_WritePin(PWM_WL_PORT, PWM_WL_PIN, GPIO_PIN_RESET);   /* W相下桥臂关闭 */
}

/**
  * @brief  U相上桥臂导通，W相下桥臂导通
  * @param  无
  * @retval 无
  */
void motor_uhwl(void)
{
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_1, g_bldc_motor.pwm_duty);
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_3, 0);
    
    HAL_GPIO_WritePin(PWM_WL_PORT, PWM_WL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWM_VL_PORT, PWM_VL_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  V相上桥臂导通，W相下桥臂导通
  * @param  无
  * @retval 无
  */
void motor_vhwl(void)
{
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_2, g_bldc_motor.pwm_duty);
	__HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_3, 0);
    
    HAL_GPIO_WritePin(PWM_WL_PORT, PWM_WL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWM_VL_PORT, PWM_VL_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  V相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void motor_vhul(void)
{
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_2, g_bldc_motor.pwm_duty);
	__HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_3, 0);
    
    HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PWM_VL_PORT, PWM_VL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWM_WL_PORT, PWM_WL_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  W相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void motor_whul(void)
{
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_3, g_bldc_motor.pwm_duty);

    HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PWM_VL_PORT, PWM_VL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWM_WL_PORT, PWM_WL_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  W相上桥臂导通，V相下桥臂导通
  * @param  无
  * @retval 无
  */
void motor_whvl(void)
{
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&m_timer8_handle, TIM_CHANNEL_3, g_bldc_motor.pwm_duty);

    HAL_GPIO_WritePin(PWM_VL_PORT, PWM_VL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWM_WL_PORT, PWM_WL_PIN, GPIO_PIN_RESET);
}

