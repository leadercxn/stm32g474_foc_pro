#include <math.h>
#include "string.h"

#include "boards.h"
#include "adc.h"
#include "sys.h"
#include "util.h"

#include "parameters.h"
#include "sensors_task.h"
#include "motor_ctrl_task.h"
#include "trace.h"

#define SENSORS_TASK_PERIOD_MS      500  //
#define ADC_I_OFFSET_SAMP_TIMES     25   //静态电流采样次数

static uint16_t adc_reg_origin_data[ADC_TOTAL_COLLECT_NUM] = {0};                          //adc规则通道采样原始数据
static uint16_t adc_reg_average_data[ADC_REG_CHAN_NUM] = {0};                              //adc规则通道采样平均数据

static float    adc_sample_physical_value[ADC_REG_CHAN_NUM + ADC_INJ_CHAN_NUM] = {0};      //adc采样物理量数据， 电压单位V, 电流单位A, 温度单位℃

static uint32_t adc_quiescent_i_offset_data[ADC_INJ_CHAN_NUM] = {0};                       //电流偏移adc采样数据 u,v,w
static uint32_t adc_inj_origin_data[ADC_INJ_CHAN_NUM] = {0};

uint16_t adc_reg_sample_data_get(uint8_t ch)
{
    return  adc_reg_average_data[ch];
}

uint16_t adc_inj_sample_data_get(uint8_t ch)
{
    return  (uint16_t)adc_inj_origin_data[ch];
}

float adc_sample_physical_value_get(adc_channel_e ch)
{
    if(ch >= (ADC_REG_CHAN_NUM + ADC_INJ_CHAN_NUM))
    {
        return 0.0f;                            //错误通道
    }

    return adc_sample_physical_value[ch];
}


/**
 * @brief       原始数据平均化处理
 * @param       *p_origin_data  ：ADC原始值数据
 * @param       *p_average_data ADC平均化后的数据缓存
 * @param       ch_num ：采集的ADC通道数量
 * @param       len ：   每个通道的采集次数
 * @retval      无
 */
static void adc_reg_origin_data_average_handler(uint16_t *p_origin_data, uint16_t *p_average_data , uint16_t ch_num, uint16_t len)
{
    uint32_t temp[ADC_REG_CHAN_NUM] = {0};                      /* 定义一个缓存数组 */
    int i , j;

    for (i = 0 ; i < len ; i++)                             /* 根据ADC通道数循环获取，并累加 */
    {
        for (j = 0 ; j < ch_num ; j++)                      /* 将采集到的ADC值，各通道进行累加 */
        {
            temp[j] += p_origin_data[j + i * ch_num];
        }
    }

    for (j = 0 ; j < ch_num ; j++)
    {
        temp[j] /= len;                                     /* 获取平均值 */
        p_average_data[j] = temp[j];                        /* 将滤波后的值存到buf里 */
    }
}

const float Rp = 10000.0f;                  /* 10K */
const float T2 = (273.15f + 25.0f);         /* T2 */
const float Bx = 3380.0f;                   /* B */
const float Ka = 273.15f;

/**
 * @brief       计算温度值 -- 正点原子官方API
 * @param       para: 温度采集对应ADC通道的值（已滤波）
 * @note        计算温度分为两步：
                1.根据ADC采集到的值计算当前对应的Rt
                2.根据Rt计算对应的温度值
 * @retval      温度值
 */
static float temp_get(uint16_t adc_value)
{
    float Rt;
    float temp;
    Rt = 3.3f * 4700.0f / (adc_value * 3.3f / 4096.0f) - 4700.0f;
    /* like this R=5000, T2=273.15+25,B=3470, RT=5000*EXP(3470*(1/T1-1/(273.15+25)) */
    temp = Rt / Rp;
    temp = log(temp);       /* ln(Rt/Rp) */
    temp /= Bx;             /* ln(Rt/Rp)/B */
    temp += (1.0f / T2);
    temp = 1.0f / (temp);
    temp -= Ka;
    return temp;
}

/**
 * @brief       adc 平均值转化为对应的物理量
 */
static void adc_reg_average_data_to_physical_value(void)
{
    // U_VOLT
    adc_sample_physical_value[ADC_CH_U_VOLT] = (float) (adc_reg_average_data[ADC_CH_U_VOLT] * 3.30 * 25.0 / 4096.0f);

    // V_VOLT
    adc_sample_physical_value[ADC_CH_V_VOLT] = (float) (adc_reg_average_data[ADC_CH_V_VOLT] * 3.30 * 25.0 / 4096.0f);

    // W_VOLT
    adc_sample_physical_value[ADC_CH_W_VOLT] = (float) (adc_reg_average_data[ADC_CH_W_VOLT] * 3.30 * 25.0 / 4096.0f);

    // VBUS
    adc_sample_physical_value[ADC_CH_VBUS] = (float) (adc_reg_average_data[ADC_CH_VBUS] * 3.30 * 25.0 / 4096.0f);

    // TEMP
    adc_sample_physical_value[ADC_CH_TEMP] = temp_get(adc_reg_average_data[ADC_CH_TEMP]);
}

static void adc_inj_data_to_physical_value(void)
{
    int temp;
    float   result = 0.0f;

    // U_I
    temp = adc_inj_origin_data[0] - adc_quiescent_i_offset_data[0];
    if(temp >= 0)
    {
        result = temp * (float)(3.3f / 4.0960f / 0.12f);

        result *= 0.001f;

        adc_sample_physical_value[ADC_CH_U_I] = result;
    }
    else
    {
        adc_sample_physical_value[ADC_CH_U_I] = 0.0f;
    }

    // V_I
    temp = adc_inj_origin_data[1] - adc_quiescent_i_offset_data[1];
    if(temp >= 0)
    {
        result = temp * (float)(3.3f / 4.0960f / 0.12f);

        result *= 0.001f;

        adc_sample_physical_value[ADC_CH_V_I] = result;
    }
    else
    {
        adc_sample_physical_value[ADC_CH_V_I] = 0.0f;
    }

    // W_I
    temp = adc_inj_origin_data[2] - adc_quiescent_i_offset_data[2];
    if(temp >= 0)
    {
        result = temp * (float)(3.3f / 4.0960f / 0.12f);

        result *= 0.001f;

        adc_sample_physical_value[ADC_CH_W_I] = result;
    }
    else
    {
        adc_sample_physical_value[ADC_CH_W_I] = 0.0f;
    }
}

//static uint16_t test_ticks = 0;

/**
 * 传感器逻辑任务
 */
int sensors_task(void)
{
    static uint32_t sensors_tasks_ticks = 0;

    static uint32_t quiescent_i_cal_ticks = 0;

    static bool is_init = false;

    static uint32_t quiescent_i_adc_buff[3][ADC_I_OFFSET_SAMP_TIMES] = {0}; //
    static uint8_t  quiescent_i_samp_index = 0;                                 //静态电流采样索引

    if(!is_init)
    {
        adc_reg_start((uint32_t *)adc_reg_origin_data);  //启动DMA传输
        is_init = true;
    }

    if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), quiescent_i_cal_ticks, 10))   //间隔10ms
    {
        quiescent_i_cal_ticks = sys_time_ms_get();
        if(g_app_param.motor_sta == MOTOR_STA_STOP)     //电机处于停止状态
        {
            uint32_t quiescent_i_adc_avg[3] = {0};      //静态电流平均值U,V,W
            
            quiescent_i_adc_buff[0][quiescent_i_samp_index] = adc_inj_origin_data[0];
            quiescent_i_adc_buff[1][quiescent_i_samp_index] = adc_inj_origin_data[1];
            quiescent_i_adc_buff[2][quiescent_i_samp_index] = adc_inj_origin_data[2];

            quiescent_i_samp_index++;
            if(quiescent_i_samp_index > ADC_I_OFFSET_SAMP_TIMES)                //每50次统计一次静态值 10 * 25
            {
                quiescent_i_samp_index = 0;                                     //重置采样索引

                for(uint8_t i = 0; i < 3; i++)
                {
                    quiescent_i_adc_avg[i] = 0;
                    for(uint8_t j = 0; j < ADC_I_OFFSET_SAMP_TIMES; j++)
                    {
                        quiescent_i_adc_avg[i] += quiescent_i_adc_buff[i][j];
                    }

                    quiescent_i_adc_avg[i] /= ADC_I_OFFSET_SAMP_TIMES;          //计算平均值
                    adc_quiescent_i_offset_data[i] = quiescent_i_adc_avg[i];    //保存静态电流偏移数据

//                    trace_debug("quiescent i %d: %lu\r\n", i, adc_quiescent_i_offset_data[i]);
                }
            }
        }
    }

    if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), sensors_tasks_ticks, SENSORS_TASK_PERIOD_MS))
    {

    }

#if 0
    if(test_ticks >= 1000)
    {
        test_ticks = 0;

        trace_debug("sys time ms %lu\r\n", sys_time_ms_get());
    }
#endif

    return 0;
}


/**
 * @brief       规则通道ADC转换完成的回调函数
 * @param       无
 * @retval      无
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC2) 
    {
        adc_reg_stop();  //停止DMA传输

        adc_reg_origin_data_average_handler(adc_reg_origin_data, adc_reg_average_data, ADC_REG_CHAN_NUM, ADC_SAMPLE_NUM);  //对采集到的原始数据进行平均化处理
        adc_reg_average_data_to_physical_value();             //将平均化后的数据转换为物理量

        adc_reg_start((uint32_t *)adc_reg_origin_data);  //重新启动DMA传输
    }
}

/**
 * @brief       注入通道ADC转换完成的回调函数， 参考 adc_inj_start 执行频率，目前应该是 10K
 * @param       无
 * @retval      无
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        adc_inj_origin_data[0] = HAL_ADCEx_InjectedGetValue(&m_adc2_handle, ADC_INJECTED_RANK_1); //U电流
        adc_inj_origin_data[1] = HAL_ADCEx_InjectedGetValue(&m_adc2_handle, ADC_INJECTED_RANK_2); //V电流
        adc_inj_origin_data[2] = HAL_ADCEx_InjectedGetValue(&m_adc2_handle, ADC_INJECTED_RANK_3); //W电流

        adc_inj_data_to_physical_value();

//        test_ticks++;

        if(g_app_param.motor_sta != MOTOR_STA_STOP)     //电机在非停机状态下都要运行
        {
            motor_run();
        }
    }
}

