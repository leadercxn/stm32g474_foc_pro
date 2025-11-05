#include "string.h"

#include "boards.h"
#include "util.h"
#include "adc.h"

#include "parameters.h"
#include "trace.h"

#include "monitor_task.h"
#include "sensors_task.h"

#define OUTPUT_PHASE_UNDER_VOLT_TH      (15.0f)    //输出相欠压阈值  单位V
#define OUTPUT_PHASE_UNDER_VOLT_TICK    (200)      //输出相欠压检测时间  单位 10ms
#define MBUS_VOLT_UNDER_TH              (20.0f)    //母线欠压阈值   单位V
#define MBUS_VOLT_OVER_TH               (30.0f)    //母线过压阈值   单位V

int monitor_task(void)
{
    static uint32_t monitor_ticks = 0;

    static uint16_t u_under_volt_tick = 0;
    static uint16_t v_under_volt_tick = 0;
    static uint16_t w_under_volt_tick = 0;

    static uint16_t m_under_volt_tick = 0;
    static uint16_t m_over_volt_tick  = 0;

    float  temp_f = 0.0f;

    if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), monitor_ticks, 10))   //间隔 10ms
    {
        monitor_ticks = sys_time_ms_get();

        temp_f = adc_sample_physical_value_get(ADC_CH_U_VOLT);  // U相电压
        if(temp_f < OUTPUT_PHASE_UNDER_VOLT_TH)
        {
            u_under_volt_tick++;

            if(u_under_volt_tick > OUTPUT_PHASE_UNDER_VOLT_TICK)
            {
                u_under_volt_tick = OUTPUT_PHASE_UNDER_VOLT_TICK;

                SET_BIT64(g_app_param.evt_code, EVT_U_UNDER_VOLT);
            }
        }
        else
        {
            u_under_volt_tick = 0;

            CLR_BIT64(g_app_param.evt_code, EVT_U_UNDER_VOLT);
        }

        temp_f = adc_sample_physical_value_get(ADC_CH_V_VOLT);  // V相电压
        if(temp_f < OUTPUT_PHASE_UNDER_VOLT_TH)
        {
            v_under_volt_tick++;

            if(v_under_volt_tick > OUTPUT_PHASE_UNDER_VOLT_TICK)
            {
                v_under_volt_tick = OUTPUT_PHASE_UNDER_VOLT_TICK;

                SET_BIT64(g_app_param.evt_code, EVT_V_UNDER_VOLT);
            }
        }
        else
        {
            v_under_volt_tick = 0;

            CLR_BIT64(g_app_param.evt_code, EVT_V_UNDER_VOLT);
        }

        temp_f = adc_sample_physical_value_get(ADC_CH_W_VOLT);  // W相电压
        if(temp_f < OUTPUT_PHASE_UNDER_VOLT_TH)
        {
            u_under_volt_tick++;

            if(u_under_volt_tick > OUTPUT_PHASE_UNDER_VOLT_TICK)
            {
                u_under_volt_tick = OUTPUT_PHASE_UNDER_VOLT_TICK;

                SET_BIT64(g_app_param.evt_code, EVT_W_UNDER_VOLT);
            }
        }
        else
        {
            u_under_volt_tick = 0;

            CLR_BIT64(g_app_param.evt_code, EVT_W_UNDER_VOLT);
        }

        // 母线电压
        temp_f = adc_sample_physical_value_get(ADC_CH_VBUS);
        if(temp_f < MBUS_VOLT_UNDER_TH)              //欠压
        {
            m_under_volt_tick++;
            m_over_volt_tick = 0;
        }
        else if(temp_f > MBUS_VOLT_OVER_TH)          //过压
        {
            m_under_volt_tick = 0;
            m_over_volt_tick++;
        }
        else                                        //正常范围
        {
            m_under_volt_tick = 0;
            m_over_volt_tick  = 0;

        }


    }

    return 0;
}









