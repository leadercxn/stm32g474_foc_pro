#include "math.h"
#include "string.h"

#include "boards.h"
#include "parameters.h"
#include "lib_error.h"
#include "util.h"
#include "adc.h"
#include "uart.h"

#include "mbconfig.h"

#include "sensors_task.h"
#include "mb_slaver_task.h"

#undef  TRACE_ENABLE
#include "trace.h"

static uint16_t mb_reg[REG_MAX];        //HMI通信寄存器
static bool     m_sys_reset_req = false;    //系统复位请求

/**
 * @brief 获取想要读取的Coil量的值  fun_code: 0x01
 * 
 * @param [in]  startAddress  读取的起始寄存器地址
 * @param [in]  quantity      读取的数量
 * @param [out] statusList    返回寄存器的数据
 */
void GetCoilStatus(uint16_t startAddress, uint16_t quantity, bool *statusList)
{
    trace_verbose("GetCoilStatus, addr 0x%04x, quantity %d\r\n", startAddress, quantity);

#if 0
    //bit0 bit1 bit2 bit3 .... bitL -> bitH
    uint8_t coil_data[32] = {1,1,0,1,0,1,0,0,0,0,1,1,1,1,1,0};  // 0x2B 0x7C

    for(uint16_t cnt = 0; cnt < quantity; cnt++)
    {
        statusList[cnt] = coil_data[cnt];
    }
#endif

}

/**
 * @brief 获取想要读取的InputStatus量的值   fun_code: 0x02
 * 
 * @param [in]  startAddress  读取的起始寄存器地址
 * @param [in]  quantity      读取的数量
 * @param [out] statusValue   返回寄存器的数据
 */
void GetInputStatus(uint16_t startAddress, uint16_t quantity, bool *statusValue)
{
    trace_verbose("GetInputStatus\r\n");
#if 0
    //bit0 bit1 bit2 bit3 .... bitL -> bitH
    uint8_t coil_data[32] = {1,0,0,1,0,1,0,0,0,0,1,1,0,1,1,0};  // 0x29 0x6C

    for(uint16_t cnt = 0; cnt < quantity; cnt++)
    {
        statusValue[cnt] = coil_data[cnt];
    }
#endif
}

/**
 * @brief 获取想要读取的holding保持寄存器的值   fun_code: 0x03
 * 
 * @param [in]  startAddress    读取的起始寄存器地址
 * @param [in]  quantity        读取的数量
 * @param [out] registerValue   返回寄存器的数据
 */
void GetHoldingRegister(uint16_t startAddress, uint16_t quantity, uint16_t *registerValue)
{
    trace_verbose("GetHoldingRegister\r\n");

    //HMI通信的寄存器
    for(uint16_t cnt = 0; cnt < quantity; cnt++)
    {
        registerValue[cnt] = mb_reg[startAddress + cnt];
    }
}

/**
 * @brief 获取想要读取的输入寄存器的值     fun_code: 0x04
 * 
 * @param [in]  startAddress    读取的起始寄存器地址
 * @param [in]  quantity        读取的数量
 * @param [out] registerValue   返回寄存器的数据
 */
void GetInputRegister(uint16_t startAddress, uint16_t quantity, uint16_t *registerValue)
{
    trace_verbose("GetInputRegister\r\n");
#if 0
    uint16_t reg_data[32] = {0x9999, 0x1111, 0x2222, 0x3333, 0x4444};

    for(uint16_t cnt = 0; cnt < quantity; cnt++)
    {
        registerValue[cnt] = reg_data[cnt];
    }
#endif

    //BMS通信的寄存器
    for(uint16_t cnt = 0; cnt < quantity; cnt++)
    {
//        registerValue[cnt] = mb_bms_reg[startAddress + cnt];
    }
}

/**
 * @brief 设置单个线圈的值  fun_code: 0x05
 * 
 * @param [in]  coilAddress        要写入的线圈地址
 * @param [in]  coilValue          写入的线圈值
 */
void SetSingleCoil(uint16_t coilAddress, bool coilValue)
{
    trace_verbose("SetSingleCoil\r\n");
}

/**
 * @brief 设置单个寄存器的值    fun_code: 0x06
 * 
 * @param [in]  registerAddress    要写入的寄存器地址
 * @param [in]  registerValue      写入的寄存器值
 */
void SetSingleRegister(uint16_t registerAddress, uint16_t registerValue)
{
    trace_verbose("SetSingleRegister\r\n");

    //HMI设置寄存器
    mb_reg[registerAddress] = registerValue;
}

/**
 * @brief 设置多个线圈的值  fun_code: 0x0F
 * 
 * @param [in]  startAddress        要写入的线圈起始地址
 * @param [in]  quantity            写入的线圈数量
 * @param [in]  statusValue         要写入的多个线圈值
 */
void SetMultipleCoil(uint16_t startAddress, uint16_t quantity, bool *statusValue)
{
    trace_verbose("SetMultipleCoil\r\n");
}

/**
 * @brief 设置多个寄存器的值    fun_code: 0x10
 * 
 * @param [in]  startAddress        要写入的寄存器起始地址
 * @param [in]  quantity            写入的寄存器数量
 * @param [in]  registerValue         要写入的多个寄存器值
 */
void SetMultipleRegister(uint16_t startAddress, uint16_t quantity, uint16_t *registerValue)
{
    trace_verbose("SetMultipleRegister\r\n");

    // HMI 通信
    for(uint16_t cnt = 0; cnt < quantity; cnt++)
    {
        mb_reg[startAddress + cnt] = registerValue[cnt];
    }
}

static void sys_update_2_reg(void)
{
    float    temp_f = 0.0f;

    if(g_app_param.motor_sta == MOTOR_STA_STOP)
    {
        mb_reg[REG_SW] = 0;
    }
    else
    {
        mb_reg[REG_SW] = 1;
    }

    mb_reg[REG_DIR]             = g_app_param.motor_dir;

    memcpy((uint8_t *)&mb_reg[REG_TARGET_SPEED_L16], (uint8_t *)&g_app_param.motor_speed_set, 4);

    //速度环参数
    memcpy((uint8_t *)&mb_reg[REG_SPEED_PID_P_L16],     (uint8_t *)&g_mb_ctrl_param.speed_pid_p, 4);
    memcpy((uint8_t *)&mb_reg[REG_SPEED_PID_I_L16],     (uint8_t *)&g_mb_ctrl_param.speed_pid_i, 4);
    memcpy((uint8_t *)&mb_reg[REG_SPEED_PID_KB_L16],    (uint8_t *)&g_mb_ctrl_param.speed_pid_kb, 4);
    memcpy((uint8_t *)&mb_reg[REG_SPEED_PID_LIMIT_L16], (uint8_t *)&g_mb_ctrl_param.speed_pid_limit, 4);

    //电流参数
    memcpy((uint8_t *)&mb_reg[REG_I_PID_P_L16],     (uint8_t *)&g_mb_ctrl_param.i_pid_p, 4);
    memcpy((uint8_t *)&mb_reg[REG_I_PID_I_L16],     (uint8_t *)&g_mb_ctrl_param.i_pid_i, 4);
    memcpy((uint8_t *)&mb_reg[REG_I_PID_KB_L16],    (uint8_t *)&g_mb_ctrl_param.i_pid_kb, 4);
    memcpy((uint8_t *)&mb_reg[REG_I_PID_LIMIT_L16], (uint8_t *)&g_mb_ctrl_param.i_pid_limit, 4);

    //电机参数
    memcpy((uint8_t *)&mb_reg[REG_PHASE_RS_L16],    (uint8_t *)&g_mb_ctrl_param.phase_rs, 4);
    memcpy((uint8_t *)&mb_reg[REG_PHASE_LS_L16],    (uint8_t *)&g_mb_ctrl_param.phase_ls, 4);
    memcpy((uint8_t *)&mb_reg[REG_FLUX_LINK_L16],   (uint8_t *)&g_mb_ctrl_param.flux_link, 4);

    //速度
    memcpy((uint8_t *)&mb_reg[REG_SPEED_MAX_L16],    (uint8_t *)&g_mb_ctrl_param.speed_max, 4);
    memcpy((uint8_t *)&mb_reg[REG_SPEED_MIN_L16],    (uint8_t *)&g_mb_ctrl_param.speed_min, 4);

    //过流，过压阈值
    temp_f = adc_sample_physical_value_get(ADC_CH_VBUS);
    memcpy((uint8_t *)&mb_reg[REG_VBUS_VOLT_L16],    (uint8_t *)&temp_f, 4);

    memcpy((uint8_t *)&mb_reg[REG_I_ERR_TH_L16],     (uint8_t *)&g_mb_ctrl_param.i_err_th, 4);
    memcpy((uint8_t *)&mb_reg[REG_V_ERR_TH_L16],     (uint8_t *)&g_mb_ctrl_param.v_err_th, 4);

    //PLL参数
    memcpy((uint8_t *)&mb_reg[REG_PLL_P_L16],    (uint8_t *)&g_mb_ctrl_param.pll_p, 4);
    memcpy((uint8_t *)&mb_reg[REG_PLL_I_L16],    (uint8_t *)&g_mb_ctrl_param.pll_i, 4);

    mb_reg[REG_POLE_PAIRS] = g_mb_ctrl_param.motor_pole_pairs;

    mb_reg[REG_MB_ADDR]    = g_app_param.slave_addr;

}

static void reg_update_2_sys(void)
{
    //开关机
    if(mb_reg[REG_SW] == 0)
    {
        g_app_param.motor_sta = MOTOR_STA_STOPPING;
    }
    else
    {
        g_app_param.motor_sta   = MOTOR_STA_STARTING;
        g_app_param.iq_acc_dir  = ACC_START;

        g_speed_ref = g_app_param.motor_speed_set;
    }

    //目标速度
    memcpy((uint8_t *)&g_app_param.motor_speed_set, (uint8_t *)&mb_reg[REG_TARGET_SPEED_L16], 4);


    // 只有在关机后，才可以修改参数
    if(g_app_param.motor_sta == MOTOR_STA_STOP)
    {
        if(mb_reg[REG_DIR] <= MOTOR_DIR_CCW)
        {
            g_app_param.motor_dir = mb_reg[REG_DIR];
        }



        // modbus地址
        if( (mb_reg[REG_MB_ADDR] >= 1) && (mb_reg[REG_MB_ADDR] <= 250) )
        {
            g_app_param.slave_addr = mb_reg[REG_MB_ADDR];
        }
    }

}


/**
 * 
 */
int mb_slaver_task(void)
{
    uint8_t tx_data[64] = {0};
    uint8_t tx_len = 0;

    uint8_t m_rx_data[64] = {0};
    uint8_t m_rx_len = 0;

    static bool init_done = false;

    if(init_done == false)
    {
        init_done = true;

        memset((uint8_t *)&mb_reg, 0, sizeof(mb_reg));
    }

    //HMI
    m_rx_len = usart3_rx(m_rx_data);
    if(m_rx_len > 0)
    {
        trace_debug("A1B1 rx len %d data: ", m_rx_len);
        trace_dump(m_rx_data, m_rx_len);

        sys_update_2_reg();

        tx_len = ParsingMasterAccessCommand(m_rx_data, tx_data, m_rx_len, g_app_param.slave_addr);
        if(tx_len != 0XFFFF)
        {
            trace_debug("A1B1 tx len %d data: ", tx_len);
            trace_dump(tx_data, tx_len);

            usart3_tx(tx_data, tx_len);

            reg_update_2_sys();
        }
    }

    //收到复位命令
    if(m_sys_reset_req)
    {
        NVIC_SystemReset();
    }

    return 0;
}

