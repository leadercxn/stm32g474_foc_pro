#ifndef PARAMETERS_H__
#define PARAMETERS_H__

#include <stdint.h>

#include "boards.h"
#include "foc.h"
#include "pid.h"

#include "speed_pid.h"
#include "foc_algorithm.h"
#include "smo_pll.h"
#include "iir_lpf.h"
#include "arm_math.h"
#include "if_start.h"

#define SYS_CLK_FREQ    170000000
#define PWM_FREQ        20000       //20K
#define PWM_PERIOD      8500        //(SYS_CLK_FREQ / PWM_FREQ)
#define MAX_PWM_DUTY    ((PWM_PERIOD - 1) * 0.96)  //最大占空比

#define PWM_TIM_PULSE_TPWM  (SYS_CLK_FREQ / (PWM_FREQ  / 2) )   //因为中心对齐

//电机参数
#define MOTOR_POLE_PAIRS    2           //电机极对数

//#define MOTOR_PHASE_RES     0.4f        //电机相电阻，单位欧姆
//#define MOTOR_PHASE_LS      0.0008f     //电机相电感，单位亨利
//#define MOTOR_FLUXLINK      0.01623f    //电机磁链常数

// 别人demo参数
#define MOTOR_PHASE_RES     0.2f        //电机相电阻，单位欧姆
#define MOTOR_PHASE_LS      0.0004f     //电机相电感，单位亨利
#define MOTOR_FLUXLINK      0.0090969f  //电机磁链常数

//程序设定参数
#define MOTOR_SPEED_MAX_RPM     4000  //电机最高转速
#define MOTOR_SPEED_MIN_RPM     100   //电机最小速度
#define VBUS_VLOT               24.0f //母线电压，单位V

//FOC参数
#define FOC_PERIOD              0.0001f     //FOC运行的时间间隔
#define SPEED_LOOP_CLOSE_RAD_S  20.0f       //速度环切入闭环的速度  单位: rad/s

#define RAD_S_2_RING_S          DOUBLE_PI   // rad/s 转换为 转/秒

/**
 * 算法参数
 */
// 速度环默认参数
#define SPEED_PI_P          0.003f
#define SPEED_PI_I          5.0f
#define SPEED_PI_KB         0.015f
#define SPEED_PI_LOW_LIMIT  -6.0f
#define SPEED_PI_UP_LIMIT   6.0f

// Q轴电流环默认参数
#define Q_PI_P              3.199f
#define Q_PI_I              2282.8f
#define Q_PI_KB             15.0f
#define Q_PI_LOW_LIMIT      -10.0f
#define Q_PI_UP_LIMIT       10.0f


// 电机状态
typedef enum
{
    MOTOR_STA_STOP,     //停止

    MOTOR_STA_STOPPING, //停止中
    MOTOR_STA_RUNNING,  //运行中
    MOTOR_STA_STARTING, //启动中

    MOTOR_STA_VF_START,     //vf启动中
    MOTOR_STA_VF_ACC,       //vf加速中
    MOTOR_STA_VF_DEC,       //vf减速中
    MOTOR_STA_VF_CONST,     //vf恒速中

    MOTOR_STA_EKF_START,    //ekf启动中
    MOTOR_STA_EKF_ACC,      //ekf加速中
    MOTOR_STA_EKF_DEC,      //ekf减速中
    MOTOR_STA_EKF_CONST,    //ekf恒速中

    MOTOR_STA_ERROR,        //故障状态
} motor_sta_e;

typedef enum
{
    MOTOR_START_STA_ACC,        //加速中
    MOTOR_START_STA_ACC_END,    //加速完成
    MOTOR_START_STA_CONST,      //恒速转动
} motor_start_sta_e;

typedef enum
{
    MOTOR_DIR_CW,      //顺时针
    MOTOR_DIR_CCW,     //逆时针
} motor_dir_e;

typedef enum
{
    ACC_DONE,       //变速完成
    ACC_UP,         //加速
    ACC_DOWN,       //减速
    ACC_START,      //开始加速
} motor_acc_dir_e;

typedef enum
{
    MOTOR_CMD_NONE,      //无命令
    MOTOR_CMD_STARTUP,   //启动命令
    MOTOR_CMD_STOP,      //停止命令
    MOTOR_CMD_RESET,     //复位命令
    MOTOR_CMD_STOP_NIW,  //紧急停止命令
} motor_cmd_e;

/*********** MODBUS **************/
typedef enum
{
    REG_SW = 0x01,          //开关机
    REG_DIR,                //电机方向

    REG_TARGET_SPEED_L16,   //目标速度 低16位
    REG_TARGET_SPEED_H16,   //目标速度 高16位

//电机调试参数
    REG_SPEED_PID_P_L16,    //速度环P参数 低16位
    REG_SPEED_PID_P_H16,    //速度环P参数 高16位
    REG_SPEED_PID_I_L16,    //速度环I参数 低16位
    REG_SPEED_PID_I_H16,    //速度环I参数 高16位
    REG_SPEED_PID_KB_L16,   //速度环Kb参数 低16位
    REG_SPEED_PID_KB_H16,   //速度环Kb参数 高16位
    REG_SPEED_PID_LIMIT_L16,   //速度环幅值 低16位
    REG_SPEED_PID_LIMIT_H16,   //速度环幅值 高16位

    REG_I_PID_P_L16,        //电流环P参数 低16位
    REG_I_PID_P_H16,        //电流环P参数 高16位
    REG_I_PID_I_L16,        //电流环I参数 低16位
    REG_I_PID_I_H16,        //电流环I参数 高16位
    REG_I_PID_KB_L16,       //电流环Kb参数 低16位
    REG_I_PID_KB_H16,       //电流环Kb参数 高16位
    REG_I_PID_LIMIT_L16,    //电流环幅值 低16位
    REG_I_PID_LIMIT_H16,    //电流环幅值 高16位

    REG_PHASE_RS_L16,       //相电阻 低16位
    REG_PHASE_RS_H16,       //相电阻 高16位
    REG_PHASE_LS_L16,       //相电感 低16位
    REG_PHASE_LS_H16,       //相电感 高16位
    REG_FLUX_LINK_L16,      //磁链 低16位
    REG_FLUX_LINK_H16,      //磁链 高16位

    REG_SPEED_MAX_L16,      //最大速度 低16位
    REG_SPEED_MAX_H16,      //最大速度 高16位
    REG_SPEED_MIN_L16,      //最小速度 低16位
    REG_SPEED_MIN_H16,      //最小速度 高16位

    REG_I_ERR_TH_L16,       //过流阈值 低16位
    REG_I_ERR_TH_H16,       //过流阈值 高16位
    REG_V_ERR_TH_L16,       //过压阈值 低16位
    REG_V_ERR_TH_H16,       //过压阈值 高16位

    REG_PLL_P_L16,          //PLL p参数 低16位
    REG_PLL_P_H16,          //PLL p参数 高16位
    REG_PLL_I_L16,          //PLL i参数 低16位
    REG_PLL_I_H16,          //PLL i参数 高16位

    REG_POLE_PAIRS,         //电机极对数
    REG_MB_ADDR,            //modbus地址

// 运行状态参数
    REG_VBUS_VOLT,     //母线电压
    REG_BSP_TEMP,      //板载温度
    REG_U_VOLT,        //U相电压
    REG_V_VOLT,        //V相电压
    REG_W_VOLT,        //W相电压
    REG_U_CURR,        //U相电流
    REG_V_CURR,        //V相电流
    REG_W_CURR,        //W相电流

    REG_CURR_SPEED,    //当前速度
    REG_CURR_THETA,    //当前角度

    REG_EVT_CODE0 = 124,    //事件码 Bit0 ~ bit15
    REG_EVT_CODE1,          //事件码 Bit16 ~ bit31
    REG_EVT_CODE2,          //事件码 Bit32 ~ bit47
    REG_EVT_CODE3,          //事件码 Bit48 ~ bit63

    REG_MAX = 128,
} mb_reg_e;

// 显示故障bit排位 L -> H
typedef enum {
    EVT_I_SHORT,            //短路
    EVT_OVER_CUR,           //过流
    EVT_MB_OVER_VOLT,       //直流母线过压
    EVT_MB_UNDER_VOLT,      //直流母线欠压

    EVT_TEMP_SENS_ERR,      //温度传感器故障
    EVT_RAD_OVER_TEMP,      //散热片过温
    EVT_ROTOR_ABNORMAL,     //转子异常(堵转)
    EVT_INPUT_PHASE_LOSS,   //输入 缺相

    EVT_OUTPUT_PHASE_LOSS,  //输出 缺相
    EVT_OVER_CUR_REDU_FREQ, //过流降频
    EVT_LIMIT_FREQ,         //限频
    EVT_IPM_ERR,            //IPM 模块故障 (IGBT模块)

    EVT_STARTUP_FAIL,       //启动失败
    EVT_BOX_OVER_TEMP,      //机箱过温
    EVT_IGBT_OVER_TEMP,     //IGBT过温
    EVT_OUT_OVER_VOLT,      //输出过压

    EVT_OUT_UNDER_VOLT,     //输出欠压
    EVT_OTHER_FAULT,        //其他故障
    EVT_U_UNDER_VOLT,       //U相欠压
    EVT_V_UNDER_VOLT,       //V相欠压

    EVT_W_UNDER_VOLT,       //W相欠压
} sys_evt_e;



/*********** 电机状态结构体 **************/

// 全局应用参数
typedef struct
{
    uint8_t     slave_addr;                     // modbus 从机地址

    motor_sta_e         motor_sta;              // 电机状态
    motor_sta_e         pre_motor_sta;          // 电机前一状态

    motor_dir_e         motor_dir;              // 电机方向
    motor_start_sta_e   motor_start_acc_sta;    // 电机启动加速状态
    motor_cmd_e         motor_cmd;              // 电机命令
    motor_cmd_e         old_motor_cmd;          // 上一次电机命令

    float       target_speed_ring_s;    // 电机设定速度，单位 ring/s 圈/秒
    float       curr_speed_rad_s;       // 电机当前速度，单位 rad/s  弧度/秒

    float       vf_target_uq;       // q轴电压 单位V
    float       target_iq;          // q轴电流 单位A

    float       vf_curr_uq;         //  vf当前Uq
    float       curr_iq;            //  当前Iq
    float       vf_curr_theta;      //  vf当前角度值

    float       vf_step_rad;        //  VF阶段 步进角度，单位：弧度
    float       ekf_step_ring_s;    //  EKF阶段 步进角度，单位：圈/秒

    motor_acc_dir_e     iq_acc_dir;     // iq加速的方向,  0：iq达标  1：iq加速  2:iq减速 4:开始加速
    bool        is_speed_ring_start;    // 速度环开始标记

    uint64_t    evt_code;               // 事件代码
} app_param_t;

extern app_param_t g_app_param;

//调试参数
typedef struct
{
    //速度环参数
    float    speed_pid_p;        //
    float    speed_pid_i;        //
    float    speed_pid_kb;       //
    float    speed_pid_limit;    //正负对称

    //电流环参数
    float    i_pid_p;            //
    float    i_pid_i;            //
    float    i_pid_kb;           //
    float    i_pid_limit;        //正负对称

    //电机参数
    float    phase_rs;           // 相电阻
    float    phase_ls;           // 相电感
    float    flux_link;          // 磁链

    float    speed_max;          // 最大速度 1 = 1 RPM
    float    speed_min;          // 最小速度 1 = 1 RPM

    float    i_err_th;           // 母线过流阈值
    float    v_err_th;           // 母线过压阈值

    float    pll_p;              // pll p参数
    float    pll_i;              // pll i参数

    uint16_t motor_pole_pairs;   // 电机极对数
} mb_ctrl_param_t;


extern mb_ctrl_param_t g_mb_ctrl_param;

#endif
