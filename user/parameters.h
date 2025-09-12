#ifndef PARAMETERS_H__
#define PARAMETERS_H__

#include <stdint.h>

#include "boards.h"
#include "foc.h"
#include "pid.h"

#include "speed_pid.h"
#include "foc_algorithm.h"
#include "SMO_PLL.h"
#include "IIR_LPF.h"
#include "arm_math.h"

#define SYS_CLK_FREQ    170000000
#define PWM_FREQ        20000       //20K
#define PWM_PERIOD      8500        //(SYS_CLK_FREQ / PWM_FREQ)
#define MAX_PWM_DUTY    ((PWM_PERIOD - 1) * 0.96)  //最大占空比

#define PWM_TIM_PULSE_TPWM  (SYS_CLK_FREQ / (PWM_FREQ  / 2) )   //因为中心对齐，

//电机参数
#define MOTOR_POLE_PAIRS    2           //电机极对数
#define MOTOR_I_MAX         19.80f      //电机最大电流，单位A
#define MOTOR_RATED_I       6.6f        //电机额定电流，单位A
#define MOTOR_RATED_V       24.0f       //电机额定电压，单位V

#define MOTOR_PHASE_RES     0.4f        //电机相电阻，单位欧姆
#define MOTOR_PHASE_LS      0.0008f     //电机相电感，单位亨利
#define MOTOR_FLUXLINK      0.01623f    //电机磁链常数

// 别人demo参数
//#define MOTOR_PHASE_RES     0.2f        //电机相电阻，单位欧姆
//#define MOTOR_PHASE_LS      0.0004f     //电机相电感，单位亨利
//#define MOTOR_FLUXLINK      0.0090969f  //电机磁链常数

//程序设定参数
#define MOTOR_SPEED_MAX_RPM     4000  //电机最高转速
#define MOTOR_SPEED_MIN_RPM     10    //电机最小速度
#define MOTOR_UQ_MAX            240   //电机q轴电压最大值，单位0.1V
#define VBUS_VLOT               24.0f //母线电压，单位V

//FOC参数
#define FOC_PERIOD              0.0001f     //FOC运行的时间间隔
#define SPEED_LOOP_CLOSE_RAD_S  20.0f       //速度环切入闭环的速度  单位: rad/s

#define FIRST_ORDER_LPF(OUT, IN, FAC)  OUT = (1.0f - FAC) * OUT + FAC * IN;  /*一阶滤波 */

#define VOFA_PRINTF     printf      //配合vofa上位机使用

// 电机状态
typedef enum
{
    MOTOR_STA_STOP,     //停止
    MOTOR_STA_STOPPING, //停止中
    MOTOR_STA_RUNNING,  //运行中
    MOTOR_STA_STARTING, //启动中

    MOTOR_STA_ERROR,    //故障状态
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

/***************************************** 电机状态结构体 ***********************************************/

// 全局应用参数
typedef struct
{
    motor_sta_e motor_sta;          // 电机状态
    motor_sta_e pre_motor_sta;      // 电机前一状态

    motor_dir_e motor_dir;          // 电机方向

    motor_start_sta_e   motor_start_acc_sta;    //电机启动加速状态

    uint16_t    motor_speed_set;    // 电机设定速度，单位RPM
    uint16_t    motor_speed_real;   // 电机实际速度，单位RPM

    float       target_uq;          // q轴电压 单位V
    float       target_iq;          // q轴电流 单位A

    float       curr_uq;            //  当前Uq
    float       curr_iq;            //  当前Iq
    float       curr_theta;         //  当前角度值

    float       foc_ts;             // FOC计算周期，单位s

    float       ekf_theta;
    float       ekf_angle_speed;
    float       ekf_u_alpha;
    float       ekf_u_beta;


    motor_acc_dir_e     iq_acc_dir;     // iq加速的方向,  0：iq达标  1：iq加速  2:iq减速 4:开始加速
    bool        is_speed_ring_start;    // 速度环开始标记
} app_param_t;

extern app_param_t g_app_param;

extern pi_cal_t g_iq_pi;
extern pi_cal_t g_id_pi;

#endif
