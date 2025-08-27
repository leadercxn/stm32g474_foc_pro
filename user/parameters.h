#ifndef PARAMETERS_H__
#define PARAMETERS_H__

#include <stdint.h>

#include "boards.h"
#include "foc.h"

#define SYS_CLK_FREQ    170000000
#define PWM_FREQ        20000       //20K
#define PWM_PERIOD      8500        //(SYS_CLK_FREQ / PWM_FREQ)
#define MAX_PWM_DUTY    ((PWM_PERIOD - 1) * 0.96)  //最大占空比
//#define PWM_PERIOD      9444        //170000000 / 18000 = 9444
//#define PWM_PERIOD      4722        //170000000 / 36000 = 4722


#define MOTOR_POLE_PAIR 2           //电机极对数
#define MOTOR_PHASE_RES 0.4f        //电机相电阻，单位欧姆
#define MOTOR_PHASE_LS  0.0008f     //电机相电感，单位亨利

#define MOTOR_SPEED_MAX_RPM   4000  //电机最高转速
#define MOTOR_SPEED_MIN_RPM   10  //电机最小速度

#define MOTOR_UQ_MAX    240         //电机q轴电压最大值，单位0.1V
#define VBUS_VLOT       24.0f

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
    MOTOR_DIR_CW,      //顺时针
    MOTOR_DIR_CCW,     //逆时针
} motor_dir_e;

/***************************************** 电机状态结构体 ***********************************************/
typedef struct {
    uint8_t    run_flag;       //运行标志  0- 停止  1- 运行
    uint8_t    locked_rotor;   //堵转标记  0- 否    1-堵转
    uint8_t    dir;            //电机旋转方向   0顺时针 1逆时针
    int32_t    pos;            /* 电机位置 */
    int32_t    speed;          /* 电机速度 */
    int16_t    current;        /* 电机速度 */
    uint16_t   pwm_duty;       /* 电机占空比 */
    uint32_t   lock_time;      /* 电机堵转时间 */
    uint32_t   no_single;
    uint32_t   count_j;
} bldc_obj_t;

// 全局应用参数
typedef struct
{
    motor_sta_e motor_sta;          // 电机状态
    motor_dir_e motor_dir;          // 电机方向

    uint16_t    motor_speed_set;      // 电机设定速度，单位RPM
    uint16_t    motor_speed_real;     // 电机实际速度，单位RPM

    float       uq;     // q轴电压

    foc_param_t foc_i;  // 电机foc i电流值
    foc_param_t foc_u;  // 电机foc u电压值
} app_param_t;

extern app_param_t g_app_param;
extern bldc_obj_t  g_bldc_motor;


#endif
