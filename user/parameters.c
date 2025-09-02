#include "parameters.h"
#include "math.h"

bldc_obj_t g_bldc_motor = {
    0,
    0,
    0,
    MOTOR_DIR_CCW,
    0,
    0,
    0,
    0,
    0,
    0   };  /* 电机结构体初始值 */

app_param_t g_app_param = {
    .motor_sta = MOTOR_STA_STOP,
    .motor_dir = MOTOR_DIR_CW,

    .motor_speed_set    = 60,
    .motor_speed_real   = 0,

    .uq = 4.0f,
    .iq = 1.0f,
};

smo_pll_t g_smo_pll = {
    .rs = MOTOR_PHASE_RES,
    .ls = MOTOR_PHASE_LS,

    .h = 0.3f,

    .pll_kp = 500.0f,
    .pll_ki = 10000.0f,
};

current_foc_t g_current_foc = {0};

// 电流环PID
pid_obj_t g_moter_i_loop_pid = {
    .mode = PID_POSITION,
    .kp = 1.2f,
    .ki = 5.0f,

    .max_out  = VBUS_VLOT * 0.5f * 0.5f,   //电流环的输出就是Uq
    .max_iout = 20.0f,
};

// 速度环PID
pid_obj_t g_moter_vel_loop_pid = {
    .mode = PID_POSITION,
    .kp = 0.02f,
    .ki = 0.5f,

    .max_out  = MOTOR_I_MAX * 0.5f * 0.3f,   //速度环的输出是电流值, 是电流环的输入
    .max_iout = 200.0f,
};
