#include "parameters.h"
#include "math.h"


app_param_t g_app_param = {
    .motor_sta = MOTOR_STA_STOP,
    .pre_motor_sta = MOTOR_STA_STOP,
    .motor_dir = MOTOR_DIR_CW,
    .motor_start_acc_sta = MOTOR_START_STA_ACC,

    .motor_speed_set    = 60,
    .motor_speed_real   = 0,

    .target_uq = 4.0f,
    .target_iq = 1.0f,

    .curr_uq = 0.0f,
    .curr_theta = 0.0f,
};

// ekf pi控制器
pi_cal_t g_iq_pi;
pi_cal_t g_id_pi;



