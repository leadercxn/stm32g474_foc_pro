#include "parameters.h"
#include "math.h"


app_param_t g_app_param = {
    .slave_addr = 1,

    .motor_sta = MOTOR_STA_STOP,
    .pre_motor_sta = MOTOR_STA_STOP,
    .motor_dir = MOTOR_DIR_CCW,
    .motor_start_acc_sta = MOTOR_START_STA_ACC,

    .motor_speed_set    = 10.0f,

    .target_uq = 0.9f,
    .target_iq = 0.5f,

    .curr_uq = 0.0f,
    .curr_iq = 0.0f,
    .curr_theta = 0.0f,

    .iq_acc_dir = ACC_DONE,
    .is_speed_ring_start = false,

    .evt_code = 0,

    .target_step_angle = 0.001f,
};

mb_ctrl_param_t g_mb_ctrl_param;


