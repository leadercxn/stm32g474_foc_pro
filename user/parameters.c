#include "parameters.h"
#include "math.h"


app_param_t g_app_param = {
    .slave_addr = 1,

    .motor_sta              = MOTOR_STA_STOP,
    .pre_motor_sta          = MOTOR_STA_STOP,
    .motor_dir              = MOTOR_DIR_CCW,
    .motor_start_acc_sta    = MOTOR_START_STA_ACC,
    .motor_cmd              = MOTOR_CMD_NONE,
    .old_motor_cmd          = MOTOR_CMD_NONE,

    .target_speed_ring_s    = 10.0f,

    .vf_target_uq = 0.9f,
    .target_iq = 0.5f,

    .vf_curr_uq     = 0.0f,
    .curr_iq        = 0.0f,
    .vf_curr_theta  = 0.0f,

    .iq_acc_dir = ACC_DONE,
    .is_speed_ring_start = false,

    .evt_code = 0,

    .vf_step_rad = 0.001f,
    .ekf_step_ring_s = 1.0f,
};

mb_ctrl_param_t g_mb_ctrl_param;


