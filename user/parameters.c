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
    0};  /* 电机结构体初始值 */

app_param_t g_app_param = {
    .motor_sta = MOTOR_STA_STOP,
    .motor_dir = MOTOR_DIR_CW,

    .motor_speed_set    = 60,
    .motor_speed_real   = 0,

    .uq = 8.0f,
};


