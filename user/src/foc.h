#ifndef FOC_H__
#define FOC_H__

/**
 * 在使用时，建议优先用宏，宏运算 比 调用 sqrtf 函数快
 */
#define SQRT_3                  1.73205080757f  //√3
#define SQRT_3_DIV_2            0.86602540378f  //√3 / 2
#define SQRT_3_DIV_3            0.57735026919f  //√3 / 3
#define DOUBLE_SQRT_3_DIV_3     1.15470053838f  //2 * √3 / 3

#define PI                      3.14159265359f  //圆周率
#define PI_DIV_3                1.04719755120f  //PI / 3                
#define PI_DIV_2                1.57079632679f  //PI / 2
#define PI_DIV_4                0.78539816339f  //PI / 4
#define PI_DIV_5                0.62831853072f  //PI / 5
#define PI_DIV_6                0.52359877560f  //PI / 6
#define DOUBLE_PI               6.28318530718f  //2 * PI
#define ONE_DIV_PI              0.31830988618f  //1 / PI

typedef struct
{
    float u;
    float v;
    float w;

    float alpha;
    float beta;

    float d;
    float q;

    float pos_radian;  //位置角度，单位弧度
} foc_param_t;

typedef struct {
    float kp;
    float ki;
    float out_max;
    float iout_max;
    float out;
    float sum_err;
} pi_cal_t;

float radian_normalize(float radian);
void pi_cal(pi_cal_t *sptr, float error);

void svpwm_set(float u, float v, float w);
void torque_set(float uq, float ud, float pos_radian);

#endif
