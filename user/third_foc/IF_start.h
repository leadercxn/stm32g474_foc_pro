#ifndef __IF_START_H
#define __IF_START_H

#include "stdint.h"

typedef struct
{
//定位阶段
	float iq_location;
//速度拉升
	float iq_speed;
	float speed_rad;	//目标速度--单位rad/s
	float speed_acc;	//过程加速度
	float if_time_s;	//过程时间--单位s
	//-----
	float step_time;	//离散时间步长
	float if_we;
	float if_theta;
	uint32_t if_abs_time;
} if_start_t;

extern if_start_t g_if_start_def;



/***
IF强拖启动函数：放在FOC中断里面 --- 10Khz
*/
void if_start_algorithm(float *iq, float *theta, if_start_t *if_start_def);
/**
IF强拖启动参数初始化
**/
void if_start_param_init(void);

#endif



