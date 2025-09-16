#ifndef _IF_Start_H
#define _IF_Start_H

#include "stdint.h"

typedef struct
{

//定位阶段
	float Iq_location;
//速度拉升
	float Iq_speed;
	float Speed_rad;//目标速度--单位rad/s
	float Speed_acc;//过程加速度
	float IF_time_S;//过程时间--单位s
	//-----
	float Step_time;//离散时间步长
	float IF_we;
	float IF_theta;
	uint32_t IF_abs_time;
}IF_start_DEF;

extern IF_start_DEF g_IF_start_def;



/***
IF强拖启动函数：放在FOC中断里面 --- 10Khz
*/
void IF_start_Algorithm(float *Iq, float *theta, IF_start_DEF *IF_start_Def);
/**
IF强拖启动参数初始化
**/
void IF_Start_Init(void);

#endif



