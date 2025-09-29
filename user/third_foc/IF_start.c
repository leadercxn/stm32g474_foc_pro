#include "if_start.h"


//结构体
if_start_t g_if_start_def;

/**
IF强拖启动参数初始化
**/
void if_start_param_init(void)
{
	g_if_start_def.iq_location = 0.8f;
	
  	g_if_start_def.iq_speed  = 0.3f;
	g_if_start_def.speed_rad = 80.0f;	//rad/s
	g_if_start_def.step_time = 3.0f;	//s
	g_if_start_def.speed_acc = g_if_start_def.speed_rad / g_if_start_def.step_time;
	
	g_if_start_def.if_time_s = 0.0001f;
  	g_if_start_def.if_theta  = 0.0f;
}




/***
IF强拖启动函数：放在FOC中断里面 --- 10Khz
*/
void if_start_algorithm(float *iq, float *theta, if_start_t *if_start_def)
{
  /*************第一阶段---IDLE**************/
	//0.5s
	if((if_start_def->if_abs_time < 5000) && (if_start_def->if_abs_time > 1))
	{
	    *iq = 0.0f;
		*theta = 0.0f;
	}
  	/*************第二阶段---强拖定位**************/  
	//0.5
	else if((if_start_def->if_abs_time < 10000) && (if_start_def->if_abs_time > 5000))
	{
	    *iq = if_start_def->iq_location;
		*theta = 0.0f;
	}
 	/*************第三阶段---速度拉升**************/
  	//
	else if((if_start_def->if_abs_time < 45000) && (if_start_def->if_abs_time > 10000))
	{
	    *iq = if_start_def->iq_speed;
		if_start_def->if_we += if_start_def->speed_acc * if_start_def->if_time_s;

		if_start_def->if_theta += if_start_def->if_we * if_start_def->if_time_s;

		if(if_start_def->if_theta > 6.28318f)
		{
			if_start_def->if_theta -= 6.28318f;
		}
		*theta = if_start_def->if_theta;	
	}
 	/*************第三阶段---速度维持**************/
	else if(if_start_def->if_abs_time > 45000)
	{
		if_start_def->if_theta += g_if_start_def.speed_rad * if_start_def->if_time_s;
		*iq = if_start_def->iq_speed;

		if(if_start_def->if_theta>6.28318f)
		{
			if_start_def->if_theta-=6.28318f;
		}
		*theta = if_start_def->if_theta;	
		if_start_def->if_abs_time = 46000;
	}
	
}