#include "IF_start.h"


//结构体
IF_start_DEF g_IF_start_def;
/**
IF强拖启动参数初始化
**/
void IF_Start_Init(void)
{
	g_IF_start_def.Iq_location = 0.8f;
	
  	g_IF_start_def.Iq_speed  = 0.3f;
	g_IF_start_def.Speed_rad = 80.0f;	//rad/s
	g_IF_start_def.Step_time = 3.0f;	//s
	g_IF_start_def.Speed_acc = g_IF_start_def.Speed_rad / g_IF_start_def.Step_time;
	
	g_IF_start_def.IF_time_S = 0.0001f;
  	g_IF_start_def.IF_theta  = 0.0f;
}




/***
IF强拖启动函数：放在FOC中断里面 --- 10Khz
*/
void IF_start_Algorithm(float *Iq, float *theta, IF_start_DEF *IF_start_Def)
{
  /*************第一阶段---IDLE**************/
	//0.5s
	if((IF_start_Def->IF_abs_time < 5000) && (IF_start_Def->IF_abs_time > 1))
	{
	    *Iq = 0.0f;
		*theta = 0.0f;	
	}
  	/*************第二阶段---强拖定位**************/  
	//0.5
	else if((IF_start_Def->IF_abs_time < 10000) && (IF_start_Def->IF_abs_time > 5000))
	{
	    *Iq = IF_start_Def->Iq_location;
		*theta = 0.0f;
	}
 	/*************第三阶段---速度拉升**************/
  	//
	else if((IF_start_Def->IF_abs_time < 45000) && (IF_start_Def->IF_abs_time > 10000))
	{
	    *Iq = IF_start_Def->Iq_speed;
		IF_start_Def->IF_we += IF_start_Def->Speed_acc * IF_start_Def->IF_time_S;
		    
		IF_start_Def->IF_theta += IF_start_Def->IF_we * IF_start_Def->IF_time_S;

		if(IF_start_Def->IF_theta > 6.28318f)
		{
			IF_start_Def->IF_theta -= 6.28318f;
		}	
		*theta = IF_start_Def->IF_theta;	
	}
 	/*************第三阶段---速度维持**************/
	else if(IF_start_Def->IF_abs_time > 45000)
	{
		IF_start_Def->IF_theta += g_IF_start_def.Speed_rad * IF_start_Def->IF_time_S;
		*Iq = IF_start_Def->Iq_speed;

		if(IF_start_Def->IF_theta>6.28318f)
		{
			IF_start_Def->IF_theta-=6.28318f;
		}	
		*theta = IF_start_Def->IF_theta;	
		IF_start_Def->IF_abs_time = 46000;
	}
	
}