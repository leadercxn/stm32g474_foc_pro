/**********************************
            
**********************************/
#include "speed_pid.h"
#include "parameters.h"

#define SPEED_PID_PERIOD 0.001F

real32_T SPEED_PI_I   = 5.0F;
real32_T SPEED_PI_KB  = 0.015F;
real32_T SPEED_PI_LOW_LIMIT = -5.0F;
real32_T SPEED_PI_P   = 0.003F;
real32_T SPEED_PI_UP_LIMIT = 5.0F;

                   
real32_T g_Speed_Ref;        //速度参考          
real32_T g_Speed_Fdk;        //速度反馈          
real32_T g_Speed_Pid_Out;    //速度PID输出，也就是Q轴电流环的参考             

SPEED_PID_DEF g_Speed_Pid;

void Speed_Pid_Calc(real32_T ref_temp,real32_T fdb_temp,real32_T* out_temp,SPEED_PID_DEF* current_pid_temp)
{

  real32_T error;
  real32_T temp;


#if 1
    /**********梯形缓冲************/
    if(ref_temp != current_pid_temp->speed_ref_last)
		{
			error = ref_temp - current_pid_temp->speed_ref_last;

			if(error > 0.0f)
			{
				current_pid_temp->speed_ref_last += current_pid_temp->speed_step_add;
			}
			else
			{
				current_pid_temp->speed_ref_last -= current_pid_temp->speed_step_add;
			}

		  if((error < 0.5f) && (error > -0.5f))
			{
				current_pid_temp->speed_ref_last = ref_temp;				
			}
		}

    //正常情况反转1----反转情况
    if((ref_temp < 0.0f) && ((PLL_def.we < -20.0f) || (g_FOC_Output.EKF[2] < -20.0f)))
    {
      current_pid_temp->speed_reversal_to_forward = 1;
    }
      //正常情况正转2---正转情况
    else if((ref_temp > 0.0f) && ((PLL_def.we > 20.0f) || (g_FOC_Output.EKF[2] > 20.0f)))
    {
      current_pid_temp->speed_reversal_to_forward = 0;
    }
    else
    {
      current_pid_temp->speed_reversal_to_forward = 2;
    }
#endif

  error = 6.28318548F * ref_temp - fdb_temp;             //2*pi的作用是 单位转换   Hz转换为rad/s

#if 1

    if((current_pid_temp->speed_start_flag == 1) || (g_app_param.is_speed_ring_start))
    {
      //给定正转--实际SMO反转情况1
      if((ref_temp > 1.0f) && (fdb_temp < -400.0f)) 
      {			
        current_pid_temp->err_time_count++;
        error -= 100.0f * current_pid_temp->err_time_count;			 
      }	
      //给定反转--实际SMO正转情况2
      else if((ref_temp < -1.0f) && (fdb_temp > 400.0f))
      {
        current_pid_temp->err_time_count++;
        error += 100.0f * current_pid_temp->err_time_count;				
      }
      else
      {
        current_pid_temp->err_time_count=0;
      }

      //不在可控范围内
      if((fdb_temp > 500)||(fdb_temp < -500))
      {
        SMO_PLL_Init(&SMO_Struct_def, &PLL_def) ;
        current_pid_temp->err_time_count = 0;
      }
    }

#endif

  temp = (error + current_pid_temp->I_Sum) * current_pid_temp->P_Gain;

  if (temp > current_pid_temp->Max_Output)
  {
    *out_temp = current_pid_temp->Max_Output;
  }
  else if (temp < current_pid_temp->Min_Output)
  {
    *out_temp = current_pid_temp->Min_Output;
  }
  else
  {
    *out_temp = temp;
  }

  current_pid_temp->I_Sum += ( (*out_temp - temp) * current_pid_temp->B_Gain + current_pid_temp->I_Gain* error) * SPEED_PID_PERIOD;
}


void speed_pid_initialize(void)
{
  g_Speed_Pid.P_Gain = SPEED_PI_P;
  g_Speed_Pid.I_Gain = SPEED_PI_I;
  g_Speed_Pid.B_Gain = SPEED_PI_KB;
  g_Speed_Pid.Max_Output = SPEED_PI_UP_LIMIT;
  g_Speed_Pid.Min_Output = SPEED_PI_LOW_LIMIT;
  g_Speed_Pid.I_Sum = 0.0f;

  g_Speed_Pid.speed_step_add    = 0.5f;
	g_Speed_Pid.speed_ref_last    = 0.0f;
	g_Speed_Pid.speed_start_flag  = 0;
	g_Speed_Pid.err_time_count    = 0;
	g_Speed_Pid.err_time_flag     = 0;
}


