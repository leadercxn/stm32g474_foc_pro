/**********************************
            
**********************************/
#include "speed_pid.h"
#include "parameters.h"

#define SPEED_PID_PERIOD    0.001F

real32_T g_speed_ref;        //速度参考, 目标速度， rad/s
real32_T g_speed_fdk;        //速度反馈，实际速度,  rad/s
real32_T g_speed_pid_out;    //速度PID输出，也就是Q轴电流环的参考             

speed_pid_t g_speed_pid;

void speed_pid_cal(real32_T ref_temp, real32_T fdb_temp, real32_T* out_temp, speed_pid_t* current_pid_temp)
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
#endif

  error = 6.28318548F * ref_temp - fdb_temp;             //2*pi的作用是 单位转换   Hz转换为rad/s

#if 1
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
        current_pid_temp->err_time_count = 0;
      }
#endif

  temp = (error + current_pid_temp->i_sum) * current_pid_temp->p_gain;

  if (temp > current_pid_temp->max_output)
  {
    *out_temp = current_pid_temp->max_output;
  }
  else if (temp < current_pid_temp->min_output)
  {
    *out_temp = current_pid_temp->min_output;
  }
  else
  {
    *out_temp = temp;
  }

  current_pid_temp->i_sum += ( (*out_temp - temp) * current_pid_temp->b_gain + current_pid_temp->i_gain* error) * SPEED_PID_PERIOD;
}


void speed_pid_param_init(void)
{
  g_speed_pid.p_gain     = g_mb_ctrl_param.speed_pid_p;
  g_speed_pid.i_gain     = g_mb_ctrl_param.speed_pid_i;
  g_speed_pid.b_gain     = g_mb_ctrl_param.speed_pid_kb;
  g_speed_pid.max_output = g_mb_ctrl_param.speed_pid_limit;
  g_speed_pid.min_output = -g_mb_ctrl_param.speed_pid_limit;
  g_speed_pid.i_sum      = 0.0f;

  g_speed_pid.speed_step_add    = 0.5f;
	g_speed_pid.speed_ref_last    = 0.0f;
	g_speed_pid.speed_start_flag  = 0;
	g_speed_pid.err_time_count    = 0;
	g_speed_pid.err_time_flag     = 0;
}


