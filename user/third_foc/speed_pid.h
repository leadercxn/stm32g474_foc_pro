
#ifndef __SPEED_PID_H
#define __SPEED_PID_H

#include <stddef.h>
#include "rtwtypes.h"

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#define speed_pid_M                    (rtM)



extern real32_T g_speed_ref;                 
extern real32_T g_speed_fdk;                 
extern real32_T g_speed_pid_out;             

typedef struct
{
  real32_T p_gain;
  real32_T i_gain;
  real32_T d_gain;
  real32_T b_gain;
  real32_T max_output;
  real32_T min_output;
  real32_T i_sum;

  float last_speed;
	float speed_step_add;

	uint16_T err_time_count;
	uint16_T err_time_flag;

	float   speed_ref_last;
	uint8_T speed_start_flag;
	uint8_T speed_reversal_to_forward;
	uint8_T speed_add_flag;
} speed_pid_t;

extern speed_pid_t g_speed_pid;

extern void speed_pid_param_init(void);
extern void speed_pid_cal(real32_T ref_temp, real32_T fdb_temp, real32_T* out_temp, speed_pid_t* current_pid_temp);

#endif                            
