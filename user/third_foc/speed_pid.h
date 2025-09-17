
#ifndef RTW_HEADER_speed_pid_h_
#define RTW_HEADER_speed_pid_h_
#include <stddef.h>
#ifndef speed_pid_COMMON_INCLUDES_
# define speed_pid_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                               

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#define speed_pid_M                    (rtM)



extern real32_T g_Speed_Ref;                 
extern real32_T g_Speed_Fdk;                 
extern real32_T g_Speed_Pid_Out;             

typedef struct
{
  real32_T P_Gain;
  real32_T I_Gain;
  real32_T D_Gain;
  real32_T B_Gain;
  real32_T Max_Output;
  real32_T Min_Output;
  real32_T I_Sum;

  float last_speed;
	float speed_step_add;

	uint16_T err_time_count;
	uint16_T err_time_flag;

	float speed_ref_last;
	uint8_T speed_start_flag;
	uint8_T speed_reversal_to_forward;
	uint8_T speed_add_flag;
}SPEED_PID_DEF;

extern SPEED_PID_DEF g_Speed_Pid;

extern void speed_pid_initialize(void);
extern void Speed_Pid_Calc(real32_T ref_temp,real32_T fdb_temp,real32_T* out_temp,SPEED_PID_DEF* current_pid_temp);

extern real32_T SPEED_PI_I;
extern real32_T SPEED_PI_KB;
extern real32_T SPEED_PI_LOW_LIMIT;
extern real32_T SPEED_PI_P;
extern real32_T SPEED_PI_UP_LIMIT;


#endif                            
