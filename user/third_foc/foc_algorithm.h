#ifndef __FOC_ALGORITHM_H
#define __FOC_ALGORITHM_H
#include <stddef.h>
#include "rtwtypes.h"

#include "parameters.h"

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#define foc_algorithm_M                (rtM)


typedef struct tag_RTM RT_MODEL;


typedef struct
{
  real_T EKF_States[4];   
  real_T L_Ident_States;     
  real_T R_flux_Ident_States;    
  real32_T EKF_Interface[7];
  real32_T R_flux_Ident_Interface[3];
  real32_T L_Ident_Interface[2];
  real32_T R_flux_Ident_Output[2];       
  real32_T L_Ident_Output;          
} foc_interface_sts_t;


typedef struct
{
  real32_T id_ref;                     
  real32_T iq_ref;                     
  real32_T speed_fdk;                  
  real32_T theta;                      
  real32_T ia;                         
  real32_T ib;                         
  real32_T ic;                         
  real32_T udc;                        
  real32_T tpwm;                       
  real32_T rs;                         
  real32_T ls;
  real32_T flux;                       
} foc_input_t;


typedef struct
{
  real32_T Tcmp1;                      
  real32_T Tcmp2;                      
  real32_T Tcmp3;                      
  real32_T EKF[4];  //0:i_alpha  1:ibeta  2:角速度omiga  3:转子位置 theta
  real32_T L_RF[3];                    
} foc_output_t;


typedef struct
{
  real32_T ia;
  real32_T ib;
  real32_T ic;
} current_abc_t;

typedef struct
{
  real32_T i_alpha;
  real32_T i_beta;
} current_alpha_beta_t;

typedef struct
{
  real32_T v_alpha;
  real32_T v_beta;
} volt_alpha_beta_t;

typedef struct
{
  real32_T cos;
  real32_T sin;
} transf_cos_sin_t;

typedef struct
{
  real32_T id;
  real32_T iq;
} current_dq_t;

typedef struct
{
  real32_T vd;
  real32_T vq;
} volt_dq_t;

typedef struct
{
  real32_T p_gain;
  real32_T i_gain;
  real32_T d_gain;
  real32_T b_gain;
  real32_T max_output;
  real32_T min_output;
  real32_T i_sum;
} current_pid_t;

struct tag_RTM {
  const char_T *errorStatus;
};

extern foc_input_t    g_foc_input;
extern foc_output_t   g_foc_output;
extern volt_dq_t      g_voltage_dq;
extern current_dq_t   g_current_dq;

extern void foc_algorithm_init(void);
extern void foc_algorithm_step(void);

extern RT_MODEL *const rtM;


#endif                                

