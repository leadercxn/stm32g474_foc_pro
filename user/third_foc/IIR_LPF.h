#ifndef _IIR_LPF_H
#define _IIR_LPF_H

#include "parameters.h"

typedef struct IIR
{
	float status0;
	float status1;
	float b0;
	float b1;
	float b2;
	float a0;
	float a1;
	float a2;
	float gain0;
	float gain1;
}IIR_Butter_DEF;

extern IIR_Butter_DEF   SMO_IIR_LPF_PAR_Ealfa;//SMO1输出滤波-低通
extern IIR_Butter_DEF   SMO_IIR_LPF_PAR_Ebeta;//SMO1输出滤波-低通
extern IIR_Butter_DEF   PLL_IIR_LPF_PAR;//PLL输出滤波-低通


void IIR_LPF_Start_wrapper(void);
void IIR_filter_Init(float temp[8],IIR_Butter_DEF*d_iir_lpf);
void IIR_filter(float in ,float *out , IIR_Butter_DEF*d_iir_lpf);
#endif


