#ifndef __IIR_LPF_H
#define __IIR_LPF_H

typedef struct
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
} iir_butter_t;

extern iir_butter_t   g_smo_iir_lpf_par_ealfa;	//SMO1输出滤波-低通
extern iir_butter_t   g_smo_iir_lpf_par_ebeta;	//SMO1输出滤波-低通
extern iir_butter_t   g_pll_iir_lpf_par;		//PLL输出滤波-低通


void iir_lpf_param_init(void);
void iir_filter(float in ,float *out , iir_butter_t *d_iir_lpf);
#endif


