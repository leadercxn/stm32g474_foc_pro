#include "iir_lpf.h"

//结构体声明引用
iir_butter_t   g_smo_iir_lpf_par_ealfa;	//SMO1输出滤波-低通
iir_butter_t   g_smo_iir_lpf_par_ebeta;	//SMO1输出滤波-低通
iir_butter_t   g_pll_iir_lpf_par;		//PLL输出滤波-低通


void iir_filter(float in ,float *out , iir_butter_t *d_iir_lpf)
{

    float temp;
	// w(n) = gain*x(n) - a1*w(n-1) - a2*w(n-2)
	temp = d_iir_lpf->gain0 * in - (d_iir_lpf->status0 * d_iir_lpf->a1)	\
	        - (d_iir_lpf->status1 * d_iir_lpf->a2);

     //y(n) = gain[b0*w(n) + b1*w(n-1) + b2*w(n-2)]
    *out =  d_iir_lpf->gain1 * (d_iir_lpf->b0*temp + d_iir_lpf->b1 * d_iir_lpf->status0 \
	                            + d_iir_lpf->b2 * d_iir_lpf->status1);

	d_iir_lpf->status1 = d_iir_lpf->status0;
	d_iir_lpf->status0 = temp;
}

static void iir_filter_init(float temp[8], iir_butter_t *d_iir_lpf)
{
	d_iir_lpf->b0 = temp[0];
	d_iir_lpf->b1 = temp[1];	
	d_iir_lpf->b2 = temp[2];
	d_iir_lpf->a0 = temp[3];
	d_iir_lpf->a1 = temp[4];	
	d_iir_lpf->a2 = temp[5];
	d_iir_lpf->gain0 = temp[6];
	d_iir_lpf->gain1 = temp[7];
	d_iir_lpf->status0 = 0.0f;
	d_iir_lpf->status1 = 0.0f;
}

void iir_lpf_param_init(void)
{
	//1阶
//{1.0f,1.0f,0.0f,1.0f,-0.909929f,0.0f,0.045035f,1.0f};10khz  150h截止频率
//{1.0f,1.0f,0.0f,1.0f,-0.939063f,0.0f,0.030468f,1.0f};10khz  100h截止频率
//{1.0f,1.0f,0.0f,1.0f,-0.969067f,0.0f,0.015466f,1.0f};10khz  50h截止频率
	//2阶
//{1.0f,2.0f,1.0f,1.0f,-1.9111970f,0.914975f,0.000944f,1.0f};10khz  100h截止频率	
//{1.0f,2.0f,1.0f,1.0f,-1.9555782f,0.956543f,0.000241f,1.0f};10khz  50h截止频率	

	float smo_iir_coeff[8] = {1.0f, 1.0f, 0.0f, 1.0f, -0.939063f, 0.0f, 0.030468f, 1.0f};//SMO输出的Ealfa Ebeta 滤波
	float pll_iir_coeff[8] = {1.0f, 2.0f, 1.0f, 1.0f, -1.9555782f, 0.956543f, 0.000241f, 1.0f};//PLL输出的We滤波

	iir_filter_init(smo_iir_coeff, &g_smo_iir_lpf_par_ealfa);	//SMO_Ealfa
	iir_filter_init(smo_iir_coeff, &g_smo_iir_lpf_par_ebeta);	//SMO_Ealfa
	iir_filter_init(pll_iir_coeff, &g_pll_iir_lpf_par);			//PLL
}
	

