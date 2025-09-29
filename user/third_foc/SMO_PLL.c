#include "smo_pll.h"
//#define RS_PARAMETER     0.2f           //电阻
//#define LS_PARAMETER     0.0004f          //电感
//#define FLUX_PARAMETER   0.0090969f        //磁链

//float R = 0.2f;
//float Ld = 0.0004f;
//float flux = 0.00950f;
//提前计算，减小MCU内核消耗

//// Ld =  1/Ld
//float Ld = 1000.0f;
//// RLd = R/Ld
//float RLd = 500.0f;

// Ld =  1/Ld
static float m_ld = 2500.0f;
// RLd = R/Ld
static float m_rld = 500.0f;

static float m_gain_h = 7.0f;

pll_struct_t g_pll;		//PLL锁相环结构体
smo_struct_t g_smo;		//滑膜结构体
 
 
/*
*符号函数：sgn（x）
*x >0 1
*x <0 -1
*
*
*/
float sign(float *date)
{
	float re = 0;

   	if(*date > 1.0f)
		re = 1.0f;
	else if(*date < -1.0f)
		re = -1.0f;
	else
		re = *date;

	return re;
}

/*
*滑膜观测函数
*输入 We u_alfa  u_beta
*输出 e_alfa e_beta
*
*/
void smo_observer(float u_alfa, float u_beta, float i_alfa, float i_beta, smo_struct_t *smo)
{
	smo->est_i_alfa_d = (m_ld*u_alfa) + (-m_rld*smo->est_i_alfa) + (-m_ld*smo->v_alfa);
	smo->est_i_beta_d = (m_ld*u_beta) + (-m_rld*smo->est_i_beta) + (-m_ld*smo->v_beta);
	//积分
	smo->est_i_alfa += smo->est_i_alfa_d * FOC_PERIOD;
	smo->est_i_beta +=	smo->est_i_beta_d * FOC_PERIOD;
	//实际值-估计值
  	smo->est_i_alfa_err = smo->est_i_alfa - i_alfa;
	smo->est_i_beta_err = smo->est_i_beta - i_beta;
	//输出
	smo->v_alfa = sign( &smo->est_i_alfa_err) * m_gain_h;
	smo->v_beta = sign( &smo->est_i_beta_err) * m_gain_h;
	//IIR滤波
	iir_filter(smo->v_alfa, &smo->v_alfa, &g_smo_iir_lpf_par_ealfa);
	iir_filter(smo->v_beta, &smo->v_beta, &g_smo_iir_lpf_par_ebeta);
}

/*
*PLL控制函数
*
*输入 e_alfa e_beta
*输出 We theta
*
*/
void pll_control(float e_alfa, float e_beta, pll_struct_t *pll)
{
    float err = 0.0f;
	err = -e_alfa * arm_cos_f32(pll->compensation_theta) - e_beta * arm_sin_f32(pll->compensation_theta);
	pll->we = pll->p *err +  pll->err_sum;
	pll->err_sum += pll->i * err * FOC_PERIOD;

	//角速度积分->角度
	pll->compensation_theta+= pll->we * FOC_PERIOD;
	if(pll->compensation_theta > DOUBLE_PI)
	{
	  pll->compensation_theta -= DOUBLE_PI;
	}

	if(pll->compensation_theta < 0.0f)
	{
	   pll->compensation_theta += DOUBLE_PI;
	}

	pll->theta  =  pll->compensation_theta;
		
	 //we输出滤波
	iir_filter(pll->we ,&pll->we, &g_pll_iir_lpf_par);
	//电机反转补偿π
	if((pll->we < -10.0f) && (g_speed_ref < 0.0f))
	{
		pll->theta += PI;

		if(pll->theta > DOUBLE_PI)
		{
			pll->theta -= DOUBLE_PI;
		}

		if(pll->theta < 0.0f)
		{
			pll->theta += DOUBLE_PI;
		}
	}
}

void smo_pll_param_init(smo_struct_t *smo, pll_struct_t *pll)
{
	smo->est_i_alfa 	= 0.0f;
	smo->est_i_alfa_d 	= 0.0f;
	smo->est_i_alfa_err	= 0.0f;
	smo->est_i_beta		= 0.0f;
	smo->est_i_beta_d	= 0.0f;
	smo->est_i_beta_err	= 0.0f;
	smo->v_alfa			= 0.0f;
	smo->v_beta			= 0.0f;

	pll->err_sum 		= 0.0f;
	pll->p 				= 600.0f;
	pll->i 				= 15000.0f;
	pll->theta 			= 0.0f;
	pll->we  			= 0.0f;
}


