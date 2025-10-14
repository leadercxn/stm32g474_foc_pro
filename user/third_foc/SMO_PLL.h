#ifndef __SMO_PLL_H
#define __SMO_PLL_H

#include "parameters.h"

typedef struct
{
	float est_i_alfa;
	float est_i_beta;
  	float est_i_alfa_d;
	float est_i_beta_d;
	
	float est_i_alfa_err;
	float est_i_beta_err;
	float v_alfa;
	float v_beta;
} smo_struct_t;

typedef struct
{
   	float we;
	float theta;
	float compensation_theta;
	float p;
	float i;
	float err_sum;
} pll_struct_t;



extern pll_struct_t g_pll;
extern smo_struct_t g_smo;

void smo_observer(float u_alfa, float u_beta, float i_alfa, float i_beta, smo_struct_t *smo);
void pll_control(float e_alfa, float e_beta, pll_struct_t *pll);
void smo_pll_param_init(smo_struct_t *smo, pll_struct_t *pll);

#endif

