#ifndef _SMO_PLL_H
#define _SMO_PLL_H
#include "parameters.h"

typedef struct SMO
{
	float estimate_Ialfa;
	float estimate_Ibeta;
  float estimate_Ialfa_D;
	float estimate_Ibeta_D;
	
	float estimate_Ialfa_err;
	float estimate_Ibeta_err;
	float Valfa;
	float Vbeta;


}SMO_Struct_DEF;

typedef struct PLL
{
   float we;
	 float theta;
	 float compensation_theta;
	 float P;
	 float I;
	 float err_sum;

}PLL_DEF;



extern PLL_DEF PLL_def;

extern SMO_Struct_DEF SMO_Struct_def;

void SMO_Observer(float Ualfa,float Ubeta,float Ialfa,float Ibeta,SMO_Struct_DEF*SMO_Struct);
void PLL_control(float Ealfa,float Ebeta,PLL_DEF*PLL_Def);
void SMO_PLL_Init(SMO_Struct_DEF*SMO_Struct,PLL_DEF*PLL_Def);
#endif

