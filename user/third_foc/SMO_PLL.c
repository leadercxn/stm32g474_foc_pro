#include "SMO_PLL.h"
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
float Ld = 2500.0f;
// RLd = R/Ld
float RLd = 500.0f;


float Gain_h = 7.0f;

 PLL_DEF PLL_def;//PLL锁相环结构体

 SMO_Struct_DEF SMO_Struct_def;//滑膜结构体
 
 
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
   if(*date >1.0f) re=1.0f;
	 else if(*date <-1.0f)re=-1.0f;
	 else re = *date ;	 
	 return re;
}
/*
*滑膜观测函数
*输入 We Ualfa  Ubeta
*输出Ealfa Ebeta
*
*
*/
void SMO_Observer(float Ualfa,float Ubeta,float Ialfa,float Ibeta,SMO_Struct_DEF*SMO_Struct)
{
   //
	SMO_Struct->estimate_Ialfa_D = (Ld*Ualfa)+ (-RLd*SMO_Struct->estimate_Ialfa)
	                                +(-Ld*SMO_Struct->Valfa);
	SMO_Struct->estimate_Ibeta_D = (Ld*Ubeta)+ (-RLd*SMO_Struct->estimate_Ibeta)
	                                +(-Ld*SMO_Struct->Vbeta);
	//积分
	SMO_Struct->estimate_Ialfa += SMO_Struct->estimate_Ialfa_D*FOC_PERIOD;
	SMO_Struct->estimate_Ibeta +=	SMO_Struct->estimate_Ibeta_D*FOC_PERIOD;
	//实际值-估计值
  	SMO_Struct->estimate_Ialfa_err = SMO_Struct->estimate_Ialfa - Ialfa;
	SMO_Struct->estimate_Ibeta_err = SMO_Struct->estimate_Ibeta - Ibeta;
	//输出
	SMO_Struct->Valfa = sign( &SMO_Struct->estimate_Ialfa_err) *Gain_h;
	SMO_Struct->Vbeta = sign( &SMO_Struct->estimate_Ibeta_err) *Gain_h;
	//IIR滤波
	IIR_filter(SMO_Struct->Valfa ,&SMO_Struct->Valfa, &SMO_IIR_LPF_PAR_Ealfa);
	IIR_filter(SMO_Struct->Vbeta ,&SMO_Struct->Vbeta, &SMO_IIR_LPF_PAR_Ebeta);
}
/*
*PLL控制函数
*
*输入Ealfa Ebeta
*输出 We theta
*
*/
void PLL_control(float Ealfa,float Ebeta,PLL_DEF*PLL_Def)
{
   float err = 0.0f;
	 err = -Ealfa*arm_cos_f32(PLL_Def->compensation_theta) - Ebeta*arm_sin_f32(PLL_Def->compensation_theta);
	 PLL_Def->we = PLL_Def->P *err +  PLL_Def->err_sum;
	 PLL_Def->err_sum += PLL_Def->I * err * FOC_PERIOD;
	//角速度积分->角度
	PLL_Def->compensation_theta+= PLL_Def->we * FOC_PERIOD;
		if(PLL_Def->compensation_theta>2.0f*PI)
	{
	  PLL_Def->compensation_theta -=2.0f*PI;
	}
	if(PLL_Def->compensation_theta<0.0f*PI)
	{
	   PLL_Def->compensation_theta +=2.0f*PI;
	}
	PLL_Def->theta  =  PLL_Def->compensation_theta;
		
	 //we输出滤波
	IIR_filter(PLL_Def->we ,&PLL_Def->we, &PLL_IIR_LPF_PAR);
	//电机反转补偿π
	if((PLL_Def->we<-10.0f)&&(g_Speed_Ref<0.0f))
	{
		PLL_Def->theta +=PI;

		if(PLL_Def->theta>2.0f*PI)
		{
			 PLL_Def->theta -=2.0f*PI;
		}
		if(PLL_Def->theta<0.0f*PI)
		{
			 PLL_Def->theta +=2.0f*PI;
		}
	}

}

void SMO_PLL_Init(SMO_Struct_DEF*SMO_Struct,PLL_DEF*PLL_Def)
{
		SMO_Struct->estimate_Ialfa = 0.0f;
		SMO_Struct->estimate_Ialfa_D = 0.0f;
		SMO_Struct->estimate_Ialfa_err= 0.0f;
		SMO_Struct->estimate_Ibeta= 0.0f;
		SMO_Struct->estimate_Ibeta_D= 0.0f;
		SMO_Struct->estimate_Ibeta_err= 0.0f;
		SMO_Struct->Valfa= 0.0f;
		SMO_Struct->Vbeta= 0.0f;

		PLL_Def->err_sum = 0.0f;
		PLL_Def->P = 600.0f;
		PLL_Def->I = 15000.0f;
		PLL_Def->theta = 0.0f;
		PLL_Def->we  =0.0f;
}


