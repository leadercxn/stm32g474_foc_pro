#include "foc_algorithm.h"

#include "trace.h"

#define D_PI_I          2282.8f
#define D_PI_KB         15.0f
#define D_PI_LOW_LIMIT  -10.0f
#define D_PI_P          3.199f
#define D_PI_UP_LIMIT   10.0f

#define Q_PI_I          2282.8f
#define Q_PI_KB         15.0f
#define Q_PI_LOW_LIMIT  -10.0f
#define Q_PI_P          3.199f
#define Q_PI_UP_LIMIT   10.0f


foc_input_t   g_foc_input;
foc_output_t  g_foc_output;
current_dq_t  g_current_dq; 
volt_dq_t     g_voltage_dq;

RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

extern void stm32_ekf_start_wrapper(real_T *xD);
extern void stm32_ekf_outputs_wrapper(const real32_T *u,
                                        real32_T *y,
                                        const real_T *xD);
extern void stm32_ekf_update_wrapper(const real32_T *u,
                                       real32_T *y,
                                       real_T *xD);  

static foc_interface_sts_t    m_foc_interface_sts;
static current_abc_t          m_current_i_abc;
static current_alpha_beta_t   m_current_alpha_beta;
static volt_alpha_beta_t      m_volt_alpha_beta;
static transf_cos_sin_t       m_transf_cos_sin;
static current_pid_t          m_current_d_pid;
static current_pid_t          m_current_q_pid;

/***************************************
功能：Clark变换
形参：三相电流以及alpha_beta电流
说明：由三相互差120度变换到两相互差90度
***************************************/
static void clarke_transf(current_abc_t current_abc_temp, current_alpha_beta_t* current_alpha_beta_temp)
{
  current_alpha_beta_temp->i_alpha = (current_abc_temp.ia - (current_abc_temp.ib + current_abc_temp.ic) * 0.5F) * 2.0F / 3.0F;
  current_alpha_beta_temp->i_beta = (current_abc_temp.ib - current_abc_temp.ic) * 0.866025388F * 2.0F / 3.0F;
}
/***************************************
功能：SVPWM计算
形参：alpha_beta电压以及母线电压、定时器周期
说明：根据alpha_beta电压计算三相占空比
***************************************/
void svpwm_calc(volt_alpha_beta_t v_alpha_beta_temp, real32_T udc_temp, real32_T tpwm_temp)
{
  int32_T sector;
  real32_T Tcmp1,Tcmp2,Tcmp3,Tx,Ty,f_temp,Ta,Tb,Tc;
  sector = 0;
  Tcmp1 = 0.0F;
  Tcmp2 = 0.0F;
  Tcmp3 = 0.0F;
  if (v_alpha_beta_temp.v_beta > 0.0F) {
    sector = 1;
  }
  
  if ((1.73205078F * v_alpha_beta_temp.v_alpha - v_alpha_beta_temp.v_beta) / 2.0F > 0.0F) {
    sector += 2;
  }
  
  if ((-1.73205078F * v_alpha_beta_temp.v_alpha - v_alpha_beta_temp.v_beta) / 2.0F > 0.0F) {
    sector += 4;
  }

//  trace_debug("sector %d, alpha %.3f, beta %.3f\r\n", sector, v_alpha_beta_temp.v_alpha, v_alpha_beta_temp.v_beta);

  switch (sector) {
  case 1:
    Tx = (-1.5F * v_alpha_beta_temp.v_alpha + 0.866025388F * v_alpha_beta_temp.v_beta) * (tpwm_temp / udc_temp);
    Ty = (1.5F * v_alpha_beta_temp.v_alpha + 0.866025388F * v_alpha_beta_temp.v_beta) * (tpwm_temp / udc_temp);
    break;
    
  case 2:
    Tx = (1.5F * v_alpha_beta_temp.v_alpha + 0.866025388F * v_alpha_beta_temp.v_beta) * (tpwm_temp / udc_temp);
    Ty = -(1.73205078F * v_alpha_beta_temp.v_beta * tpwm_temp / udc_temp);
    break;
    
  case 3:
    Tx = -((-1.5F * v_alpha_beta_temp.v_alpha + 0.866025388F * v_alpha_beta_temp.v_beta) * (tpwm_temp / udc_temp));
    Ty = 1.73205078F * v_alpha_beta_temp.v_beta * tpwm_temp / udc_temp;
    break;
    
  case 4:
    Tx = -(1.73205078F * v_alpha_beta_temp.v_beta * tpwm_temp / udc_temp);
    Ty = (-1.5F * v_alpha_beta_temp.v_alpha + 0.866025388F * v_alpha_beta_temp.v_beta) * (tpwm_temp / udc_temp);
    break;
    
  case 5:
    Tx = 1.73205078F * v_alpha_beta_temp.v_beta * tpwm_temp / udc_temp;
    Ty = -((1.5F * v_alpha_beta_temp.v_alpha + 0.866025388F * v_alpha_beta_temp.v_beta) * (tpwm_temp / udc_temp));
    break;
    
  default:
    Tx = -((1.5F * v_alpha_beta_temp.v_alpha + 0.866025388F * v_alpha_beta_temp.v_beta) * (tpwm_temp / udc_temp));
    Ty = -((-1.5F * v_alpha_beta_temp.v_alpha + 0.866025388F * v_alpha_beta_temp.v_beta) * (tpwm_temp / udc_temp));
    break;
  }
  
  f_temp = Tx + Ty;
  if (f_temp > tpwm_temp) {
    Tx /= f_temp;
    Ty /= (Tx + Ty);
  }
  
  Ta = (tpwm_temp - (Tx + Ty)) / 4.0F;
  Tb = Tx / 2.0F + Ta;
  Tc = Ty / 2.0F + Tb;
  switch (sector) {
  case 1:
    Tcmp1 = Tb;
    Tcmp2 = Ta;
    Tcmp3 = Tc;
    break;
    
  case 2:
    Tcmp1 = Ta;
    Tcmp2 = Tc;
    Tcmp3 = Tb;
    break;
    
  case 3:
    Tcmp1 = Ta;
    Tcmp2 = Tb;
    Tcmp3 = Tc;
    break;
    
  case 4:
    Tcmp1 = Tc;
    Tcmp2 = Tb;
    Tcmp3 = Ta;
    break;
    
  case 5:
    Tcmp1 = Tc;
    Tcmp2 = Ta;
    Tcmp3 = Tb;
    break;
    
  case 6:
    Tcmp1 = Tb;
    Tcmp2 = Tc;
    Tcmp3 = Ta;
    break;
  }
  
  g_foc_output.Tcmp1 = Tcmp1;
  g_foc_output.Tcmp2 = Tcmp2;
  g_foc_output.Tcmp3 = Tcmp3;
}

/***************************************
功能：COS_SIN值计算
形参：角度以及COS_SIN结构体
说明：COS_SIN值计算
***************************************/
void angle_to_cos_sin(real32_T angle_temp, transf_cos_sin_t* cos_sin_temp)
{
  cos_sin_temp->cos = arm_cos_f32(angle_temp);
  cos_sin_temp->sin = arm_sin_f32(angle_temp);
}

/***************************************
功能：PARK变换
形参：alpha_beta电流、COS_SIN值、DQ轴电流
说明：交流变直流
***************************************/
static void park_transf(current_alpha_beta_t current_alpha_beta_temp, transf_cos_sin_t cos_sin_temp, current_dq_t *current_dq_temp)
{
  current_dq_temp->id = current_alpha_beta_temp.i_alpha * cos_sin_temp.cos + current_alpha_beta_temp.i_beta * cos_sin_temp.sin;
  current_dq_temp->iq = -current_alpha_beta_temp.i_alpha * cos_sin_temp.sin + current_alpha_beta_temp.i_beta * cos_sin_temp.cos;
}

/***************************************
功能：反PARK变换
形参：DQ轴电压、COS_SIN值、alpha_beta电压
说明：直流变交流
***************************************/
void rev_park_transf(volt_dq_t v_dq_temp, transf_cos_sin_t cos_sin_temp, volt_alpha_beta_t* v_alpha_beta_temp)
{
  v_alpha_beta_temp->v_alpha = cos_sin_temp.cos * v_dq_temp.vd - cos_sin_temp.sin * v_dq_temp.vq;
  v_alpha_beta_temp->v_beta  = cos_sin_temp.sin * v_dq_temp.vd + cos_sin_temp.cos * v_dq_temp.vq;
}

/***************************************
功能：电流环PID
形参：电流参考、电流反馈、电压输出、PID结构体
说明：根据电流误差去调节电流输出
***************************************/
static void current_pid_calc(real32_T ref_temp, real32_T fdb_temp, real32_T* out_temp, current_pid_t* current_pid_temp)
{
  real32_T error;
  real32_T temp;
  error = ref_temp - fdb_temp;
  temp = current_pid_temp->p_gain * error + current_pid_temp->i_sum;
  if (temp > current_pid_temp->max_output) 
  {
    *out_temp = current_pid_temp->max_output;
  } 
  else if (temp < current_pid_temp->min_output) 
  {
    *out_temp = current_pid_temp->min_output;
  } 
  else 
  {
    *out_temp = temp;
  }
  current_pid_temp->i_sum += ((*out_temp - temp) * current_pid_temp->b_gain + current_pid_temp->i_gain * error) *FOC_PERIOD;
}



void foc_algorithm_step(void)
{
  m_current_i_abc.ia = g_foc_input.ia;         //三相电流赋值
  m_current_i_abc.ib = g_foc_input.ib;
  m_current_i_abc.ic = g_foc_input.ic;

  clarke_transf(m_current_i_abc, &m_current_alpha_beta);                                    //CLARK 变换
  angle_to_cos_sin(g_foc_input.theta, &m_transf_cos_sin);                                   //由角度计算 park变换和 反park变换的 COS SIN值
  park_transf(m_current_alpha_beta, m_transf_cos_sin, &g_current_dq);                       //Park变换，由Ialpha Ibeta 与角度信息，去计算Id Iq  // 由交流信息转化为直流信息，方便PID控制 

	current_pid_calc(g_foc_input.id_ref, g_current_dq.id, &g_voltage_dq.vd, &m_current_d_pid); //D轴电流环PID  根据电流参考与电流反馈去计算 输出电压
  current_pid_calc(g_foc_input.iq_ref, g_current_dq.iq, &g_voltage_dq.vq, &m_current_q_pid); //Q轴电流环PID  根据电流参考与电流反馈去计算 输出电压

  rev_park_transf(g_voltage_dq, m_transf_cos_sin, &m_volt_alpha_beta);                     //反park变换  通过电流环得到的dq轴电压信息结合角度信息，去把直流信息转化为交流信息用于SVPWM的输入

  m_foc_interface_sts.EKF_Interface[0] = m_volt_alpha_beta.v_alpha;   //扩展卡尔曼估计转子位置与速度需要的输入信息
  m_foc_interface_sts.EKF_Interface[1] = m_volt_alpha_beta.v_beta;    //状态观测器输入
  m_foc_interface_sts.EKF_Interface[2] = m_current_alpha_beta.i_alpha;
  m_foc_interface_sts.EKF_Interface[3] = m_current_alpha_beta.i_beta;
  m_foc_interface_sts.EKF_Interface[4] = g_foc_input.rs;
  m_foc_interface_sts.EKF_Interface[5] = g_foc_input.ls;
  m_foc_interface_sts.EKF_Interface[6] = g_foc_input.flux;

  smo_observer(m_volt_alpha_beta.v_alpha, m_volt_alpha_beta.v_beta, m_current_alpha_beta.i_alpha, m_current_alpha_beta.i_beta, &g_smo);

  stm32_ekf_outputs_wrapper(&m_foc_interface_sts.EKF_Interface[0], &g_foc_output.EKF[0],  //扩展卡尔曼估计转子位置与速度的输出函数
                            &m_foc_interface_sts.EKF_States[0]);

  svpwm_calc(m_volt_alpha_beta, g_foc_input.udc, g_foc_input.tpwm);       //SVPWM 计算模块

  stm32_ekf_update_wrapper(&m_foc_interface_sts.EKF_Interface[0], &g_foc_output.EKF[0],   //扩展卡尔曼滤波算法的计算
                           &m_foc_interface_sts.EKF_States[0]);  

  pll_control(g_smo.v_alfa, g_smo.v_beta, &g_pll);
}

void foc_algorithm_init(void)
{
  //电流环PID 参数 初始化
  {
    m_current_d_pid.p_gain      = D_PI_P;
    m_current_d_pid.i_gain      = D_PI_I;
    m_current_d_pid.b_gain      = D_PI_KB;
    m_current_d_pid.max_output  = D_PI_UP_LIMIT;
    m_current_d_pid.min_output  = D_PI_LOW_LIMIT;
    m_current_d_pid.i_sum       = 0.0f;//注意积分值需要清零
    
    m_current_q_pid.p_gain      = Q_PI_P;
    m_current_q_pid.i_gain      = Q_PI_I;
    m_current_q_pid.b_gain      = Q_PI_KB;
    m_current_q_pid.max_output  = Q_PI_UP_LIMIT;
    m_current_q_pid.min_output  = Q_PI_LOW_LIMIT;
    m_current_q_pid.i_sum       = 0.0f;
  }

  speed_pid_param_init();  //速度环PID 参数 初始化
	
  smo_pll_param_init(&g_smo, &g_pll) ;  
	
  stm32_ekf_start_wrapper(&m_foc_interface_sts.EKF_States[0]);//扩展卡尔曼滤波算法 参数初始化

  iir_lpf_param_init();

  //状态变量初始化
  {
    real_T initVector[4] = { 0, 0, 0, 0 };
    
    {
      int_T i1;
      real_T *dw_DSTATE = &m_foc_interface_sts.EKF_States[0];
      for (i1=0; i1 < 4; i1++) {
        dw_DSTATE[i1] = initVector[i1];
      }
    }
  }
 
  {
    real_T initVector[1] = { 0 };
    
    {
      int_T i1;
      for (i1=0; i1 < 1; i1++) {
        m_foc_interface_sts.L_Ident_States = initVector[0];
      }
    }
  }
  

  {
    real_T initVector[1] = { 0 };
    
    {
      int_T i1;
      for (i1=0; i1 < 1; i1++) {
        m_foc_interface_sts.R_flux_Ident_States = initVector[0];
      }
    }
  }

}

