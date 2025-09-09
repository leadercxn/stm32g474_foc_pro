#ifndef __EKF_H
#define __EKF_H

#include "boards.h"

typedef struct
{
	float ekf_input[7];     // 输入参数 ，顺序依此是：
							// iq -> pid算出uq后经逆park变换得到的 ualpha, ubeta;
							// 实测电流经 clark 变化后的 ialpha, ibeta,
							// 电阻，电感，磁链常数

	float ekf_states[4];    // 状态变量，顺序依次是：
							// ialpha， ibeta,
							// 估算的角速度 omiga， 转子位置 theta
	
}  ekf_data_def_t;

void apt_ekf_init(void);
_RAM_FUNC void apt_ekf_update(const float *u, float *xd);

#endif  /*__EKF_H*/
