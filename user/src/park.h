#ifndef PARK_H__
#define PARK_H__

#include "foc.h"

void park_cal(foc_param_t *foc_param);
void inv_park_cal(foc_param_t *foc_param);
void park_cal_param(float alpha, float beta, float pos_radian, float *d, float *q);
void inv_park_cal_param(float d, float q, float pos_radian, float *alpha, float *beta);

#endif
