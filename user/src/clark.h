#ifndef CLARK_H__
#define CLARK_H__

#include "foc.h"

void clark_cal(foc_param_t *foc_param);
void inv_clark_cal(foc_param_t *foc_param);
void clark_cal_param(float u, float v, float w, float *alpha, float *beta);
void inv_clark_cal_param(float alpha, float beta, float *u, float *v, float *w);

#endif
