#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

#include "global_declare.h"
#include "math.h"

void LpFilter(ST_LPF *lpf);
void PID_Calc(ST_PID *pid, float reference, float feedback);
float Sgn(float x);
void CalTD(ST_TD *pStTD);
void Covert_coordinate(ST_VECTOR *a);
void Concert_coorindnate(ST_VECTOR *global, ST_VECTOR *local, float fpQ);
float ConvertAngle(float fpAngA);
float ClipFloat(float fpValue, float fpMin, float fpMax);
float Geometric_mean(float a, float b);
float KalmanUpdate(KalmanFilter *kf, float v_ins, float v_whl);
void ramp_signal(float *p_Output, float DesValue, float Step);

#endif
