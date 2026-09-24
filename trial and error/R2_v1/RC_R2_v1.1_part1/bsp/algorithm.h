#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

#include "global_declare.h"
#include <math.h>

void LpFilter(ST_LPF *lpf);

void PID_Calc(ST_PID *pid, float reference, float feedback);
void Cascade_PID_Calc(ST_CASCADE_PID *cascade_pid, float outer_reference, float outer_feedback);
void Cascade_PID_TD_Calc(ST_CASCADE_PID *cascade_pid, float outer_reference, float outer_feedback);
float Sgn(float x);
void CalTD(ST_TD *pStTD);

float ConvertAngle(float fpAngA);
void Covert_coordinate(ST_VECTOR *a);
void Convert_velt(ST_VECTOR *global, ST_VECTOR *local, float fpQ);

float ClipFloat(float fpValue, float fpMin, float fpMax);
float Geometric_mean(float a, float b);
float normalize_angle(float angle_deg);

void ramp_signal(float *p_Output, float DesValue, float Step);

float uint_to_float(const int x_int, const float x_min, const float x_max, const int bits);
int float_to_uint(const float x, const float x_min, const float x_max, const int bits);

float feedforward_curve_sin(float goal_pos, float current_pos, int interval_ms, int time_ms);
float feedforward_linear(float goal_pos, float current_pos, int interval_ms, int time_ms);
float feedforward_G_tor(float tor_k, float angle);

void PID_Calc_Pos(ST_PID *pStPID);
void PID_Calc_Angle(ST_PID *pStPID);
#endif
