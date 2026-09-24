#ifndef __ALGORITHM_H
#define __ALGORITHM_H
#include "math_algorithm.h"
#include "Types.h"
#include "pid.h"
float AdaptiveLPF_Update(AdaptiveLPF *f, float feedback, float target);
void ramp_signal(float* p_Output, float DesValue, float Step);
void set_bit1(u8 *data ,u8 n);
void set_bit0(u8 *data ,u8 n);
u8 get_bit(u8 data ,u8 n);

float plan_of_time(float start_pos, float end_pos, float time);
//void s_curve_ramp_signal(float* p_Output, float DesValue, float Step, float TimeStep);
void s_curve_ramp_signal(float* p_Output, float DesValue, float Step, float Curve);

void LpFilter(ST_LPF *lpf);
float KalmanUpdate(KalmanFilter* kf, float v_ins, float v_whl);

//void J60_motor_control(void);
//void G_vofa_watch(void);
#endif


