#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

#include "global_declare.h"
#include <math.h>

void LpFilter(ST_LPF *lpf);

void PID_Calc(ST_PID *pid, float reference, float feedback);
void Cascade_PID_Calc(ST_CASCADE_PID *cascade_pid, float outer_reference, float outer_feedback);
void Cascade_PID_TD_Calc(ST_CASCADE_PID *cascade_pid, float outer_reference, float outer_feedback);
void PI_Feedforward_Calc(ST_PI_Feedforward *pi_ff, float reference, float feedback);
void PID_Fuzzy_Calc(ST_PID_Fuzzy *pStPID, float reference, float feedback);
float Sgn(float x);
void CalTD(ST_TD *pStTD);

float ConvertAngle(float fpAngA);
void Covert_coordinate(ST_VECTOR *a);
void Convert_Velt_Global2Local(ST_VECTOR global, ST_VECTOR *local, float fpQ);
void Convert_Velt_Local2Global(ST_VECTOR *global, ST_VECTOR local, float fpQ);

float ClipFloat(float fpValue, float fpMin, float fpMax);
float Geometric_mean(float a, float b);

void ramp_signal(float *p_Output, float DesValue, float Step);
void Cubic_Curve_Set(Cube_Line* cube, float p0, float v0, float p1, float v1, int32_t tim_total);
void Cubic_Curve_Calc(float* aim_p, float* aim_v, Cube_Line cube, int32_t tim, int32_t tim_total);

void Quintic_Curve_Set(Quintic_Line *quintic,
                       float p0, float v0, float a0,
                       float p1, float v1, float a1,
                       int32_t tim_total);
void Quintic_Curve_Calc(float *aim_p,
                        float *aim_v,
                        float *aim_a,
                        Quintic_Line quintic,
                        int32_t tim,
                        int32_t tim_total);

int32_t Nav_Time_Calc(uint8_t inx, int32_t v_s);
void Trapezoid_Curve_Calc(float *aim_p, float *aim_v, float p0, float p1, float v_target, int32_t tim, int32_t tim_total);
void Bezier_3rd_Set(Bezier_3rd *bezier, ST_VECTOR *p0, ST_VECTOR *v0, ST_VECTOR *p1, ST_VECTOR *v1, int32_t tim_total);
void Bezier_3rd_Calc(Bezier_3rd bezier, float *aim_px, float *aim_py, float *aim_yaw, float *aim_vx, float *aim_vy, float *aim_vyaw, int32_t tim, int32_t tim_total);
float Angle_Limit(float delta_a);

float uint_to_float(const int x_int, const float x_min, const float x_max, const int bits);
int float_to_uint(const float x, const float x_min, const float x_max, const int bits);

float feedforward_curve_sin(float goal_pos, float current_pos, int interval_ms, int time_ms);
float feedforward_linear(float goal_pos, float current_pos, int interval_ms, int time_ms);
float feedforward_G_tor(float tor_k, float angle);

void PID_Calc_Pos(ST_PID *pStPID);
void PID_Calc_Angle(ST_PID *pid, float reference, float feedback);
#endif
