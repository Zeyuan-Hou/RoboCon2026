#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

#include <math.h>
#include "Global_Variables.h"

void PID_Init(ST_PID *pid, float p, float i, float d, float EMin, float EMax, float SumEMax, float UMax, float UpMax, float UdMax);
void PID_Calc(ST_PID *pid, float fpDes, float fpFB);
float Sgn(float x);
void CalTD(ST_TD *pStTD);

float ClipFloat(float fpValue, float fpMin, float fpMax);

float linear_target_curve(float init_pos, float target_pos, int32_t tim, int32_t total_tim);
float sin_target_curve(float init_pos, float target_pos, int32_t tim, int32_t total_tim);

uint8_t bin_array_to_u8(uint8_t *bits);

#endif
