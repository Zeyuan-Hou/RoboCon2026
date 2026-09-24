#ifndef __PID_ALGORITHM_H__
#define __PID_ALGORITHM_H__

#include "Robot.h"

void PID_Init(ST_PID *pid, float p, float i, float d, float EMin, float EMax, float SumEMax, float UMax, float UpMax, float UdMax);
void PID_Calc_NEW(ST_PID *pStPID);
void PID_Calc(ST_PID *pid,float fpDes,float fpFB);
float Sgn(float x);
void CalTD(ST_TD *pStTD);

#endif
