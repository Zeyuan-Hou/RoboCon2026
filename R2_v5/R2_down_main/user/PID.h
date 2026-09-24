#ifndef _PID_H_
#define _PID_H_

#include <stdio.h>
#include <math.h>
#include "main.h"
#include "Type.h"
#include "math_algorithm.h"

void PID_Calc(ST_PID *pStPID, float target, float feedback);
void PID_Calc_NEW(ST_PID *pStPID);
int Sgn(float x) ;
void CalTD(ST_TD *pStTD);
void PID_CascadeCalc(ST_CascadePID *pStCasPID, float outer_target, float outer_feedback, float inner_feedback);
void PID_CascadeCalc_special(ST_CascadePID *pStCasPID, float outer_target, float outer_feedback, float inner_feedback);
void PID_Calc_NEW_wheel(ST_PID *pStPID);
void PID_Calc_Angle(ST_PID *pid, float reference, float feedback);
#endif
