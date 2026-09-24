#ifndef __PID_H
#define __PID_H
#include "ROBOT.h"
#include "math_algorithm.h"
void PID_Calc(ST_PID *pStPID, float target, float feedback);
void PID_Calc_NEW(ST_PID *pStPID);
void PID_CascadeCalc(ST_CascadePID *pStCasPID, float outer_target, float outer_feedback, float inner_feedback);
void CalTD(ST_TD *pStTD);
int Sgn(float x) ;
#endif


