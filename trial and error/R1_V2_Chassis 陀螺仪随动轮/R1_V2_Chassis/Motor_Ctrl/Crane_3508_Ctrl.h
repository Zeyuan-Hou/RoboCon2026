#ifndef ___CRANE_3508_CTRL_H___
#define ___CRANE_3508_CTRL_H___

#include "Can_Bsp.h"
#include "pid.h"


void Crane_3508_Ctrl(void);
void Crane_3508_Calc(void);
float getCubicCurveY_1(float x1, float y1, float x2, float y2);
float getCubicCurveY_2(float x1, float y1, float x2, float y2);
void M3508_AngleSmoothTransition(uint16_t time);
extern float posi_1,posi_2;
#endif
