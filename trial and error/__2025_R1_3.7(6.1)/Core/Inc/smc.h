#ifndef __SMC_H
#define __SMC_H
#include "ROBOT.h"
#include "pid.h"
#include "math_algorithm.h"
void CalSMC(ST_SMC *pStSMC);
float SMC_SatFunc(float in, float d);
#endif
