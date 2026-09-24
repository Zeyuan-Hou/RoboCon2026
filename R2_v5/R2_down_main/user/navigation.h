#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "main.h"
#include "Type.h"
#include <string.h>
#include <math.h>
#include "math_algorithm.h"
#include "PID.h"
#include <string.h>
void Cubic_Curve_Set(Cube_Line* cube, float p0, float v0, float p1, float v1, int32_t tim_total);
void Navigate_Task(void);
void Point_to_Point(PATH_POINT *p);
void Cubic_Curve_Calc(float* aim_p, float* aim_v, Cube_Line cube, int32_t tim, int32_t tim_total);
void SET_NAV_PATH_PERMUTATION(void);
#endif
