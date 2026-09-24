#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "Type.h"
#include "math_algorithm.h"
#include "pid.h"
#include "CAN_BSP.h"

#define SIN_45 0.707106f
#define RpmToRad 0.10472  //2006单位转换 RPM -> rad/s
#define R_WHEEL_2006 30.0f //2006轮子半径

void GlobalVel_To_Local(ST_VEL *local_vel, ST_VEL *global_vel);

typedef struct
{
	ST_VECTOR wheel_1, wheel_2, wheel_3, wheel_4;
}chassis_velt_t;

void SpeedDistribute_Four_OmnidriectionalWhile(ST_VECTOR *p_nav);

#endif
