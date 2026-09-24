#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "global_declare.h"
#include "algorithm.h"
#include "can_bsp.h"

void SpeedDistribute_Four_SteeringWheel(ST_Nav *p_nav);
void Drive_Chassis(void);
void friction_compensation(void);

static float angle_nearest_to(float target_deg, float ref_deg);
static void swerve_optimize(float prev_deg, float *target_deg, float *wheel_vel);

#endif
