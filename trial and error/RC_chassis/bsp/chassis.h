#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "global_declare.h"
#include "algorithm.h"

typedef struct
{
    ST_VECTOR rightup, rightdown, leftup, leftdown;
} chassis_velt_t;

typedef struct
{
    float rightup, rightdown, leftup, leftdown;
} chassis_run_des;

extern chassis_run_des friction_compensation_current;
extern chassis_run_des feed_forward_current;

void chassis_friction_compensation(void);
void SpeedDistribute_Four_OmnidirectionalWheel(ST_Nav *p_nav);
void chassis_feed_forward(chassis_run_des straight_des, chassis_run_des rotation_des);

void SpeedDistribute_Four_SteeringWheel(ST_Nav *p_nav);
void steer_chassis_feed_forward(void);
//float normalize_angle(float angle_deg);

#endif
