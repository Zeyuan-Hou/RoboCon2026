#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "ROBOT.h"
#include "math_algorithm.h"
#include "pid.h"
#include "bsp_can.h"
typedef struct
{
	ST_VECTOR rightup, rightdown, leftup, leftdown;
}chassis_velt_t;

typedef struct
{
	fp32 rightup, rightdown, leftup, leftdown;
}chassis_run_des;
//void chasis_solution(ST_VEL *velt);
fp32 Handle_OmnidriectionalWhile(ST_VECTOR *expect_robot_local_velt,ST_VECTOR *pos_motor);
void SpeedDistribute_Four_OmnidriectionalWhile(ST_Nav *p_nav);

extern chassis_run_des friction_compensation_current;
void chassis_friction_compensation(void);
extern chassis_run_des feed_forward_current;
void chassis_feed_forward(chassis_run_des straight_des,chassis_run_des rotation_des);
#endif

