#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "global_declare.h"
#include "algorithm.h"
#include "can_bsp.h"

typedef struct{
    void (*Enter)(void);
    void (*Execute)(void);
    void (*Exit)(void);
}CHASSIS_STATUS;

extern CHASSIS_STATUS CS_lowspeed;
extern CHASSIS_STATUS CS_highspeed;
extern CHASSIS_STATUS CS_pos;
extern CHASSIS_STATUS *CS_curstatus;
void CS_Change(CHASSIS_STATUS *newstatus);
void CS_Run(void);

void SpeedDistribute_Four_SteeringWheel(ST_Nav *p_nav);
void Drive_Chassis(void);
void friction_compensation(void);

static float angle_nearest_to(float target_deg, float ref_deg);
static void swerve_optimize(float prev_deg, float *target_deg, float *wheel_vel);

#endif
