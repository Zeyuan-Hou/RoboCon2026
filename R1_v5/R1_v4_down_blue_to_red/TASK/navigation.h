#ifndef ___NAVIGATION_H___
#define ___NAVIGATION_H___

#include "robot.h"
#include "chassis.h"
#include "steer_wheels.h"
#include "route_plan.h"

extern uint8_t noLock_mode;

void Nav_Run(void);
void nav_clear(void);
void nav_pid_calc(void);
uint8_t nav_lockcheck(void);
void Nav_Start(uint16_t spot);
void nav_pid_adjust(void);
void Nav_Start_Test(uint16_t spot, ST_VEL *start_vel, ST_VEL *end_vel );
void Nav_generatePath(void);
void Nav_MAC_Start(uint16_t spot1, uint16_t spot2, uint16_t spot3);
void Nav_Start_withoutLock(uint16_t spot);


#endif // ___NAVIGATION_H___




