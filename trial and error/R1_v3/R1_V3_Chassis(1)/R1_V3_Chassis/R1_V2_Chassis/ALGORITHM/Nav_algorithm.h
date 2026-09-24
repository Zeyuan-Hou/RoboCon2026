#ifndef __NAV_ALGORITHM_H__
#define __NAV_ALGORITHM_H__

#include "Types.h"
#include "remote_control_task.h"
#include "Chassis_Task.h"
void NavLineMove_VelocityControl(ST_Nav *p_nav, fp32 v_start, fp32 v_end);
void NavLineMove(ST_Nav *p_nav);
void NavLineMoveWithHeading(ST_Nav *p_nav);
void Nav_Uphill(ST_Nav *p_nav,Speed*speed);
void Nav_Rotation(ST_Nav *p_nav,float angle);
void Nav_PID_Adjust(void);



#endif




