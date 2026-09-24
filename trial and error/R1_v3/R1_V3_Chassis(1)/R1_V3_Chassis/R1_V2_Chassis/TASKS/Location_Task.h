#ifndef __LOCATION_TASK_H__
#define __LOCATION_TASK_H__


#include "HIPNUC_gyro.h"
#include "pid.h"
#include "remote_control_task.h"
#include "algorithm.h"
#include "Vision.h"


 void Follower_Wheel_Location(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW);
void UpdatePositionFeedback(ST_Nav *p_nav, ST_ROBOT *pstRobot);
void WheelveltToBodyvelt(void);
void PositionToVelt(void);
void DT35_gyro_four(ST_ROBOT *pstRobot);
void DT_flag(ST_ROBOT *pstRobot);
void DT35_judge(void);
void DT35_REGION(ST_ROBOT *pstRobot);
void all_locate(void);
#endif
