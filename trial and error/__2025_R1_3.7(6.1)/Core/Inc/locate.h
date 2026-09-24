#ifndef __LOCATE_H
#define __LOCATE_H


#include "ROBOT.h"
#include "math_algorithm.h"
#include "gyro.h"
#include "pid.h"
#include "algorithm.h"
//DT35距离车中心的距离
#define DT35_X1 333.63f
#define DT35_Y1 333.63f
#define DT35_X2 333.63f
#define DT35_Y2 333.63f

//void Calibrate_Robot_Degree(ST_ROBOT *pstRobot, ST_GYRO *pstGyro);
void Robot_Location(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW);
void DT35_relocation_new(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW, ST_DT35 *p_dt35_save, ST_DT35 *p_dt35_now);
void UpdatePositionFeedback(ST_Nav *p_nav, ST_ROBOT *pstRobot);
void Mid_360_location(ST_ROBOT *pstRobot);
void dt35_relocation(void);
void PositionToVelt(void);
#endif


