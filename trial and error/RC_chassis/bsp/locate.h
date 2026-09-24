#ifndef __LOCATE_H__
#define __LOCATE_H__

#include "global_declare.h"
#include "algorithm.h"
#include "gyro.h"
#include "math.h"

// DT35距离车中心的距离
#define DT35_X1 0.0f
#define DT35_Y1 0.0f
#define DT35_X2 0.0f
#define DT35_Y2 0.0f

// void Calibrate_Robot_Degree(ST_ROBOT *pstRobot, ST_GYRO *pstGyro);
void Robot_Location(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW);
void DT35_relocation_new(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW, ST_DT35 *p_dt35_save, ST_DT35 *p_dt35_now);
void dt35_relocation(void);
void WheelveltToBodyvelt(void);
void PositionToVelt(void);
void UpdatePositionFeedback(ST_Nav *p_nav, ST_ROBOT *pstRobot);

#endif
