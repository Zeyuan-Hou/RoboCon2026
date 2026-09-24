#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "Robot.h"

/**DJI_Func**/
uint8_t DJI_Pos_Ctrl(ST_DJI_MOTOR *pstMotor,fp32 tPos);
uint8_t DJI_Pos_CtrlWithoutPID(ST_DJI_MOTOR *pstMotor,fp32 tPos);
uint8_t DJIMotorStart(void);

uint8_t wristCtrl_L(fp32 roll_t,fp32 pitch_t,lilWrist* pstWrist);
uint8_t wristCtrl_R(fp32 roll_t,fp32 pitch_t,lilWrist* pstWrist);

#endif
