#ifndef __LK_MOTOR_DRIVER_H__
#define __LK_MOTOR_DRIVER_H__
#include "CANbsp.h"
#include "Robot.h"

void LK_PID_Set(void);
uint8_t LK_Pos_Ctrl(Motor_LK_8016 *pstMotor,fp32 tPos);
uint8_t LK_Pos_CtrlWithoutPID(Motor_LK_8016 *pstMotor,fp32 tPos);

void LK_8016_Close(CAN_HandleTypeDef *hcan, uint32_t id);
void CAN_8016_Open(CAN_HandleTypeDef *hcan,u32 id);
void CAN_8016_SendCurrent_Single(CAN_HandleTypeDef *hcan,u32 id,s16 current);
void CAN_8016_SendCurrent(CAN_HandleTypeDef *hcan,u32 id,s16 current1,s16 current2,s16 current3,s16 current4);

void LK_8016_DataProcess(uint8_t * CAN_RX_BUF, Motor_LK_8016 *LK_Motor);
void LK_8016_zero_set(CAN_HandleTypeDef *hcan,u32 id);
#endif
