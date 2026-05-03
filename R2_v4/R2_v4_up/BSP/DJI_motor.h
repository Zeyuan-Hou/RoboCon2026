#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "ROBOT.h"

/**DJI_Func**/
uint8_t DJIMotorStart(void);
void DJIMotorControl(ST_DJI_MOTOR* motor);//电机控制

float GetEncoderNumber(ST_DJI_MOTOR* motor, uint8_t msg[8]);
float GetSpeed(FDCAN_RxHeaderTypeDef *pcanRxMsg, uint8_t msg[8]);
float GetCurrent(FDCAN_RxHeaderTypeDef* pcanRxMsg, uint8_t msg[8]);
void Abs_Encoder_Process(ST_ENCODER* pEncoder, uint32_t value);

#endif
