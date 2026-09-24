#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "Algorithm.h"
#include "Global_Variables.h"

/**DJI_Func**/
void DJIMotorControl(ST_DJI_MOTOR* motor);//电机控制
float GetEncoderNumber(ST_DJI_MOTOR* motor, uint8_t msg[8]);
float GetSpeed(uint8_t msg[8]);
float GetCurrent(uint8_t msg[8]);
void Abs_Encoder_Process(ST_ENCODER* pEncoder, uint32_t value);
void DJI_RxData(ST_DJI_MOTOR* motor, uint8_t msg[8]);

#endif
