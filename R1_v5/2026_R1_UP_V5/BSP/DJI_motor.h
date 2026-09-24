#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "ROBOT.h"

/**DJI_Func**/
uint8_t DJIMotorStart(void);
void DJIMotorControl(ST_DJI_MOTOR* motor);//电机控制
u8 DJIMotorControlByVision(ST_DJI_MOTOR *motor);
void DJIMotorControl_RecordingToForce(ST_DJI_MOTOR* motor,fp32 checkRange);//电机力反馈控制

void DJIVelControl(ST_DJI_MOTOR* motor);
void DJIVelControlRemote(ST_DJI_MOTOR* motor);//电机控制
	
float GetEncoderNumber(ST_DJI_MOTOR* motor,uint8_t msg[8]);
float GetSpeed(uint8_t msg[8]);
float GetCurrent(uint8_t msg[8]);
void Abs_Encoder_Process(ST_ENCODER* pEncoder,uint32_t value);

#endif
