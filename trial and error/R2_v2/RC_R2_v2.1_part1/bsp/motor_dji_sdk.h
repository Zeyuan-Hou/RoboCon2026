#ifndef __MOTOR_DJI_SDK_H__
#define __MOTOR_DJI_SDK_H__

#include "global_declare.h"
#include "algorithm.h"
#include "can_bsp.h"

float GetEncoderNumber_DJI(ST_MOTOR *motor, uint8_t msg[8]);
float GetSpeed_DJI(CAN_RxHeaderTypeDef *pcanRxMsg, uint8_t msg[8]);
float GetCurrent_DJI(CAN_RxHeaderTypeDef *pcanRxMsg, uint8_t msg[8]);
void Abs_Encoder_Process_DJI(ST_ENCODER *pEncoder, uint32_t value);

void DJI_ControlLoop(ST_MOTOR *motor);

#endif
