#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "stm32f4xx_hal.h"
#include "can.h"
#include "algorithm.h"

void DJI_CANx_SendstdData(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *pData,uint16_t Len);
float DJI_GetEncoderNumber(ST_MOTOR* motor,uint8_t msg[8]);
float DJI_GetSpeed(CAN_RxHeaderTypeDef* pcanRxMsg, uint8_t msg[8]);
float DJI_GetCurrent(CAN_RxHeaderTypeDef* pcanRxMsg, uint8_t msg[8]);
void DJI_CAN_Sendcurrent(CAN_HandleTypeDef *hcan, uint32_t id, int16_t *current);
void DJI_Abs_Encoder_Process(ST_ENCODER* pEncoder, uint32_t value);
void DJI_MotorCtrl(void);

#endif
