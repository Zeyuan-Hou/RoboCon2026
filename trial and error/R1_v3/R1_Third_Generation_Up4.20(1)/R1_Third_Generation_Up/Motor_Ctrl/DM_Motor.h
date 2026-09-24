#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "stm32f4xx_hal.h"
#include "can.h"
#include "algorithm.h"

void DM_CANx_SendstdData(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *pData,uint16_t Len);
void DM_Init(void);
void DM_Pack_Control(DM_CMD *control, uint8_t *data);
void DM_Unpack_Feedback(uint8_t *data, DM_DATA *feedback);
void DM_MotorCtrl(CAN_HandleTypeDef *hcan, uint32_t id,DM_CMD *control);
float UintToFloat(const int x_int, const float x_min, const float x_max, const uint8_t bits);
uint32_t FloatToUint(const float x, const float x_min, const float x_max, const uint8_t bits);

#endif
