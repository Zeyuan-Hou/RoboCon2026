#ifndef __QD_H__
#define __QD_H__

#include "stm32f4xx_hal.h"
#include "can.h"
#include "algorithm.h"

void QD_CANx_SendstdData(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *pData,uint16_t Len);
uint8_t Bin_Array_To_u8(uint8_t *bits);
void CAN_SendValveType(CAN_HandleTypeDef *hcan);
void DT35_GET_Data(uint8_t *pData);

#endif
