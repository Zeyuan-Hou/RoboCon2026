#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "fdcan.h"
#include "global_declare.h"
#include <string.h>

void CAN_INIT(void);
void bsp_fdcan1_init(void);
void bsp_fdcan2_init(void);
void bsp_fdcan3_init(void);

void CAN_SendStdData(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *pData, uint16_t Len);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

void CAN_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void TravelSwitchDataDeal(const uint8_t travel_switch, uint8_t *key_states);

#endif /* __FDCAN_H__ */
