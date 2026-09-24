#ifndef __bsp_can_H__
#define __bsp_can_H__

#include "can.h"
#include "global_declare.h"
#include "algorithm.h"
#include <string.h>
#include "motor_dji_sdk.h"
#include "j60_sdk.h"

void can1_start(void);
void CAN1_FILTER_CONFIG(CAN_HandleTypeDef *hcan);
void can2_start(void);
void CAN2_FILTER_CONFIG(CAN_HandleTypeDef *hcan);
void CAN_SendCurrent(CAN_HandleTypeDef *hcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CANx_SendStdData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *pData, uint16_t Len);

void TravelSwitchDataDeal(const uint8_t travel_switch, uint8_t *key_states);
void CAN_SendData_J60(CAN_HandleTypeDef *hcan, J60_MotorCMD *data);

#endif
