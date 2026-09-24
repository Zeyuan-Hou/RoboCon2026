#ifndef __bsp_can_H__
#define __bsp_can_H__

#include "can.h"
#include "global_declare.h"
#include "algorithm.h"
#include "stdint.h"
#include "string.h"

void can1_start(void);
void CAN1_FILTER_CONFIG(CAN_HandleTypeDef *hcan);
void can2_start(void);
void CAN2_FILTER_CONFIG(CAN_HandleTypeDef *hcan);
void CANx_SendStdData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *pData, uint16_t Len);

void CAN_SendCurrent_V6(CAN_HandleTypeDef *hcan, uint32_t id, uint16_t current, uint32_t motorID);
float GetSpeed_V6(CAN_RxHeaderTypeDef *pCanRxMsg, uint8_t msg[8]);
float GetAngle_V6(CAN_RxHeaderTypeDef *pCanRxMsg, uint8_t msg[8]);

void CAN_SendCurrent(CAN_HandleTypeDef *hcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
float GetEncoderNumber_DJI(ST_MOTOR *motor, uint8_t msg[8]);
float GetSpeed_DJI(CAN_RxHeaderTypeDef *pcanRxMsg, uint8_t msg[8]);
float GetCurrent_DJI(CAN_RxHeaderTypeDef *pcanRxMsg, uint8_t msg[8]);
void Abs_Encoder_Process_DJI(ST_ENCODER *pEncoder, uint32_t value);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
