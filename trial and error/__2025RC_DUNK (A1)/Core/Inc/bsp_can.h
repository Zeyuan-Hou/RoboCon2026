#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__
#include "can.h"
#include "ROBOT.h"
#include "air_operated.h"
#include "algorithm.h"
void can1_start(void);
void CAN1_FILTER_CONFIG(CAN_HandleTypeDef *hcan);
void can2_start(void);
void CAN2_FILTER_CONFIG(CAN_HandleTypeDef *hcan);
void CANx_SendStdData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *pData, uint16_t Len);
void CAN_SendCurrent(CAN_HandleTypeDef *hcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
float GetEncoderNumber(ST_MOTOR *motor, uint8_t msg[8]);
float GetSpeed_V6(CAN_RxHeaderTypeDef *pCanRxMsg, uint8_t msg[8]);
float GetAngle_V6(CAN_RxHeaderTypeDef *pCanRxMsg, uint8_t msg[8]);
float GetSpeed_DJI ( CAN_RxHeaderTypeDef* pCanRxMsg, uint8_t msg[8]);
void Abs_Encoder_Process(ST_ENCODER *pEncoder, uint32_t value);

uint32_t FloatToUint(const float x, const float x_min, const float x_max, const uint8_t bits);
float UintToFloat(const int x_int, const float x_min, const float x_max, const uint8_t bits);
void FloatsToUints(MotorCMD *param, uint8_t *data);
void UintsToFloats(uint8_t rxdata[8], MotorDATA *data);
void ctrl_motor(CAN_HandleTypeDef *hcan,uint16_t id, MotorCMD *param,uint16_t dlc);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
#endif
