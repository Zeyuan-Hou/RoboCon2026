#ifndef __CANBSP_H__
#define __CANBSP_H__

#include "main.h"
#include "Robot.h"

void can1_start(void) ;
void CAN1_FILTER_CONFIG(CAN_HandleTypeDef* hcan) ;
void can2_start(void) ;
void CAN2_FILTER_CONFIG(CAN_HandleTypeDef* hcan) ;
void CANx_SendstdData(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *pData,uint16_t Len);
//AirOperator
void CAN_Tx_GasValue(CAN_HandleTypeDef *hcan, uint8_t *gas_value);
//DJI
void CAN_Sendcurrent(CAN_HandleTypeDef *hcan, uint32_t id,int16_t current1,int16_t current2,int16_t current3,int16_t current4);
float GetEncoderNumber(ST_DJI_MOTOR* motor,uint8_t msg[8]);
float GetSpeed(CAN_RxHeaderTypeDef* pcanRxMsg,uint8_t msg[8]);
float GetCurrent(CAN_RxHeaderTypeDef* pcanRxMsg,uint8_t msg[8]);
void Abs_Encoder_Process(ST_ENCODER* pEncoder,uint32_t value);



#endif
