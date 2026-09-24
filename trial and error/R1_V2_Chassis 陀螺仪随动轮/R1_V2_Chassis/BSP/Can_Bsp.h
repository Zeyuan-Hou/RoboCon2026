#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__

#include "Types.h"
#include "pid.h"
#include "string.h"
#include "Crane_3508_Ctrl.h"
#include "can.h"
#include "QD.h"

void can1_start(void) ;
void CAN1_FILTER_CONFIG(CAN_HandleTypeDef* hcan) ;
void can2_start(void) ;
void CAN2_FILTER_CONFIG(CAN_HandleTypeDef* hcan) ;
void motor_SendCurent(uint32_t CANID, int16_t vol1, int16_t vol2, int16_t vol3, int16_t vol4);
void CAN_Sendcurrent(CAN_HandleTypeDef *hcan, uint32_t id,int16_t current1,int16_t current2,int16_t current3,int16_t current4);
void CANx_SendstdData(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *pData,uint16_t Len);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void Abs_Encoder_Process(ST_ENCODER *motor_encoder,FP32 encoder_pos);
void Motor_GetFeedback(MOTOR *motor, uint8_t rx_data[8]);
float GetEncoderNumber_(ST_MOTORT* motor,uint8_t msg[8]);
float GetSpeed(CAN_RxHeaderTypeDef* pcanRxMsg,uint8_t msg[8]);
#endif
