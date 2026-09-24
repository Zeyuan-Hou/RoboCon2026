#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__

#include "main.h"
#include "fdcan.h"
#include "Type.h"
#include "math_algorithm.h"

#define BUFFER_SIZE 8
extern uint8_t Can_rxBuffer1[BUFFER_SIZE];
extern uint8_t Can_rxBuffer2[BUFFER_SIZE];
extern uint8_t Can_rxBuffer3[BUFFER_SIZE];

typedef struct
{
	uint16_t cob_id;	// msg id
	uint8_t rtr;			//帧模式
	uint8_t len;			//数据长度
	uint8_t data[8];	//数据域
}bsp_can_msg_t;


void bsp_fdcan1_init(void);
void bsp_fdcan2_init(void);
void bsp_fdcan3_init(void);
void CAN_INIT(void) ;

void CAN_SendStdData(FDCAN_HandleTypeDef* hfdcan, uint16_t ID, uint8_t *pData, uint16_t Len);
void CAN_SendCurrent(FDCAN_HandleTypeDef *hcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
float GetSpeed_V6(uint8_t msg[8]);
float GetAngle_V6_test( uint8_t msg[8],Angle_Processor *pangle);
void TravelSwitchDataDeal(const uint8_t travel_switch, uint8_t *key_states);
void Abs_Encoder_Process(ST_ENCODER* pEncoder,uint32_t value);
float GetEncoderNumber(ST_MOTOR* motor,uint8_t msg[8]);
float GetSpeed(uint8_t msg[8]);
uint8_t bin_array_to_u8(uint8_t *bits);
void CAN_bus_off_check_reset(FDCAN_HandleTypeDef *hfdcan);
#endif
