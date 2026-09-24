#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"
#include "main.h"
#include "Robot.h"
#include "stm32h7xx_hal.h"



void CAN_INIT(void);
void bsp_fdcan1_init(void);
void bsp_fdcan2_init(void);
void bsp_fdcan3_init(void);
void CAN_SendStdData(FDCAN_HandleTypeDef* hfdcan, uint16_t ID, uint8_t *pData, uint16_t Len);
void CAN_Sendcurrent(FDCAN_HandleTypeDef *hfdcan, uint32_t id,int16_t current1,int16_t current2,int16_t current3,int16_t current4);
void CAN_bus_off_check_reset(FDCAN_HandleTypeDef *hfdcan);

#ifdef __cplusplus
}
#endif

#endif
