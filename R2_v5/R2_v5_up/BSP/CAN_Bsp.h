#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__


#include "main.h"
#include "fdcan.h"
#include "DM_Motor.h"
#include "J60_Motor.h"
#include "DJI_Motor.h"
#include "Robstride.h"

void CAN_INIT(void);
void bsp_fdcan1_init(void);
void bsp_fdcan2_init(void);
void bsp_fdcan3_init(void);
void CAN_bus_off_check_reset(FDCAN_HandleTypeDef *hfdcan);
void CANx_SendstdData(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *pData, uint16_t Len);

void CAN_Sendcurrent(FDCAN_HandleTypeDef *hfdcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void LimitSwitchDataDeal(const uint8_t travel_switch, uint8_t *key_states);

#endif /* __FDCAN_H__ */
