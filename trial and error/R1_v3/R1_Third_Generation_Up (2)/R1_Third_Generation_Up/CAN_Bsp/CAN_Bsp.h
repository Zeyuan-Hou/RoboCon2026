#ifndef __CAN_Bsp_H__
#define __CAN_Bsp_H__

#include "stm32f4xx_hal.h"
#include "can.h"
#include "algorithm.h"
#include "DJI_Motor.h"
#include "YSC_Motor.h"
#include "DM_Motor.h"
#include "QD.h"

void CAN_BSP_Init(void);
void CAN1_FILTER_CONFIG(void);
void CAN2_FILTER_CONFIG(void);

#endif
