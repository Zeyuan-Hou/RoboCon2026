#ifndef __YSC_MOTOR_H__
#define __YSC_MOTOR_H__

#include "stm32f4xx_hal.h"
#include "can.h"
#include "algorithm.h"

void YSC_Motor_CAN_Send_Data(MotorCMD *motor_cmd);
void YSC_FloatsToUints(const MotorCMD *param, uint8_t *data);
void YSC_UintsToFloats(uint8_t *rxdata, MotorDATA *data);
uint16_t YSC_Get_CANID(MotorCMD *cmd);
uint8_t YSC_Get_DLC(MotorCMD *motor_cmd);
void YSC_Motor_Rcv_Data(MotorCMD *motor_cmd,MotorDATA *data,uint8_t *rxdata);
void YSC_SetNormalCMD(MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd);
void YSC_SetMotorCMD(MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd,float torque,float position,float velocity);
void YSC_Motor_Init(MotorCMD *motor_cmd,uint8_t motor_id);
void YSC_MotorCtrl(MotorCMD *motor_cmd, MotorDATA *motor_data);

#endif
