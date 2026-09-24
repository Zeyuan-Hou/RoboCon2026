#ifndef __LOGIC_H__
#define __LOGIC_H__

#include "CAN_Bsp.h"
#include "usart.h"


uint8_t BlockArm_State_Judge(float Joint1,float Joint2,float Joint3,float Gimbal);
uint8_t BlockArm_BackSolve(ARM_BACKSOLVING *params);
void BlockArm_InputSolve(ARM_BACKSOLVING *params,MOTORINPUT *Input);
void BlockArm_Init(void);
void BlockArm_Logic(void);

void Air_Pump_Control(void);

void  PoleArm_Adjustment(void);
uint8_t PoleArm_State_Judge(float Joint1, float Joint2, float FrictionWheel);//判断是否到达目标位置的函数，利用位置容差判断
void PoleArm_Task_TD_Calc(void);//在三个move状态下计算跟随目标的函数并给motorinput赋值
void PoleArm_Logic(void);

void Communicate_Task(void);
void USART1_Receive_Data(uint8_t Communicate_KEY);
void USART1_Transmit_Data(void);

void Motor_CAN_Send(void);

#endif
